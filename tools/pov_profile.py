#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pov_profile.py — 主机侧**几何/传输口径 (profile)** + VXT1 体素表编解码 (2026-10-10).

为什么要有它: 现役 2047 管线的口径 (160×180 / 360 片 / 1-bit·3-bit 切片 / PHASE_B=180)
散落在 povstream.py / pack_obs.py / gen_anime_slices.py 的常量里。验证板换成
ICND2260 + DR1M90 之后口径变了 (358 片 / RGB565 / 传体素表不传切片), 不能再靠改
常量切换 —— 两套必须同时可用 (2047 回归 + 2260 新口径, 见 20 日计划 D16)。

  profile   transport  面板      片/圈  转速      体帧率  色           面B 相位
  2047      slices     160×180   360   ~969 RPM  —       1/3-bit 切片  180 (RTL PHASE_B)
  vb358     voxtab     160×180   358   720 RPM   24 Hz   RGB565 体素   179 (= 358/2)
  tgt603    voxtab     192×216   603   1050 RPM  35 Hz   RGB565 体素   301 (⚠ 603 奇数)

🔴 transport='voxtab' 时主机**只发体素表**, 切片由板端 (DR1M90) 按 profile 的
   n_slices / 面垂距 / mirror / 相位 自己切。所以 profile 里的几何量对主机来说只是
   **元数据** (写进 meta.json, 给离线对拍和板端核对), 主机发的字节与片数无关。

── VXT1 线上格式 ─────────────────────────────────────────────────────────────
权威定义: zynq_pov/scatter/vox_table.h。这里是它的 Python 实现 (发送端 + 测试用解码)。
  PVS1 头: flags = PVS_FLAG_VOXTAB(1<<8) | 压缩位, n_slices = 0, raw_len = VXT1 体长
  VXT1 体: [24 B 头][颜色平面 n×u16 RGB565][索引平面 varint(delta-1)]
  网格布局 idx = (vz*XZ + vx)*Y + vy, **y 最内维且与面板行号同向 (自上而下)**,
  ⇒ 体素 y 向上的 gas 坐标系写网格时 vy = (Y-1) - gy。
🔴 PVS_FLAG_VOXTAB 是 **bit 8**: 原设计 (08-05) 用 bit 7, 后来 3-bit 色深 (08-20)
   把 bit 7 占成 PVS_FLAG_3BIT ⇒ 撞位。2026-10-10 挪到 bit 8 (头里 flags 是 u16)。
"""
import math
import struct

import numpy as np

# ─────────────────────────── profile 表 ─────────────────────────────────────
# faces: 每面 (名, 到转轴垂距 mm, mirror_u, 相位 slot 偏移)。
#   相位语义: 该面第 g 槽对应角度 θ = 2π·(g + phase)/n_slices。
#   面B phase = n/2 ⇔ 2047 现役的 RTL PHASE_B=180 (+ 渲染侧面B 垂距取 +13.4mm、
#   mirror 与面A 相同), 物理等价于 "面B 在 θ 处、垂距 −13.4mm、手性相反"
#   (见 memory project_pov3d_v31_dualface_geometry_solved)。
#   🔴 这个偏移在整条链路里**只能施加一次**: 板端要么切片时 phase=n/2 且 RTL
#   PHASE_B=0, 要么切片 phase=0 且 RTL PHASE_B=n/2 —— 两处都加会再转半圈。
PROFILES = {
    '2047': dict(
        transport='slices', W=160, H=180, pitch_mm=0.9375, n_slices=360,
        rpm=None, vol_hz=None, color='1bit/3bit 切片 (pack_obs 布局)',
        faces=(('A', 0.0, True, 0), ('B', 13.4, True, 180)),
        note='现役 7020 + ICND2047 偏心双面; 默认路径, 不给 --profile 时逐字节等价'),
    'vb358': dict(
        transport='voxtab', W=160, H=180, pitch_mm=0.9375, n_slices=358,
        rpm=720, vol_hz=24, color='RGB565 体素表 (VXT1)',
        faces=(('A', 0.0, True, 0), ('B', 13.4, True, 179)),
        note='2260 验证板 (2026-10-10 用户定): 60 扫 1串2, 4,296 slice/s/面'),
    'tgt603': dict(
        transport='voxtab', W=192, H=216, pitch_mm=0.78125, n_slices=603,
        rpm=1050, vol_hz=35, color='RGB565 体素表 (VXT1)',
        faces=(('A', 0.0, True, 0), ('B', 13.4, True, 301)),
        note='量产目标 (阶段②, 未验收); ⚠ 603 为奇数, 面B 半圈相位差半槽 (0.30°)'),
}


def get_profile(name):
    if name not in PROFILES:
        raise SystemExit(f'未知 profile {name!r}, 可选: {", ".join(PROFILES)}')
    p = dict(PROFILES[name])
    p['name'] = name
    return p


def face_params(prof):
    """→ [dict(name, axis_off_mm, axis_off_px, axis_off_q8, mirror_u, phase)]。
    axis_off_q8 = pov_slice_v2 的 axis_off_q8 参数 (像素 << 8)。"""
    out = []
    for name, mm, mirror, phase in prof['faces']:
        px = mm / prof['pitch_mm']
        out.append(dict(name=name, axis_off_mm=mm, axis_off_px=round(px, 4),
                        axis_off_q8=int(round(px * 256)), mirror_u=int(bool(mirror)),
                        phase=int(phase)))
    return out


def describe(prof):
    n = prof['n_slices']
    s = (f'profile {prof["name"]}: {prof["transport"]} {prof["W"]}×{prof["H"]} '
         f'pitch {prof["pitch_mm"]}mm, {n} 片/圈')
    if prof.get('rpm'):
        rps = prof['rpm'] / 60.0
        s += (f', {prof["rpm"]} RPM = {rps:g} rev/s → 体帧率 {2 * rps:g} Hz, '
              f'{n * rps:,.0f} slice/s/面')
    s += '; ' + ' / '.join(f'面{f["name"]} {f["axis_off_mm"]}mm={f["axis_off_px"]}px'
                           f'(q8 {f["axis_off_q8"]}) mirror={f["mirror_u"]} '
                           f'phase={f["phase"]}' for f in face_params(prof))
    return s


def meta_dict(prof):
    """写进预渲染目录 meta.json 的 profile 段 (板端/离线对拍按它切)。"""
    return dict(name=prof['name'], transport=prof['transport'], W=prof['W'], H=prof['H'],
                pitch_mm=prof['pitch_mm'], n_slices=prof['n_slices'], rpm=prof.get('rpm'),
                vol_hz=prof.get('vol_hz'), faces=face_params(prof))


# ─────────────────────────── VXT1 ───────────────────────────────────────────
FLAG_VOXTAB = 0x0100            # PVS1 头 flags bit 8 (= vox_table.h PVS_FLAG_VOXTAB)
VXT_MAGIC = b'VXT1'
VXT_HDR = struct.Struct('<4sHHIIHHI')     # 24 B, = vxt_hdr_t
VXT_FLAG_KEY = 1 << 0


def rgb565_from_float(vox):
    """(…,3) float 0..255 → uint16 RGB565 (8-bit 先四舍五入再截高位, 同
    zynq_pov/tools/glb_voxel_payload.py)。全黑 (=0) 的格即"不占据"。"""
    c = np.clip(np.rint(vox), 0, 255).astype(np.uint16)
    return ((c[..., 0] >> 3) << 11) | ((c[..., 1] >> 2) << 5) | (c[..., 2] >> 3)


def grid_from_gas(v565):
    """gas 坐标 (gx, gy↑, gz) 的 RGB565 体 → pov_slice_v2 布局的一维网格。
    两件事: 轴序 → (vz, vx, vy); **y 翻转** vy = (Y-1) - gy。"""
    g = v565[:, ::-1, :]                       # y 翻转 (体素 y 向上 → 行号向下)
    return np.ascontiguousarray(np.transpose(g, (2, 0, 1))).reshape(-1)


def varint_encode(v):
    v = np.asarray(v, np.uint32)
    if v.size == 0:
        return b''
    nb = np.ones(v.size, np.int8)
    nb[v >= (1 << 7)] = 2
    nb[v >= (1 << 14)] = 3
    nb[v >= (1 << 21)] = 4
    cols = np.stack([((v >> (7 * k)) & 0x7F).astype(np.uint8) for k in range(4)], 1)
    for k in range(3):
        cols[:, k] |= np.where(nb > k + 1, 0x80, 0).astype(np.uint8)
    return cols[np.arange(4)[None, :] < nb[:, None]].tobytes()


def varint_decode(buf, n):
    """LEB128 流 → n 个 uint32 (向量化; 流必须恰好用完, 否则 ValueError)。"""
    b = np.frombuffer(buf, np.uint8)
    ends = np.flatnonzero(b < 0x80)
    if ends.size != n or (n and ends[-1] != b.size - 1):
        raise ValueError(f'varint 流: {ends.size} 个值 / {b.size} B, 期望 {n} 个且恰好用完')
    starts = np.concatenate([[0], ends[:-1] + 1])
    pos = np.arange(b.size) - np.repeat(starts, ends - starts + 1)
    if pos.size and pos.max() > 3:
        raise ValueError('varint 超过 4 字节 (索引必 < 2^28)')
    vals = (b & 0x7F).astype(np.uint64) << (7 * pos).astype(np.uint64)
    return np.add.reduceat(vals, starts).astype(np.uint32) if n else np.zeros(0, np.uint32)


def encode_vxt1(grid, XZ, Y, seq=0, key=False):
    """一维网格 (pov_slice_v2 布局, uint16) → VXT1 体字节。占据 = 非零格。"""
    grid = np.asarray(grid, np.uint16)
    if grid.size != XZ * XZ * Y:
        raise ValueError(f'网格 {grid.size} 格 != {XZ}×{XZ}×{Y}')
    idx = np.flatnonzero(grid).astype(np.int64)        # 天然升序
    d = np.diff(idx, prepend=-1) - 1                    # 约定 idx[-1] = -1
    ip = varint_encode(d.astype(np.uint32))
    hdr = VXT_HDR.pack(VXT_MAGIC, XZ, Y, idx.size, len(ip),
                       VXT_FLAG_KEY if key else 0, seq & 0xFFFF, 0)
    return hdr + grid[idx].astype('<u2').tobytes() + ip


def decode_vxt1(body):
    """VXT1 体 → (hdr dict, idx uint32[], col uint16[])。校验同 vox_scatter_apply。"""
    if len(body) < VXT_HDR.size:
        raise ValueError('VXT1 体短于 24 B 头')
    magic, xz, y, n, ib, flags, seq, _ = VXT_HDR.unpack_from(body)
    if magic != VXT_MAGIC:
        raise ValueError(f'魔数 {magic!r} != VXT1')
    if len(body) != VXT_HDR.size + 2 * n + ib:
        raise ValueError(f'长度 {len(body)} != 24 + 2×{n} + {ib}')
    col = np.frombuffer(body, '<u2', n, VXT_HDR.size)
    d = varint_decode(body[VXT_HDR.size + 2 * n:], n)
    idx = (np.cumsum(d.astype(np.int64) + 1) - 1).astype(np.int64)
    if n and idx[-1] >= xz * xz * y:
        raise ValueError(f'索引越界 {idx[-1]} ≥ {xz * xz * y}')
    return (dict(grid_xz=xz, grid_y=y, n_vox=n, idx_bytes=ib, flags=flags, seq=seq),
            idx.astype(np.uint32), col)


def display_radius_px(prof):
    """板端切片可能采到的最大体素半径 (体素中心到转轴, px)。
    面上最外列 |u| ≤ W/2, 偏移面半径 √(u²+d²); 再加 1 px 盖住 Q8 角度表误差
    (≤0.2 px) 与取整 (≤0.5·√2)。超出的体素**永远不会被显示**。"""
    d = max(abs(f['axis_off_px']) for f in face_params(prof))
    return math.hypot(prof['W'] / 2.0 + 0.5, d) + 1.0


def cylinder_mask(prof):
    """(gx, gz) 平面上 '可显示' 掩码 (W, W)。体素中心世界坐标 = 索引 - W/2
    (与 pov_slice_v2 的 rx = ... + PV2_VOX_HALF 一致)。"""
    W = prof['W']
    c = np.arange(W) - W / 2.0
    return np.hypot(c[:, None], c[None, :]) <= display_radius_px(prof)


def vxt1_from_gas_float(vox, prof, seq=0, key=False, crop=True):
    """gas 体素 (GR, GH, GR, 3) float → VXT1 体。尺寸必须与 profile 一致。
    crop=True: 丢掉显示圆柱外的体素 (gas 归一化按方盒 ±79, 四角最远到 ~111 px,
    板端永远切不到) —— 只省线上字节, 显示结果逐字节不变 (vb358_e2e.py 验证)。"""
    W, H = prof['W'], prof['H']
    if vox.shape[:3] != (W, H, W):
        raise SystemExit(f'体素格 {vox.shape[:3]} 与 profile {prof["name"]} 的 '
                         f'{W}×{H}×{W} 不符 (povstream 的动画源固定 160×180)')
    v565 = rgb565_from_float(vox)
    if crop:
        v565 = np.where(cylinder_mask(prof)[:, None, :], v565, 0).astype(np.uint16)
    return encode_vxt1(grid_from_gas(v565), W, H, seq, key)


if __name__ == '__main__':
    for k in PROFILES:
        print(describe(get_profile(k)))
