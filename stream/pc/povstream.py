#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
povstream.py — PC 侧 POV 体显示推流器 (PVS1 协议, 见 protocol.md / ../protocol.h).

frame = 360 slices × 0x3000 = 4,423,680B (pack_obs 硬件实测映射, 不可改).
管线: 动画源 → 逐帧点云变换 → 体素化 → 360 切片渲染 → 1-bit Bayer 抖动
(相位随 slice+frame 双变化, 时间抖动平滑) → pack → zlib → TCP → 板 ACK.

numpy 现渲 ~秒级/帧, 正常流程先 render 预渲染到磁盘再 stream:

  python3 povstream.py render --anim spinpulse --frames 8 --render-slices 90
  python3 povstream.py render --anim glbseq --glb-dir my_seq/ --render-slices 90
  python3 povstream.py render --anim glbanim --glb walk.glb --anim-take 0 --frames 12
  python3 povstream.py stream --dir frames_spinpulse --host <board> --fps 10 --loop
  python3 povstream.py stream --anim globe --frames 60 --loop   # 现渲直推 (慢)
  python3 povstream.py bench                                    # 压缩测量

动画源:
  spinpulse: anime GLB 点云 + 呼吸缩放 ±5% + 上下浮动 + 披风 x-shear 摆动
  globe:     程序化经纬球点云, NASA 贴图大陆, 逐帧自转
  glbseq:    --glb-dir 目录内 sorted *.glb 逐文件一帧 (外部工具烘的帧序列),
             全序列共用 bbox 归一 (防帧间 jitter), 逐文件采样结果 npz 缓存
  glbanim:   --glb 单文件真 glTF 动画 (骨骼 skinning / morph targets / 节点
             TRS), --anim-take 选 take, --frames N 均匀采样 timeline
             (t = k/N × duration, 首尾相接可 loop), 见 glb_anim.py
  spin:      任意静态 GLB 绕竖轴 (y) 自转, --frames = 一圈; --scale 整体
             缩放 (地球仪用 0.48 = 直径半幅), 贴图色要过 1-bit 用
             --lighting none + brighten/gamma/saturation 全 1.0
  palace:    程序化紫禁城空中巡游 (palace.py): 绕 y 轴一整圈 yaw +
             zoom 0.75→1.15→0.75 正弦一循环 (拉远/拉近), 红墙金顶
             白玉台/金水河/角楼, 全 1-bit 纯色, 首尾相接可 loop
  notredame: 程序化巴黎圣母院空中巡游 (notredame.py), 同 palace 巡游
             (共用 orbit_frames): 白石双塔/玫瑰窗(蓝盘红心)/铜绿坡顶/
             飞扶壁/交叉点尖塔(黄尖)/半圆后殿, 哥特竖直感, 全 1-bit 纯色
  (stream --dir = 预渲染 .bin 目录, 即任务里的 'file' 源)
"""
import os
import sys
import math
import time
import glob
import json
import zlib
import socket
import struct
import hashlib
from PIL import Image
import argparse
import threading
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, '..', '..', 'tools'))
sys.path.insert(0, TOOLS)
import pack_obs
import gen_anime_slices as gas

# ---- PVS1 协议常量 (= stream/protocol.h) ----
MAGIC = b'PVS1'
N_SLICES = 360
FRAME_RAW = N_SLICES * pack_obs.SLICE_STRIDE          # 4423680
FLAG_RLE, FLAG_ZLIB = 0x0001, 0x0002
ACK, NAK = 0x06, 0x15
HDR = struct.Struct('<4sIIHH')                        # 16B
PAD = b'\0' * (pack_obs.SLICE_STRIDE - pack_obs.SLICE_DATA)
DEFAULT_PORT = 9500


# ================= 压缩 =================

def rle_encode(data):
    """零游程 RLE: 0x00 → [0x00][run:u16le], 其余字节字面直传."""
    a = np.frombuffer(data, np.uint8)
    out = bytearray()
    pos = 0
    for j in np.flatnonzero(a):
        j = int(j)
        run = j - pos
        while run > 0:
            r = min(run, 65535)
            out += b'\x00' + r.to_bytes(2, 'little')
            run -= r
        out.append(int(a[j]))
        pos = j + 1
    run = len(a) - pos
    while run > 0:
        r = min(run, 65535)
        out += b'\x00' + r.to_bytes(2, 'little')
        run -= r
    return bytes(out)


def rle_decode(data):
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        b = data[i]
        if b == 0:
            out += b'\x00' * int.from_bytes(data[i + 1:i + 3], 'little')
            i += 3
        else:
            out.append(b)
            i += 1
    return bytes(out)


def compress_frame(raw, codec, zlevel):
    if codec == 'zlib':
        return zlib.compress(raw, zlevel), FLAG_ZLIB
    if codec == 'rle':
        return rle_encode(raw), FLAG_RLE
    return raw, 0


def decompress_frame(payload, flags):
    if flags & FLAG_ZLIB:
        return zlib.decompress(payload)
    if flags & FLAG_RLE:
        return rle_decode(payload)
    return payload


# ================= 帧渲染 (点云 → 4.4MB packed frame) =================

def render_packed_frame(vox, frame_idx, render_slices, sub, thresh, dither):
    """体素格 → 完整 360×0x3000 帧. render_slices < 360 时每个渲染角复制填
    360/render_slices 个槽 (布局不变, 省渲染时间); Bayer 相位仍逐槽+逐帧变."""
    assert N_SLICES % render_slices == 0, '--render-slices 必须整除 360'
    dup = N_SLICES // render_slices
    d_step = 2 * math.pi / render_slices
    parts = []
    for k in range(render_slices):
        img = gas.render_slice(vox, k * d_step, sub, d_step)
        for j in range(dup):
            slot = k * dup + j
            phase = slot + frame_idx * 7        # 7 与 16 互素, 逐帧遍历相位
            on = gas.to_1bit(img, thresh, dither, phase)
            parts.append(pack_obs.pack_slice(on))
            parts.append(PAD)
    return b''.join(parts)


# ================= 动画源: spinpulse =================

def _cache_dir():
    return os.environ.get('POVSTREAM_CACHE', os.path.join(HERE, 'cache'))


def load_anime_points(args):
    """GLB 采样点云, 结果缓存 npz (采样是最贵的一步, 只做一次)."""
    samples = args.samples or 1800000
    key = f'{os.path.splitext(os.path.basename(args.glb))[0]}_{samples}'
    cache = os.path.join(_cache_dir(), f'pts_{key}.npz')
    if os.path.exists(cache):
        z = np.load(cache)
        xyz, col = z['xyz'], z['col']
        print(f'[cache] {cache}: {len(xyz)} pts', flush=True)
    else:
        xyz, col = gas.points_from_glb(args.glb, samples, args.lighting, args.ambient)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        np.savez_compressed(cache, xyz=xyz, col=col)
        print(f'[cache] saved {cache}', flush=True)
    col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
    p = gas.normalize_points(xyz, args.z_stretch)
    return p.astype(np.float32), col.astype(np.float32)


def spinpulse_frames(args):
    """呼吸 ±5% + 竖直浮动 + 披风 sinusoidal x-shear. 周期 = --frames, 无缝循环."""
    p0, col = load_anime_points(args)
    p0 = p0 * 0.92                       # 留呼吸/浮动余量, 防边界 clip 糊
    n = args.frames
    for t in range(n):
        u = 2 * math.pi * t / n
        p = p0 * (1.0 + args.breath * math.sin(u))          # 呼吸
        p[:, 1] += args.bob * math.sin(u + math.pi / 2)     # 上下浮动
        # 披风摆动: 按离质心竖直距离的 x 方向剪切 (上下反相)
        p[:, 0] += args.sway * math.sin(2 * u + 0.7) * (p0[:, 1] / gas.H_BUDGET)
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: globe =================

# 手绘 80x40 世界陆海掩膜 (等距圆柱, 行=北→南 90..-90, 列=经度 -180..+180; #=陆)
# 办公网拦外链, 内嵌数据; 精度 4.5°/格, 显示 160px 下可辨认各大洲
WORLD_MAP = [
    "................................................................................",
    "............................#####..............................................",
    "............##....##........#######........#...................................",
    "..........########..####....########.....................######..######........",
    "...###...#################..#######........####....##############.#####........",
    "..#####..################....#####...##...######..#############.########.......",
    "..######..###############.....###.....#..#####...##############..#######.......",
    "...####...##############.............##..####...###############.########.......",
    "....##.....##############...........##..#########################.#####........",
    "............##############..........############################..###..........",
    "............##############.........#############################...##..........",
    "............#############........############..#################...##..........",
    ".............###########.........####..####..#####..############...#...........",
    ".............####..#####.........#################...####..####................",
    "..............######..............################...####...###.##.............",
    "...............#####.............################....###....###................",
    ".................####.............###############....##.....###.#..............",
    "..................####............##############.....#......##..##.............",
    "....................####...........############......#.......#..#..............",
    ".....................######........###########.......#..##..##...####..........",
    ".....................#########......##########..........####.#..######.........",
    ".....................##########.....##########...........##......#####.........",
    "......................##########....#########..#.....##########................",
    "......................##########....########...#....############...............",
    ".......................#########....#######....#...##############..............",
    ".......................########.....######.........#############...............",
    ".......................#######......#####...........############...............",
    ".......................######........###.............#####..###................",
    ".......................#####..................................#..##............",
    ".......................####..................................#..##.............",
    ".......................###......................................#..............",
    ".......................##.......................................................",
    ".......................##.......................................................",
    "................................................................................",
    "................................................................................",
    "................................................................................",
    "........########################################################################",
    "################################################################################",
    "################################################################################",
    "################################################################################",
]
_WM = np.array([[c == '#' for c in row.ljust(80, '.')[:80]] for row in WORLD_MAP], np.bool_)


def globe_frames(args):
    """空心壳地球 (用户定): 壳面贴 NASA earth_clean.jpg 真实大陆,
    海纯蓝/陆纯绿/冰白 (zynq_pov 实测分类), 三层壳保密度, 逐帧转纹理."""
    R = gas.R_BUDGET * 0.48                                 # 直径半幅 (2026-07-09 用户定)
    tex_path = os.path.join(HERE, 'earth_clean.jpg')
    tex = np.asarray(Image.open(tex_path).convert('RGB'), np.int32)
    TH_, TW_ = tex.shape[:2]
    n_lat, n_lon = 240, 720
    lat = np.linspace(-math.pi / 2 * 0.98, math.pi / 2 * 0.98, n_lat, dtype=np.float32)
    lon = np.linspace(0, 2 * math.pi, n_lon, endpoint=False, dtype=np.float32)
    LA, LO = np.meshgrid(lat, lon, indexing='ij')
    la1, lo1 = LA.ravel(), LO.ravel()
    la = np.concatenate([la1] * 3)
    lo = np.concatenate([lo1] * 3)
    rr = np.concatenate([np.full_like(la1, R), np.full_like(la1, R - 1.3),
                         np.full_like(la1, R - 2.6)])       # 三层壳
    p = np.stack([rr * np.cos(la) * np.cos(lo),
                  rr * np.sin(la),
                  rr * np.cos(la) * np.sin(lo)], axis=1).astype(np.float32)
    n = args.frames
    for t in range(n):
        le = (lo + 2 * math.pi * t / n) % (2 * math.pi)     # 自转: 转纹理不转点
        row = np.clip(((math.pi / 2 - la) / math.pi * TH_).astype(np.int32), 0, TH_ - 1)
        ci = np.clip((le / (2 * math.pi) * TW_).astype(np.int32), 0, TW_ - 1)
        r_, g_, b_ = tex[row, ci, 0], tex[row, ci, 1], tex[row, ci, 2]
        ocean = (b_ > g_ + 10) & (b_ > r_ + 10)
        ice = (r_ > 170) & (g_ > 170) & (b_ > 170)
        col = np.zeros((len(p), 3), np.float32)
        col[:] = (0, 255, 0)                                # 默认陆 = 纯绿
        col[ocean] = (0, 0, 255)                            # 海 = 纯蓝
        col[ice] = (255, 255, 255)                          # 冰盖 = 白
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: glbseq / glbanim (GLB 动画装载器) =================
# TODO: pov_studio.py PRESETS 尚未接入 glbseq/glbanim (本轮不做 GUI 集成)

def normalize_common(xyz, cmin, cmax, z_stretch):
    """gas.normalize_points 的固定-bbox 版: 整段动画所有帧共用同一
    center/scale (逐帧各自归一会让动画整体缩放/平移抖动)."""
    p = xyz - (cmin + cmax) / 2.0
    p[:, 2] *= z_stretch
    h = np.maximum((cmax - cmin) / 2.0, 1e-6)
    s = min(gas.R_BUDGET / h[0], gas.H_BUDGET / h[1],
            gas.R_BUDGET / max(h[2] * z_stretch, 1e-6))
    return (p * s).astype(np.float32)


def _load_glb_points_cached(path, samples, lighting, ambient):
    """单 GLB 文件采样点云 + npz 缓存 (同 load_anime_points, key 加路径
    hash 防序列目录里同名文件互撞)."""
    tag = hashlib.md5(os.path.abspath(path).encode()).hexdigest()[:8]
    key = f'{os.path.splitext(os.path.basename(path))[0]}_{tag}_{samples}'
    cache = os.path.join(_cache_dir(), f'pts_{key}.npz')
    if os.path.exists(cache):
        z = np.load(cache)
        xyz, col = z['xyz'], z['col']
        print(f'[cache] {cache}: {len(xyz)} pts', flush=True)
    else:
        xyz, col = gas.points_from_glb(path, samples, lighting, ambient)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        np.savez_compressed(cache, xyz=xyz, col=col)
        print(f'[cache] saved {cache}', flush=True)
    return np.asarray(xyz, np.float32), np.asarray(col, np.float32)


def glbseq_frames(args):
    """GLB 帧序列: --glb-dir 内 sorted *.glb, 每文件 = 一帧 (外部 DCC 工具
    逐帧导出的动画). 帧数 = 文件数 (--frames 被忽略). 先全量装载算全局
    bbox, 所有帧共用同一归一变换."""
    if not args.glb_dir:
        sys.exit('--anim glbseq 需要 --glb-dir <目录>')
    files = sorted(glob.glob(os.path.join(args.glb_dir, '*.glb')))
    if not files:
        sys.exit(f'no .glb files in {args.glb_dir}')
    samples = args.samples or 400000
    seq = [_load_glb_points_cached(f, samples, args.lighting, args.ambient)
           for f in files]
    cmin = np.min([x.min(axis=0) for x, _ in seq], axis=0)
    cmax = np.max([x.max(axis=0) for x, _ in seq], axis=0)
    args.frames = len(files)
    print(f'[glbseq] {len(files)} frames x {samples} pts, '
          f'bbox {cmin.round(2)}..{cmax.round(2)}', flush=True)
    for xyz, col in seq:
        col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
        yield gas.voxel_grid(normalize_common(xyz, cmin, cmax, args.z_stretch),
                             col, verbose=False, ssaa=args.ssaa)


def pure_rgb_snap(col, dom=0.55):
    """1-bit 海报化: 每点主导通道组合置 1.0 其余置 0 (通道≥dom×max 算主导),
    抖动存活率 100%, 密度=几何覆盖上限 (地球仪纯通道色经验的通用化).
    代价: 色彩变 7 色海报风 (R/G/B/黄/品红/青/白)."""
    m = col.max(axis=1, keepdims=True)
    return (col >= np.maximum(dom * m, 1e-6)).astype(np.float32) * 255.0


def glbanim_frames(args):
    """单 GLB 真 glTF 动画 (骨骼 skinning / morph targets / 节点 TRS,
    见 glb_anim.py). --frames N 均匀采 timeline: t = k/N × duration
    (k=0..N-1, 首尾相接可 loop); --anim-take 选 take (名或索引).
    所有帧共用全时段 bbox 归一."""
    import glb_anim
    samples = args.samples or 400000
    smp = glb_anim.AnimSampler(args.glb, take=args.anim_take, samples=samples,
                               lighting=args.lighting, ambient=args.ambient)
    n = args.frames
    if smp.duration <= 0:
        print('[glbanim] WARNING: 无动画 timeline, 全帧静态姿态', flush=True)
    times = [smp.duration * k / n for k in range(n)]
    if args.robust_fit:
        # 鲁棒 bbox: 每帧 [2,98] 分位取并集, 排除飞行特效骨骼/技能位移
        # 拖爆包围盒把人物压小 (LoL R 技能锤/箭/针线能飞 10+ 身位)
        los, his = [], []
        for t in times:
            xyz, _ = smp.points_at(t)
            los.append(np.percentile(xyz, 2, axis=0))
            his.append(np.percentile(xyz, 98, axis=0))
        cmin = np.min(los, axis=0).astype(np.float32)
        cmax = np.max(his, axis=0).astype(np.float32)
        print(f'[glbanim] robust bbox {cmin.round(2)}..{cmax.round(2)}', flush=True)
    else:
        cmin, cmax = smp.bbox_over(times)
    for t in times:
        xyz, col = smp.points_at(t)
        col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
        if args.pure_rgb:
            col = pure_rgb_snap(col, args.pure_dom)
        p = normalize_common(xyz, cmin, cmax, args.z_stretch) * args.scale
        p[:, 0] += args.x_offset
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


def spin_frames(args):
    """任意静态 GLB 绕竖轴 (y) 自转: 采样一次, 逐帧旋转点云 (--frames =
    一圈, 首尾相接可 loop). --scale 归一化后整体缩放 (<1 缩小留白);
    --shells N 洋葱状向内复制 N 层 (间距 --shell-gap voxel, 表面网格壳
    太薄时加厚, 同 globe 源三层壳套路)."""
    xyz, col = _load_glb_points_cached(args.glb, args.samples or 400000,
                                       args.lighting, args.ambient)
    col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
    p0 = gas.normalize_points(xyz, args.z_stretch) * args.scale
    if args.shells > 1:
        r_eff = float(np.linalg.norm(p0, axis=1).max())
        p0 = np.concatenate([p0 * (1.0 - k * args.shell_gap / r_eff)
                             for k in range(args.shells)])
        col = np.concatenate([col] * args.shells)
    n = args.frames
    for t in range(n):
        a = 2 * math.pi * t / n
        c, s = math.cos(a), math.sin(a)
        p = np.empty_like(p0)
        p[:, 0] = p0[:, 0] * c - p0[:, 2] * s
        p[:, 1] = p0[:, 1]
        p[:, 2] = p0[:, 0] * s + p0[:, 2] * c
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: palace / notredame (空中巡游 orbit) =================

ORBIT_ZOOM_MAX = 1.15
PALACE_ZOOM_MAX = ORBIT_ZOOM_MAX          # 兼容旧名 (test_palace.py 引用)


def orbit_prescale(p0, col):
    """建筑点云预缩放到 zoom 最大 (1.15) 时任意 yaw 角都不出预算:
    旋转约束是径向 sqrt(x²+z²) ≤ R_BUDGET (footprint 对角在 45° 时甩到
    最远, 出圆柱会被 clip 糊边), 竖直 |y| ≤ H_BUDGET."""
    r_max = float(np.hypot(p0[:, 0], p0[:, 2]).max())
    y_max = float(np.abs(p0[:, 1]).max())
    s = min((gas.R_BUDGET - 0.5) / (r_max * ORBIT_ZOOM_MAX),
            (gas.H_BUDGET - 0.5) / (y_max * ORBIT_ZOOM_MAX))
    return (p0 * s).astype(np.float32), col.astype(np.float32)


def palace_points_prescaled():
    """紫禁城点云 (palace.py), 已 orbit 预缩放."""
    import palace
    return orbit_prescale(*palace.build_palace())


def notredame_points_prescaled():
    """巴黎圣母院点云 (notredame.py), 已 orbit 预缩放."""
    import notredame
    return orbit_prescale(*notredame.build_notredame())


def orbit_frames(points, colors, args):
    """空中巡游 (palace/notredame 共用): 每循环绕 y 轴 yaw 一整圈, 同时
    zoom 按正弦 0.75→1.15→0.75 一循环 (拉远→拉近→拉远, 读作 fly-out/
    fly-in). points 须已 orbit_prescale. 先旋转再缩放再体素化,
    --frames = 循环周期, 首尾相接可 loop."""
    n = args.frames
    for t in range(n):
        u = 2 * math.pi * t / n
        zoom = 0.95 - 0.20 * math.cos(u)         # 0.75 → 1.15 → 0.75
        c, s = math.cos(u), math.sin(u)          # yaw 一整圈
        p = np.empty_like(points)
        p[:, 0] = (points[:, 0] * c - points[:, 2] * s) * zoom
        p[:, 1] = points[:, 1] * zoom
        p[:, 2] = (points[:, 0] * s + points[:, 2] * c) * zoom
        yield gas.voxel_grid(p, colors, verbose=False, ssaa=args.ssaa)


def palace_frames(args):
    """紫禁城空中巡游 (几何 palace.py, 巡游 orbit_frames)."""
    p0, col = palace_points_prescaled()
    yield from orbit_frames(p0, col, args)


def notredame_frames(args):
    """巴黎圣母院空中巡游 (几何 notredame.py, 巡游 orbit_frames)."""
    p0, col = notredame_points_prescaled()
    yield from orbit_frames(p0, col, args)


ANIMS = {'spinpulse': spinpulse_frames, 'globe': globe_frames,
         'glbseq': glbseq_frames, 'glbanim': glbanim_frames,
         'spin': spin_frames, 'palace': palace_frames,
         'notredame': notredame_frames}


def gen_packed_frames(args):
    """动画名 → 逐帧 4,423,680B packed bytes."""
    for i, vox in enumerate(ANIMS[args.anim](args)):
        t0 = time.time()
        raw = render_packed_frame(vox, i, args.render_slices, args.sub,
                                  args.thresh, not args.no_dither)
        print(f'[render] frame {i}/{args.frames} {time.time() - t0:.1f}s', flush=True)
        yield raw


# ================= render 子命令 (预渲染到磁盘) =================

def cmd_render(args):
    out_dir = args.out_dir or os.path.join(HERE, f'frames_{args.anim}')
    os.makedirs(out_dir, exist_ok=True)
    t0 = time.time()
    for i, raw in enumerate(gen_packed_frames(args)):
        assert len(raw) == FRAME_RAW
        path = os.path.join(out_dir, f'frame_{i:04d}.bin')
        with open(path, 'wb') as f:
            f.write(raw)
    meta = {'anim': args.anim, 'frames': args.frames, 'render_slices': args.render_slices,
            'frame_raw': FRAME_RAW, 'generated': time.strftime('%Y-%m-%d %H:%M:%S')}
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f'[render] {args.frames} frames -> {out_dir} ({time.time() - t0:.1f}s total)', flush=True)


# ================= 推流核心 (类 + 回调, GUI/CLI 共用) =================

class StreamerError(Exception):
    """协议级致命错 (NAK / 无重连时连接失败)。"""


class Streamer:
    """PVS1 推流器. run(make_iter) 阻塞直到帧尽/stop/致命错.

    make_iter: 无参可调用, 返回逐帧 FRAME_RAW 字节迭代器 (loop 时反复调用).
    reconnect=True 时 ACK 超时/连接断 (板重启) → 每 retry_interval 秒重连,
    重连成功后重发当前帧继续; NAK 永远致命 (协议规定 sender abort).
    回调 (可选, 在推流线程里调):
      on_frame(stats)               每帧 ACK 后
      on_status(event, detail)      event ∈ connected/lost/retry/done
    stop: threading.Event, 置位后尽快退出 (sleep/重连等待均可打断).
    """

    def __init__(self, host, port=DEFAULT_PORT, fps=10.0, loop=False,
                 codec='zlib', zlevel=6, reconnect=False, retry_interval=5.0,
                 ack_timeout=30.0, on_frame=None, on_status=None, stop=None):
        self.host, self.port = host, port
        self.fps, self.loop = fps, loop
        self.codec, self.zlevel = codec, zlevel
        self.reconnect, self.retry_interval = reconnect, retry_interval
        self.ack_timeout = ack_timeout
        self.on_frame = on_frame or (lambda st: None)
        self.on_status = on_status or (lambda ev, detail: None)
        self.stop = stop if stop is not None else threading.Event()
        self.frames = 0                 # 已 ACK 帧数
        self.sent_raw = 0
        self.sent_wire = 0
        self.last_wire = 0              # 最近一帧线上字节 (含 16B 头)
        self.reconnects = 0
        self.t_start = None

    # -- 内部: 连接 (reconnect 时无限重试, stop 可打断; 返回 None = 被停) --
    def _connect(self, first):
        while not self.stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=10)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(self.ack_timeout)
                self.on_status('connected', f'{self.host}:{self.port}')
                return sock
            except OSError as e:
                if not self.reconnect:
                    raise StreamerError(f'connect {self.host}:{self.port} failed: {e}')
                if not first:
                    self.reconnects += 1
                self.on_status('retry', f'{e}; retry in {self.retry_interval}s')
                self.stop.wait(self.retry_interval)
        return None

    def _send_one(self, sock, hdr, payload):
        """发一帧等 ACK. 返回存活 sock (可能重连过) 或 None (被停)."""
        while not self.stop.is_set():
            if sock is None:
                sock = self._connect(first=False)
                if sock is None:
                    return None
            try:
                sock.sendall(hdr + payload)
                ack = sock.recv(1)
                if ack == bytes([ACK]):
                    return sock
                if ack == bytes([NAK]):
                    sock.close()
                    raise StreamerError(f'frame {self.frames}: NAK, abort')
                raise OSError(f'bad/empty ack {ack!r} (peer closed?)')
            except (socket.timeout, OSError) as e:
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                if not self.reconnect:
                    raise StreamerError(f'frame {self.frames}: {e}')
                self.reconnects += 1
                self.on_status('lost', f'{e}; reconnect in {self.retry_interval}s')
                self.stop.wait(self.retry_interval)
        return None

    def run(self, make_iter):
        self.t_start = time.time()
        t_next = self.t_start
        sock = self._connect(first=True)
        try:
            while sock is not None and not self.stop.is_set():
                for raw in make_iter():
                    if self.stop.is_set():
                        break
                    payload, flags = compress_frame(raw, self.codec, self.zlevel)
                    hdr = HDR.pack(MAGIC, len(payload), len(raw), N_SLICES, flags)
                    sock = self._send_one(sock, hdr, payload)
                    if sock is None:
                        break
                    self.frames += 1
                    self.sent_raw += len(raw)
                    self.last_wire = len(hdr) + len(payload)
                    self.sent_wire += self.last_wire
                    self.on_frame(self)
                    if self.fps > 0:
                        t_next = max(t_next + 1.0 / self.fps, time.time() - 1.0 / self.fps)
                        dt = t_next - time.time()
                        if dt > 0:
                            self.stop.wait(dt)
                if not self.loop:
                    break
        finally:
            if sock is not None:
                sock.close()
            self.on_status('done', f'{self.frames} frames')
        return self

    # -- 统计便利 --
    def elapsed(self):
        return max(time.time() - (self.t_start or time.time()), 1e-6)

    def wire_mbps(self):
        return self.sent_wire / self.elapsed() / 1e6

    def ratio(self):
        return self.sent_raw / max(self.sent_wire, 1)


# ================= stream 子命令 =================

def frame_iter_from_dir(d):
    files = sorted(glob.glob(os.path.join(d, 'frame_*.bin')) or glob.glob(os.path.join(d, '*.bin')))
    if not files:
        sys.exit(f'no .bin frames in {d}')
    for p in files:
        raw = open(p, 'rb').read()
        if len(raw) != FRAME_RAW:
            sys.exit(f'{p}: {len(raw)}B != FRAME_RAW {FRAME_RAW}')
        yield raw


def cmd_stream(args):
    def make_iter():
        return frame_iter_from_dir(args.dir) if args.dir else gen_packed_frames(args)

    def on_status(ev, detail):
        if ev == 'connected':
            print(f'[net] connected {detail} codec={args.codec} fps={args.fps}', flush=True)
        elif ev in ('lost', 'retry'):
            print(f'[net] {ev}: {detail}', flush=True)

    def on_frame(st):
        if args.verbose:
            print(f'  frame {st.frames}: {st.last_wire - HDR.size}B wire '
                  f'({FRAME_RAW / max(st.last_wire - HDR.size, 1):.1f}x)', flush=True)

    s = Streamer(args.host, args.port, fps=args.fps, loop=args.loop,
                 codec=args.codec, zlevel=args.zlevel, reconnect=args.reconnect,
                 on_frame=on_frame, on_status=on_status)
    try:
        s.run(make_iter)
    except KeyboardInterrupt:
        print('[net] interrupted', flush=True)
    except StreamerError as e:
        print(f'[net] {e}', flush=True)
        if s.frames == 0:
            sys.exit(1)
    dt = s.elapsed()
    print(f'[stats] {s.frames} frames in {dt:.2f}s = {s.frames / dt:.2f} model fps | '
          f'wire {s.wire_mbps():.2f} MB/s (raw {s.sent_raw / dt / 1e6:.2f} MB/s) | '
          f'compression {s.ratio():.1f}x', flush=True)
    print(f'[stats] projected fps @ 9.4 MB/s link: '
          f'{9.4e6 / (s.sent_wire / max(s.frames, 1)):.1f}', flush=True)


# ================= bench 子命令 =================

def cmd_bench(args):
    raw = open(args.file, 'rb').read()
    zero = (np.frombuffer(raw, np.uint8) == 0).mean()
    print(f'{args.file}: {len(raw)}B, {zero * 100:.1f}% zeros')
    rows = []
    for name, fn, dec in [('zlib-1', lambda d: zlib.compress(d, 1), zlib.decompress),
                          ('zlib-6', lambda d: zlib.compress(d, 6), zlib.decompress),
                          ('rle', rle_encode, rle_decode)]:
        t0 = time.time(); c = fn(raw); te = time.time() - t0
        t0 = time.time(); d = dec(c); td = time.time() - t0
        assert d == raw, name
        rows.append((name, len(c), len(raw) / len(c), te, td))
    print(f'{"codec":8} {"bytes":>9} {"ratio":>6} {"enc_ms":>7} {"dec_ms":>7} {"fps@9.4MB/s":>12}')
    for name, sz, ratio, te, td in rows:
        print(f'{name:8} {sz:9} {ratio:5.1f}x {te * 1e3:7.0f} {td * 1e3:7.0f} {9.4e6 / sz:12.1f}')


# ================= CLI =================

def add_render_opts(ap):
    ap.add_argument('--anim', choices=sorted(ANIMS), default='spinpulse')
    ap.add_argument('--frames', type=int, default=36, help='动画帧数 (=循环周期)')
    ap.add_argument('--render-slices', type=int, default=360,
                    help='实际渲染角度数 (整除 360, 减少省时, 布局仍 360 槽)')
    ap.add_argument('--sub', type=int, default=3)
    ap.add_argument('--thresh', type=float, default=128)
    ap.add_argument('--no-dither', action='store_true')
    # spinpulse / glbanim
    ap.add_argument('--glb', default=gas.DEFAULT_GLB)
    ap.add_argument('--samples', type=int, default=None,
                    help='GLB 采样点数 (默认: spinpulse 1800000, glbseq/glbanim 400000)')
    ap.add_argument('--z-stretch', type=float, default=1.0)
    ap.add_argument('--ssaa', type=int, default=1,
                    help='N× 空间超采样抗锯齿: 细分格覆盖率→边缘调暗→Bayer 变点密度 (3 推荐, 配 --samples 1.2M+)')
    ap.add_argument('--scale', type=float, default=1.0,
                    help='spin: 归一化后整体缩放 (地球仪 0.48=直径半幅)')
    ap.add_argument('--shells', type=int, default=1,
                    help='spin: 点云洋葱状向内复制层数 (壳太薄时加厚)')
    ap.add_argument('--shell-gap', type=float, default=1.3,
                    help='spin: 壳层间距 (voxel)')
    ap.add_argument('--robust-fit', action='store_true',
                    help='glbanim: [2,98] 分位 bbox 归一, 抗技能位移/特效骨骼压小人物')
    ap.add_argument('--x-offset', type=float, default=0.0,
                    help='glbanim: 归一化后 x 平移 (voxel), 正值离转轴 (屏中央显示差规避)')
    ap.add_argument('--pure-rgb', action='store_true',
                    help='glbanim: 1-bit 海报化, 每点snap到主导纯通道组合 (密度上限, 7色)')
    ap.add_argument('--pure-dom', type=float, default=0.55,
                    help='pure-rgb 主导阈值 (通道≥dom×max 点亮, 越低越偏白)')
    ap.add_argument('--brighten', type=float, default=1.5)
    ap.add_argument('--gamma', type=float, default=0.9)
    ap.add_argument('--saturation', type=float, default=2.0)
    ap.add_argument('--lighting', default='lambert')
    ap.add_argument('--ambient', type=float, default=0.7)
    ap.add_argument('--breath', type=float, default=0.05, help='呼吸缩放幅度')
    ap.add_argument('--bob', type=float, default=2.5, help='竖直浮动幅度 (voxel)')
    ap.add_argument('--sway', type=float, default=3.0, help='披风 x-shear 幅度 (voxel)')
    # glbseq / glbanim
    ap.add_argument('--glb-dir', default=None,
                    help='glbseq: GLB 帧序列目录 (sorted *.glb, 每文件一帧)')
    ap.add_argument('--anim-take', default='0',
                    help='glbanim: 动画 take 名或索引 (默认 0)')


def main():
    ap = argparse.ArgumentParser(description='POV volumetric display PC streamer (PVS1)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('render', help='预渲染动画帧到磁盘')
    add_render_opts(r)
    r.add_argument('--out-dir', default=None, help='默认 stream/pc/frames_<anim>/')
    r.set_defaults(fn=cmd_render)

    s = sub.add_parser('stream', help='推流 (--dir 预渲染目录, 或 --anim 现渲)')
    add_render_opts(s)
    s.add_argument('--dir', default=None, help='预渲染 .bin 帧目录 (file 源)')
    s.add_argument('--host', default='127.0.0.1')
    s.add_argument('--port', type=int, default=DEFAULT_PORT)
    s.add_argument('--fps', type=float, default=12.0,
                   help='发送节奏上限; 设高于转速让逐帧 ACK 自动贴住翻页率 (设 4 反而被 RTT 拖到 3.5)')
    s.add_argument('--loop', action='store_true')
    s.add_argument('--reconnect', action='store_true',
                   help='连接断/ACK 超时不退出, 每 5s 重连 (板重启自动续推)')
    s.add_argument('--codec', choices=['zlib', 'rle', 'raw'], default='zlib')
    s.add_argument('--zlevel', type=int, default=6)
    s.add_argument('-v', '--verbose', action='store_true')
    s.set_defaults(fn=cmd_stream)

    b = sub.add_parser('bench', help='压缩方案测量')
    b.add_argument('--file', default=os.path.join(TOOLS, 'anime_slices.bin'))
    b.set_defaults(fn=cmd_bench)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
