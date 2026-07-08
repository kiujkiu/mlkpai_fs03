#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_anime_slices.py — anime GLB/点云 → 360 子午面切片 → FS03 DDR 镜像 (2026-07-08).

几何约定 (照搬 zynq_pov/tools/_gen_anime_slices.py 的 multivox 式带符号半径):
  转轴 = 模型竖直中轴 = 屏 X 中心 (X=79.5, 轴在屏中间不是边缘)。
  X = 带符号半径 d = X-79.5 ∈ [-79.5,+79.5]; slice i (θ_i=i°) 的 X>79.5 半边
  显示方位角 θ_i 半平面, X<79.5 半边显示 θ_i+180°; Y 0..179 = 高 (上→下)。
  厚度 = 逐像素最近邻体素采样 (~1 voxel), 可选 --sub N 子角度 max 混合保证相邻
  片覆盖连续。全像素统一处理, 无按径向距离变化的补偿 (Voxon P3 专利红线)。

颜色: GLB 纹理采样 + lambert 光照 → brighten/gamma/saturation (沿用 anime_to_bin
  已验证参数) → 1-bit/通道 (Bayer 4x4 有序抖动, 可关)。

打包: pack_obs.py (= gen_chess_obs 实测映射)。slice i @ 偏移 i*0x3000,
  11664B 数据 + padding 0, 默认 360 slices = 4,423,680B。

用法 (Windows python, 需 pygltflib):
  python gen_anime_slices.py [--glb PATH] [--slices 360] [--thresh 128]
      [--no-dither] [--sub 3] [--samples 1800000] [--z-stretch 1.5]
      [--points PATH(PovPoint bin 备胎)] [--out anime_slices.bin]
"""
import os
import sys
import math
import argparse
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pack_obs
from pack_obs import W, H, SLICE_STRIDE, SLICE_DATA

ZYNQ_POV_HOST = os.path.abspath(os.path.join(HERE, '..', '..', 'zynq_pov', 'host'))
DEFAULT_GLB = os.path.join(ZYNQ_POV_HOST, 'anime_62459.glb')

GR = W      # 体素格 x/z 尺寸 (半径预算 79)
GH = H      # 体素格 y 尺寸 (高度预算 178)
R_BUDGET = W // 2 - 1.0    # 79
H_BUDGET = H // 2 - 1.0    # 89

BAYER4 = np.array([[0, 8, 2, 10], [12, 4, 14, 6],
                   [3, 11, 1, 9], [15, 7, 13, 5]], np.float32)

DIGITS = {  # 3x5 位图字体 (借 gen_chess_obs)
    '0': ['111', '101', '101', '101', '111'], '1': ['010', '110', '010', '010', '111'],
    '2': ['111', '001', '111', '100', '111'], '3': ['111', '001', '111', '001', '111'],
    '4': ['101', '101', '111', '001', '001'], '5': ['111', '100', '111', '001', '111'],
    '6': ['111', '100', '111', '101', '111'], '7': ['111', '001', '010', '100', '100'],
    '8': ['111', '101', '111', '101', '111'], '9': ['111', '101', '111', '001', '111'],
}


def color_adjust(col, brighten=1.5, gamma=0.9, saturation=2.0):
    """向量化版 glb_to_points.normalize_and_quantize 颜色链 (顺序一致:
    gamma → hue-preserving brighten → HSV saturation boost)。col float (N,3) 0..255。"""
    c = col.astype(np.float32)
    if gamma != 1.0:
        c = 255.0 * np.power(np.clip(c / 255.0, 0, 1), gamma)
    if brighten != 1.0:
        mx = c.max(axis=1, keepdims=True)
        tgt = np.minimum(255.0, mx * brighten)
        ratio = np.where(mx > 0, tgt / np.maximum(mx, 1e-6), 1.0)
        c = np.minimum(255.0, c * ratio)
    if saturation != 1.0:
        v = c.max(axis=1)
        mn = c.min(axis=1)
        s = np.where(v > 0, (v - mn) / np.maximum(v, 1e-6), 0.0)
        s2 = np.minimum(1.0, s * saturation)
        # 同 hue 下压 min 通道: c' = v - (v-c) * s2/s
        k = np.where(s > 1e-6, s2 / np.maximum(s, 1e-6), 1.0)[:, None]
        c = v[:, None] - (v[:, None] - c) * k
    return np.clip(c, 0, 255)


def points_from_glb(path, n_samples, lighting, ambient):
    if ZYNQ_POV_HOST not in sys.path:
        sys.path.insert(0, ZYNQ_POV_HOST)
    from glb_to_points import load_glb, sample_triangles
    print(f'[glb] load {path}', flush=True)
    tris = load_glb(path, verbose=True)
    print(f'[glb] sampling {n_samples} pts (lighting={lighting}, ambient={ambient})...', flush=True)
    pts = sample_triangles(tris, n_samples, lighting=lighting, ambient=ambient)
    a = np.array(pts, np.float32)
    return a[:, :3], a[:, 3:6]


def points_from_bin(path):
    """PovPoint 16B struct 备胎 (密度按 ±40 尺度做的, 放大到 160x180 可能有洞)。"""
    raw = open(path, 'rb').read()
    n = len(raw) // 16
    pts = np.frombuffer(raw, dtype=np.dtype([('x', '<i2'), ('y', '<i2'), ('z', '<i2'),
                                             ('p0', '<i2'), ('r', 'u1'), ('g', 'u1'),
                                             ('b', 'u1'), ('p1', 'u1'), ('p2', '<i4')]), count=n)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float32)
    col = np.stack([pts['r'], pts['g'], pts['b']], axis=1).astype(np.float32)
    return xyz, col


def normalize_points(xyz, z_stretch, verbose=True):
    """居中 + 等比缩放到 (±R_BUDGET, ±H_BUDGET, ±R_BUDGET), 返回归一化点。
    动画调用方可用同一 scale 逐帧变换 (呼吸/摆动) 后再喂 voxel_grid。"""
    p = xyz.copy()
    cmin, cmax = p.min(axis=0), p.max(axis=0)
    p -= (cmin + cmax) / 2
    p[:, 2] *= z_stretch
    hx = np.abs(p[:, 0]).max()
    hy = np.abs(p[:, 1]).max()
    hz = np.abs(p[:, 2]).max()
    s = min(R_BUDGET / max(hx, 1e-6), H_BUDGET / max(hy, 1e-6), R_BUDGET / max(hz, 1e-6))
    p *= s
    if verbose:
        print(f'[vox] scale={s:.3f} extents x±{hx*s:.1f} y±{hy*s:.1f} z±{hz*s:.1f}', flush=True)
    return p


def voxel_grid(p, col, verbose=True):
    """归一化点 (voxel 坐标, 原点在中心) → 体素格颜色平均 (GR,GH,GR,3)。"""
    gx = np.clip(np.rint(p[:, 0]).astype(np.int32) + GR // 2, 0, GR - 1)
    gy = np.clip(np.rint(p[:, 1]).astype(np.int32) + GH // 2, 0, GH - 1)
    gz = np.clip(np.rint(p[:, 2]).astype(np.int32) + GR // 2, 0, GR - 1)
    acc = np.zeros((GR, GH, GR, 3), np.float32)
    cnt = np.zeros((GR, GH, GR), np.float32)
    np.add.at(acc, (gx, gy, gz), col)
    np.add.at(cnt, (gx, gy, gz), 1.0)
    occ = cnt > 0
    acc[occ] /= cnt[occ][:, None]
    if verbose:
        print(f'[vox] {int(occ.sum())} occupied cells', flush=True)
    return acc


def voxelize(xyz, col, z_stretch):
    """居中 + 等比缩放 + 体素化 (静态一步到位, 保持旧 API)。"""
    return voxel_grid(normalize_points(xyz, z_stretch), col)


def render_slice(vox, theta, sub, d_step):
    """角度 theta 的观察者视角 float 图 (H,W,3)。sub 个子角度 max 混合。"""
    D = np.arange(W, dtype=np.float32) - (W - 1) / 2.0
    gy = (H - 1) - np.arange(H)          # 屏 Y 上→下, 体素 y 下→上
    img = np.zeros((H, W, 3), np.float32)
    offs = [0.0] if sub <= 1 else [(k / sub - (sub - 1) / (2.0 * sub)) * d_step for k in range(sub)]
    for off in offs:
        c, s = math.cos(theta + off), math.sin(theta + off)
        wx = np.clip(np.rint(D * c).astype(np.int32) + GR // 2, 0, GR - 1)
        wz = np.clip(np.rint(D * s).astype(np.int32) + GR // 2, 0, GR - 1)
        np.maximum(img, vox[wx[None, :], gy[:, None], wz[None, :]], out=img)
    return img


def to_1bit(img, thresh, dither, phase):
    """float 图 → bool 图。dither = Bayer 4x4 有序抖动 (阈值均值 = thresh),
    phase 随 slice 移位 → 旋转时空间抖动变时间抖动, 观感更顺。"""
    if not dither:
        return img >= thresh
    t = (BAYER4 + 0.5) / 16.0 * (2.0 * thresh)
    t = np.roll(np.roll(t, phase % 4, axis=0), (phase // 4) % 4, axis=1)
    tm = np.tile(t, (H // 4 + 1, W // 4 + 1))[:H, :W]
    return img > np.clip(tm, 1.0, 255.0)[:, :, None]


def draw_num(px, x0, y0, text, fg):
    for ch in text:
        for ry, row in enumerate(DIGITS[ch]):
            for rx, c in enumerate(row):
                if c == '1':
                    px[x0 + rx, y0 + ry] = fg
        x0 += 4


def make_preview(slice_bufs, n_slices, path, scale=4, count=12, cols=4):
    """从打包字节 unpack 回观察者视角 (验证的是 bin 本身), 4x 拼图。"""
    rows = (count + cols - 1) // cols
    pad = 2
    tile_w, tile_h = W * scale + pad, H * scale + pad
    canvas = Image.new('RGB', (cols * tile_w + pad, rows * tile_h + pad), (24, 24, 24))
    for k in range(count):
        i = k * n_slices // count
        img = pack_obs.unpack_slice(slice_bufs[i]).astype(np.uint8) * 255
        tile = Image.fromarray(img).resize((W * scale, H * scale), Image.NEAREST)
        px = tile.load()
        deg = i * 360 // n_slices
        draw_num(px, 3, 3, str(deg), (255, 255, 0))
        canvas.paste(tile, (pad + (k % cols) * tile_w, pad + (k // cols) * tile_h))
    canvas.save(path)
    print(f'[preview] {path} ({count} slices, {scale}x)', flush=True)


def main():
    ap = argparse.ArgumentParser(description='anime GLB → 360 POV slices → FS03 DDR image')
    ap.add_argument('--glb', default=DEFAULT_GLB)
    ap.add_argument('--points', default=None, help='PovPoint .bin 备胎 (跳过 GLB)')
    ap.add_argument('--slices', type=int, default=360)
    ap.add_argument('--thresh', type=float, default=128, help='1-bit 亮度阈值 (0..255)')
    ap.add_argument('--no-dither', action='store_true', help='关 Bayer 有序抖动')
    ap.add_argument('--sub', type=int, default=3, help='子角度采样数 (厚度连续性)')
    ap.add_argument('--samples', type=int, default=1800000)
    ap.add_argument('--z-stretch', type=float, default=1.0,
                    help='薄模型 z 方向增肥 (anime_62459 bbox 近立方, 1.5 反而缩小整体, 默认关)')
    ap.add_argument('--brighten', type=float, default=1.5)
    ap.add_argument('--gamma', type=float, default=0.9)
    ap.add_argument('--saturation', type=float, default=2.0)
    ap.add_argument('--lighting', default='lambert')
    ap.add_argument('--ambient', type=float, default=0.7)
    ap.add_argument('--out', default=os.path.join(HERE, 'anime_slices.bin'))
    ap.add_argument('--preview', default=os.path.join(HERE, 'anime_slices_preview.png'))
    args = ap.parse_args()

    if args.points:
        xyz, col = points_from_bin(args.points)
    else:
        xyz, col = points_from_glb(args.glb, args.samples, args.lighting, args.ambient)
    print(f'[pts] {len(xyz)} points', flush=True)
    col = color_adjust(col, args.brighten, args.gamma, args.saturation)
    vox = voxelize(xyz, col, args.z_stretch)

    d_step = 2 * math.pi / args.slices
    bufs = []
    lit_total = 0
    for i in range(args.slices):
        img = render_slice(vox, i * d_step, args.sub, d_step)
        on = to_1bit(img, args.thresh, not args.no_dither, i)
        lit_total += int(on.any(axis=2).sum())
        buf = pack_obs.pack_slice(on)
        bufs.append(buf + b'\0' * (SLICE_STRIDE - SLICE_DATA))
        if i % 45 == 0:
            print(f'  slice {i}: {int(on.any(axis=2).sum())} lit px', flush=True)

    blob = b''.join(bufs)
    with open(args.out, 'wb') as f:
        f.write(blob)
    print(f'[out] {args.out}: {len(blob)} bytes '
          f'({args.slices} x 0x{SLICE_STRIDE:X}, avg {lit_total // args.slices} lit px/slice)', flush=True)
    make_preview(bufs, args.slices, args.preview)


if __name__ == '__main__':
    main()
