#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_anime_slices.py — anime GLB/点云 → 360 子午面切片 → FS03 DDR 镜像 (2026-07-08).

几何约定 (multivox 式带符号半径 + 2026-07-27 新增偏移轴支持):
  u = X-79.5 ∈ [-79.5,+79.5] 沿屏宽; Y 0..179 = 高 (上→下)。
  --gap-mm 0 (穿心, 旧行为): 转轴在屏 X 中心, u = 带符号半径, slice i (θ_i=i°)
    的 u>0 半边显示方位角 θ_i 半平面, u<0 半边显示 θ_i+180°。
  --gap-mm D>0 (正反双屏有间距, 屏不再过圆心): 每屏面到轴垂距 D/2, 屏面是
    与转轴平行的**偏移平面**而非子午面。世界点 = u·û(θ) + (D/2)·n̂(θ),
    û=(cosθ,sinθ), n̂=(-sinθ,cosθ)。等价 R=√(u²+(D/2)²), 方位=θ+atan2(D/2,u)。
    ⚠ 半径 < D/2 的圆柱是**中心盲区**, 几何固有扫不到; 但轴心是模型内部,
      D=13.8mm 时盲区仅占截面积 0.85% (占直径 9.2%), 外观基本无损。
    ⚠ 此时 360 片**各不相同** (穿心时 i 与 i+180 互为镜像, 是 2× 冗余存法)。
    ✅ 两屏关于轴对称 ⇒ 屏B@θ ≡ 屏A@(θ+180), PHASE_B=180 与单份 DDR 数据不变。
  🔴 --mirror-u 默认 **ON** (2026-07-27 上板实测): 屏 X 轴物理方向与 û(θ) 相反,
     每片须对屏中心轴做轴对称。穿心几何下 mirror(slice i)==slice i+180, 手性错误
     等价于相位转 180° 会被光电标定悄悄吸收掉 —— 有间距后才第一次变成肉眼可见的镜像。
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
PITCH_MM = 0.9375          # P0.9375 COB 屏点间距 (体素单位 = 1 像素 = 0.9375mm)
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


def voxel_grid(p, col, verbose=True, ssaa=1):
    """归一化点 (voxel 坐标, 原点在中心) → 体素格颜色平均 (GR,GH,GR,3)。
    ssaa>1: N× 细分格占据率作覆盖权重 — 表面片穿满一格 ≈ N² 子格 = 1.0,
    边缘体素按覆盖率调暗, 下游 Bayer 抖动把灰度变点密度 (1-bit 灰度抗锯齿)。
    覆盖率统计靠点采样撑, ssaa=3 建议 --samples ≥ 1.2M。"""
    gx = np.clip(np.rint(p[:, 0]).astype(np.int32) + GR // 2, 0, GR - 1)
    gy = np.clip(np.rint(p[:, 1]).astype(np.int32) + GH // 2, 0, GH - 1)
    gz = np.clip(np.rint(p[:, 2]).astype(np.int32) + GR // 2, 0, GR - 1)
    acc = np.zeros((GR, GH, GR, 3), np.float32)
    cnt = np.zeros((GR, GH, GR), np.float32)
    np.add.at(acc, (gx, gy, gz), col)
    np.add.at(cnt, (gx, gy, gz), 1.0)
    occ = cnt > 0
    acc[occ] /= cnt[occ][:, None]
    if ssaa > 1:
        n = int(ssaa)
        # 子格 rint 对齐主格: 主格 i 的子格集 = {n*i-n//2 .. n*i+n//2}
        f = np.rint(p * n).astype(np.int64)
        subs = np.unique(f, axis=0)
        mx = np.clip((subs[:, 0] + n // 2) // n + GR // 2, 0, GR - 1).astype(np.int32)
        my = np.clip((subs[:, 1] + n // 2) // n + GH // 2, 0, GH - 1).astype(np.int32)
        mz = np.clip((subs[:, 2] + n // 2) // n + GR // 2, 0, GR - 1).astype(np.int32)
        cov = np.zeros((GR, GH, GR), np.float32)
        np.add.at(cov, (mx, my, mz), 1.0)
        np.clip(cov / float(n * n), 0.0, 1.0, out=cov)
        acc *= cov[:, :, :, None]
        if verbose:
            e = cov[occ]
            print(f'[vox] ssaa{n} coverage mean={e.mean():.2f} '
                  f'edge(<0.67)={(e < 0.67).mean():.0%}', flush=True)
    if verbose:
        print(f'[vox] {int(occ.sum())} occupied cells', flush=True)
    return acc


def voxelize(xyz, col, z_stretch):
    """居中 + 等比缩放 + 体素化 (静态一步到位, 保持旧 API)。"""
    return voxel_grid(normalize_points(xyz, z_stretch), col)


def render_slice(vox, theta, sub, d_step, axis_off=0.0, mirror_u=False):
    """角度 theta 的观察者视角 float 图 (H,W,3)。sub 个子角度 max 混合。

    axis_off = 屏面到转轴的垂距 (体素单位 = 屏像素)。
      0     → 屏面穿心, **逐字节退化成旧行为** (过原点射线, slice 编号语义不变)
      >0    → 正反双屏有间距: 屏面是与转轴平行、垂距 axis_off 的**偏移平面**,
              不再是子午面。沿用旧约定 u 走 û(θ)=(cosθ,sinθ), 偏移走法向
              n̂(θ)=(-sinθ,cosθ): 世界点 = u·û(θ) + axis_off·n̂(θ)。
              等价极坐标: R=√(u²+axis_off²), 方位 = θ + atan2(axis_off, u)
              —— 一列像素不再共面, 近轴处方位偏移极大 (u=0 处直接偏 90°)。
    ⚠ 半径 < axis_off 的圆柱永远扫不到 (中心盲区), 属几何固有, 软件补不回来。
    屏 B 关于转轴对称 (垂距 -axis_off) ⇒ 屏B@θ ≡ 屏A@(θ+180), 故 PHASE_B=180
      仍严格成立, DDR 只需一份 360 片 (但此时 360 片各不相同, 不再是 2× 冗余)。
    """
    D = np.arange(W, dtype=np.float32) - (W - 1) / 2.0    # u: 沿屏宽, ±79.5
    if mirror_u:
        D = -D          # 屏 X 轴物理方向与 û(θ) 相反 → 每片对中心轴做轴对称
    gy = (H - 1) - np.arange(H)          # 屏 Y 上→下, 体素 y 下→上
    img = np.zeros((H, W, 3), np.float32)
    offs = [0.0] if sub <= 1 else [(k / sub - (sub - 1) / (2.0 * sub)) * d_step for k in range(sub)]
    for off in offs:
        c, s = math.cos(theta + off), math.sin(theta + off)
        # 偏移平面: u 沿 û=(c,s) (旧约定不动), 垂距沿法向 n̂=(-s,c)
        # axis_off=0 时化简为 (D*c, D*s) = 旧代码, 逐字节一致
        wxf = D * c - axis_off * s
        wzf = D * s + axis_off * c
        wx = np.clip(np.rint(wxf).astype(np.int32) + GR // 2, 0, GR - 1)
        wz = np.clip(np.rint(wzf).astype(np.int32) + GR // 2, 0, GR - 1)
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
    ap.add_argument('--gap-mm', type=float, default=13.8,
                    help='正反双屏两屏面间距 mm (每屏到轴垂距 = 一半); 0 = 穿心旧行为')
    ap.add_argument('--no-mirror-u', dest='mirror_u', action='store_false', default=True,
                    help='关掉中心轴镜像。默认开: 本机屏 X 轴物理方向与 û(θ) 相反, 2026-07-27 上板实测确认')
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

    axis_off = (args.gap_mm / 2.0) / PITCH_MM      # mm → 体素(像素)单位
    if axis_off > 0:
        r_out = math.hypot(R_BUDGET + 0.5, axis_off)
        print(f'[geom] 双屏间距 {args.gap_mm}mm → 每屏到轴垂距 '
              f'{args.gap_mm/2:.2f}mm = {axis_off:.2f}px (偏移平面, 非子午面)', flush=True)
        print(f'[geom] 中心盲区 直径 {args.gap_mm:.1f}mm = {2*axis_off:.1f}px, '
              f'占截面积 {100*(axis_off/r_out)**2:.2f}% | 有效外径 {r_out:.2f}px | '
              f'整屏跨方位 {2*math.degrees(math.atan2(R_BUDGET+0.5, axis_off)):.1f}°', flush=True)
    else:
        print('[geom] gap=0 → 穿心子午面 (旧行为)', flush=True)
    print('[geom] mirror_u=%s (中心轴镜像; 默认 ON, 本机实测)'
          % ('ON' if args.mirror_u else 'OFF'), flush=True)
    d_step = 2 * math.pi / args.slices
    bufs = []
    lit_total = 0
    for i in range(args.slices):
        img = render_slice(vox, i * d_step, args.sub, d_step, axis_off, args.mirror_u)
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
