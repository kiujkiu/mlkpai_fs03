#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
palace.py — 程序化紫禁城 (Forbidden City) 风格化表面点云生成器.

povstream.py 'palace' 动画源的几何部分. 设计坐标系 (设计单位 ≈ voxel):
z 轴 = 南北中轴 (南 -75 → 北 +75), y = 竖直 (地面 0), x = 东西 (±70).
高度按显示可读性夸张 ~2x (真紫禁城更扁), 总包络 ~140 x 70(h) x 150.

1-bit 显示约束: 颜色只用纯通道组合 (红/黄/白/绿/青), 中间值在 1-bit
抖动下不可控. 全部表面点云 (非实心), 总点数目标 300k-800k.

元素 (南→北): 外墙红 + 四角楼(黄攒尖顶) / 午门 U 形(开口朝南) /
金水河青色弧带 / 三层汉白玉台基 + 太和殿(重檐庑殿)/中和殿(攒尖)/
保和殿(庑殿) / 乾清宫区两殿 / 中轴白玉御道 / 西北东北绿树.

用法:
  import palace; xyz, col = palace.build_palace()   # y 已居中
  python3 palace.py                                  # 打印分色统计
"""
import math
import numpy as np

# 1-bit 安全色 (纯通道组合)
RED = (255.0, 0.0, 0.0)          # 宫墙/柱
YELLOW = (255.0, 255.0, 0.0)     # 金色琉璃瓦
WHITE = (255.0, 255.0, 255.0)    # 汉白玉台基/御道
GREEN = (0.0, 255.0, 0.0)        # 树
CYAN = (0.0, 255.0, 255.0)       # 金水河

DENSITY = 8.0                    # 点/设计单位², build_palace 可覆盖


# ================= 基础采样原语 =================

def _tri_pts(a, b, c, n, rng):
    """三角形内均匀 n 点."""
    u, v = rng.random(n), rng.random(n)
    m = u + v > 1.0
    u[m], v[m] = 1.0 - u[m], 1.0 - v[m]
    return a + u[:, None] * (b - a) + v[:, None] * (c - a)


def _tri_area(a, b, c):
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


def quad(a, b, c, d, color, density, rng):
    """四边形 (两三角拆分, 面积配点) 均匀表面点. c==d 时退化为三角形.
    返回 (pts (N,3), cols (N,3))."""
    a, b, c, d = (np.asarray(v, np.float64) for v in (a, b, c, d))
    s1, s2 = _tri_area(a, b, c), _tri_area(a, c, d)
    n1 = int(round(s1 * density))
    n2 = int(round(s2 * density))
    parts = []
    if n1 > 0:
        parts.append(_tri_pts(a, b, c, n1, rng))
    if n2 > 0:
        parts.append(_tri_pts(a, c, d, n2, rng))
    if not parts:
        parts.append(a[None, :])
    pts = np.concatenate(parts)
    return pts, np.tile(np.asarray(color, np.float64), (len(pts), 1))


ALL_FACES = ('x-', 'x+', 'y-', 'y+', 'z-', 'z+')


def box_surface(center, size, color, density, rng, faces=ALL_FACES):
    """轴对齐盒子表面点云 (默认 6 面, faces 可选子集). center/size = (x,y,z)."""
    cx, cy, cz = center
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    x0, x1, y0, y1, z0, z1 = cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz
    F = {
        'x-': ((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)),
        'x+': ((x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0)),
        'y-': ((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)),
        'y+': ((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)),
        'z-': ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)),
        'z+': ((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)),
    }
    ps, cs = [], []
    for f in faces:
        p, c = quad(*F[f], color, density, rng)
        ps.append(p); cs.append(c)
    return np.concatenate(ps), np.concatenate(cs)


def hip_roof(center, base_wh, ridge_len, height, color, density, rng, axis=None):
    """庑殿/歇山风格坡顶: 矩形底 (w x d, 在 y=center[1]) 收到一条正脊.
    center = (cx, y_base, cz); ridge_len=0 → 攒尖 (四棱锥).
    axis: 正脊方向 'x'/'z', 默认取底面长边."""
    cx, y0, cz = center
    w, d = base_wh
    y1 = y0 + height
    if axis is None:
        axis = 'x' if w >= d else 'z'
    hw, hd = w / 2.0, d / 2.0
    A = (cx - hw, y0, cz - hd)
    B = (cx + hw, y0, cz - hd)
    C = (cx + hw, y0, cz + hd)
    D = (cx - hw, y0, cz + hd)
    rl = ridge_len / 2.0
    if axis == 'x':
        R0, R1 = (cx - rl, y1, cz), (cx + rl, y1, cz)
        faces = [(A, B, R1, R0), (C, D, R0, R1),   # 南/北坡 (梯形)
                 (D, A, R0, R0), (B, C, R1, R1)]   # 东西山面 (三角)
    else:
        R0, R1 = (cx, y1, cz - rl), (cx, y1, cz + rl)
        faces = [(A, D, R1, R0), (C, B, R0, R1),
                 (A, B, R0, R0), (C, D, R1, R1)]
    ps, cs = [], []
    for f in faces:
        p, c = quad(*f, color, density, rng)
        ps.append(p); cs.append(c)
    return np.concatenate(ps), np.concatenate(cs)


def terrace(center_xz, y0, tiers, color, density, rng):
    """阶梯台基: tiers = [(w, d, h), ...] 自下而上叠 (侧面 + 顶面)."""
    cx, cz = center_xz
    ps, cs = [], []
    y = y0
    for w, d, h in tiers:
        p, c = box_surface((cx, y + h / 2.0, cz), (w, h, d), color, density, rng,
                           faces=('x-', 'x+', 'z-', 'z+', 'y+'))
        ps.append(p); cs.append(c)
        y += h
    return np.concatenate(ps), np.concatenate(cs)


def _tree_blob(center, r, color, density, rng):
    """球面树冠 (竖向压 0.8)."""
    n = max(8, int(4.0 * math.pi * r * r * density))
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    p = v * r
    p[:, 1] *= 0.8
    p += np.asarray(center, np.float64)
    return p, np.tile(np.asarray(color, np.float64), (n, 1))


# ================= 组合件 =================

def _hall(out, cx, cz, y0, w, d, wall_h, roof_h, density, rng,
          eave=2.5, ridge_frac=0.5):
    """单檐殿: 红墙盒 + 黄庑殿顶 (出檐 eave)."""
    out.append(box_surface((cx, y0 + wall_h / 2.0, cz), (w, wall_h, d),
                           RED, density, rng, faces=('x-', 'x+', 'z-', 'z+')))
    out.append(hip_roof((cx, y0 + wall_h, cz), (w + 2 * eave, d + 2 * eave),
                        w * ridge_frac, roof_h, YELLOW, density, rng, axis='x'))


def _hall_double_eave(out, cx, cz, y0, w, d, wall_h, roof_h, density, rng, eave=3.0):
    """重檐庑殿 (太和殿式): 下檐坡 + 红色夹层 + 上檐庑殿顶."""
    out.append(box_surface((cx, y0 + wall_h / 2.0, cz), (w, wall_h, d),
                           RED, density, rng, faces=('x-', 'x+', 'z-', 'z+')))
    y1 = y0 + wall_h
    lower_h = roof_h * 0.38
    out.append(hip_roof((cx, y1, cz), (w + 2 * eave, d + 2 * eave),
                        w * 0.72, lower_h, YELLOW, density, rng, axis='x'))
    y2 = y1 + lower_h
    w2, d2 = w * 0.64, d * 0.6
    out.append(box_surface((cx, y2 + 2.0, cz), (w2, 4.0, d2),
                           RED, density, rng, faces=('x-', 'x+', 'z-', 'z+')))
    out.append(hip_roof((cx, y2 + 4.0, cz), (w2 + 2 * eave, d2 + 2 * eave),
                        w2 * 0.5, roof_h * 0.62, YELLOW, density, rng, axis='x'))


def _river(out, density, rng):
    """金水河: 青色弧带, 弓形向南凸 (中点 z=-43, 两端 z=-34), 贴地."""
    x_half, width = 55.0, 3.5
    n = int(2.0 * x_half * width * density)
    x = rng.uniform(-x_half, x_half, n)
    zc = -34.0 - 9.0 * np.cos(x * math.pi / (2.0 * x_half))
    z = zc + rng.uniform(-width / 2.0, width / 2.0, n)
    y = rng.uniform(0.2, 1.2, n)
    out.append((np.stack([x, y, z], axis=1),
                np.tile(np.asarray(CYAN), (n, 1))))


def _axis_path(out, density, rng):
    """中轴御道: 白色窄带贯穿南北, 贴地 (其余地面留黑)."""
    p, c = quad((-3.0, 0.8, -73.0), (3.0, 0.8, -73.0),
                (3.0, 0.8, 73.0), (-3.0, 0.8, 73.0), WHITE, density * 0.5, rng)
    out.append((p, c))


# ================= 总装 =================

def build_palace(density=DENSITY, seed=20260709):
    """紫禁城点云. 返回 (xyz float32 (N,3), col float32 (N,3)),
    x/z 以中轴居中, y 以包络居中 (直接喂 gas.voxel_grid 的坐标系,
    调用方负责缩放进预算)."""
    rng = np.random.default_rng(seed)
    out = []   # [(pts, cols), ...]

    # ---- 外墙 (红, 低) ----
    wf = ('x-', 'x+', 'z-', 'z+', 'y+')
    for c_, s_ in [((0, 7, -74), (140, 14, 2)), ((0, 7, 74), (140, 14, 2)),
                   ((-69, 7, 0), (2, 14, 148)), ((69, 7, 0), (2, 14, 148))]:
        out.append(box_surface(c_, s_, RED, density, rng, faces=wf))

    # ---- 四角楼 (红身 + 黄攒尖顶) ----
    for sx in (-1, 1):
        for sz in (-1, 1):
            cx, cz = 66 * sx, 71 * sz
            out.append(box_surface((cx, 10, cz), (10, 20, 10), RED, density, rng,
                                   faces=('x-', 'x+', 'z-', 'z+')))
            out.append(hip_roof((cx, 20, cz), (13, 13), 0.0, 8.0,
                                YELLOW, density, rng))

    # ---- 午门 (U 形, 开口朝南): 主楼 + 两翼向南伸 ----
    _hall(out, 0, -58, 0, 46, 10, 24, 12, density, rng, ridge_frac=0.55)
    for sx in (-1, 1):
        out.append(box_surface((19 * sx, 12, -63), (8, 24, 20), RED, density, rng,
                               faces=('x-', 'x+', 'z-', 'z+')))
        out.append(hip_roof((19 * sx, 24, -63), (12, 24), 14.0, 9.0,
                            YELLOW, density, rng, axis='z'))

    # ---- 金水河 + 中轴御道 ----
    _river(out, density, rng)
    _axis_path(out, density, rng)

    # ---- 三层汉白玉台基 (三大殿) ----
    out.append(terrace((0, 2), 0, [(52, 50, 6), (44, 42, 6), (36, 34, 6)],
                       WHITE, density, rng))
    base_y = 18.0
    # 太和殿 (最大, 重檐庑殿)
    _hall_double_eave(out, 0, -10, base_y, 36, 20, 24, 24, density, rng)
    # 中和殿 (小方殿, 攒尖)
    out.append(box_surface((0, base_y + 8, 7), (12, 16, 12), RED, density, rng,
                           faces=('x-', 'x+', 'z-', 'z+')))
    out.append(hip_roof((0, base_y + 16, 7), (17, 17), 0.0, 14.0,
                        YELLOW, density, rng))
    # 保和殿 (中)
    _hall(out, 0, 20, base_y, 28, 16, 20, 18, density, rng)

    # ---- 乾清宫区 (北, 低台 + 两小殿) ----
    out.append(terrace((0, 55), 0, [(40, 30, 6)], WHITE, density, rng))
    _hall(out, 0, 48, 6, 26, 12, 18, 14, density, rng)
    _hall(out, 0, 62, 6, 20, 10, 14, 12, density, rng)

    # ---- 西北/东北绿树 ----
    for sx in (-1, 1):
        for cx, cz, r in [(52 * sx, 60, 5.0), (60 * sx, 52, 4.2), (47 * sx, 67, 4.5)]:
            out.append(_tree_blob((cx, 10, cz), r, GREEN, density, rng))

    xyz = np.concatenate([p for p, _ in out]).astype(np.float32)
    col = np.concatenate([c for _, c in out]).astype(np.float32)
    ymin, ymax = float(xyz[:, 1].min()), float(xyz[:, 1].max())
    xyz[:, 1] -= (ymin + ymax) / 2.0
    return xyz, col


def color_stats(col):
    """分色计数 {(r,g,b): n}."""
    u, cnt = np.unique(col.astype(np.int32), axis=0, return_counts=True)
    return {tuple(int(v) for v in row): int(n) for row, n in zip(u, cnt)}


if __name__ == '__main__':
    xyz, col = build_palace()
    print(f'total {len(xyz)} pts')
    names = {RED: 'red', YELLOW: 'yellow', WHITE: 'white',
             GREEN: 'green', CYAN: 'cyan'}
    for rgb, n in sorted(color_stats(col).items(), key=lambda kv: -kv[1]):
        tag = names.get(tuple(float(v) for v in rgb), '?')
        print(f'  {rgb} {tag:7s} {n}')
    print(f'bbox x {xyz[:, 0].min():.1f}..{xyz[:, 0].max():.1f}  '
          f'y {xyz[:, 1].min():.1f}..{xyz[:, 1].max():.1f}  '
          f'z {xyz[:, 2].min():.1f}..{xyz[:, 2].max():.1f}  '
          f'r_max {np.hypot(xyz[:, 0], xyz[:, 2]).max():.1f}')
