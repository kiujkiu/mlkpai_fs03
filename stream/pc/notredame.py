#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
notredame.py — 程序化巴黎圣母院 (Notre-Dame de Paris) 风格化表面点云生成器.

povstream.py 'notredame' 动画源的几何部分, 套 palace.py 同一坐标系:
z 轴 = 纵深主轴 (西立面朝南 -z, 后殿朝北 +z, 关于 x=0 对称),
y = 竖直 (地面 0), x = 东西 (横翼方向). 设计包络 ~65(x) x 135(h) x 140(z),
哥特式竖直感刻意夸张 (真圣母院塔 69m/长 128m, 这里高长比拉到 ~1x).

1-bit 显示约束: 只用纯通道组合色. 全部表面点云, 总点数目标 300k-800k.

元素 (南→北): 西立面双塔 (白, 平顶, 立面上 2:1 竖长比) + 三门洞留黑 +
玫瑰窗 (蓝盘红心红辐条) / 中殿 (白墙 + 高侧窗留黑 + 铜绿 GREEN 坡脊顶) /
飞扶壁 (白弧带 + 白柱墩, 每侧 6 + 后殿放射 5) / 十字翼 (东西短臂,
两端小玫瑰窗) / 交叉点尖塔 (白八棱锥 + 黄尖) / 半圆后殿 (半圆柱 + 半锥顶).
地面留黑.

用法:
  import notredame; xyz, col = notredame.build_notredame()   # y 已居中
  python3 notredame.py                                        # 打印分色统计
"""
import math
import numpy as np

from palace import quad, box_surface, hip_roof, WHITE, GREEN, RED, YELLOW

BLUE = (0.0, 0.0, 255.0)         # 玫瑰窗玻璃

DENSITY = 8.0                    # 点/设计单位², build_notredame 可覆盖

# ---- 总体布局常量 (设计单位) ----
FACADE_Z0, FACADE_D = -70.0, 18.0        # 西立面前脸 z / 进深 (z -70..-52)
FACADE_W, FACADE_H = 44.0, 56.0          # 立面总宽 / 立面体高
TOWER_W, TOWER_TOP = 17.0, 95.0          # 塔宽 (x±13.5 心, 塔间开天 x±5) / 塔顶
TOWER_CX = 13.5
NAVE_Z0, NAVE_Z1 = -52.0, 50.0           # 中殿+唱诗席 z 范围
AISLE_W, AISLE_H = 38.0, 22.0            # 侧廊全宽 / 高
CLER_W, WALL_H = 30.0, 45.0              # 高侧窗层全宽 / 主墙顶
RIDGE_H = 55.0                           # 屋脊高 (铜绿顶)
TRAN_Z0, TRAN_Z1, TRAN_XW = -2.0, 14.0, 64.0   # 十字翼 z 范围 / 全展宽 (x±32)
CROSS_Z = (TRAN_Z0 + TRAN_Z1) / 2.0      # 交叉点 z = 6
SPIRE_TOP = 128.0                        # 尖塔白锥顶
TIP_TOP = 135.0                          # 黄尖顶 (全模型最高)
APSE_ZC = 50.0                           # 后殿圆心 z
APSE_R_LO, APSE_R_HI = 19.0, 15.0        # 回廊半径 / 高侧层半径
ROSE_Y, ROSE_R = 36.0, 7.5               # 西玫瑰窗心高 / 半径


# ================= 采样原语 (补 palace 没有的) =================

def plane_pts(origin, u_vec, v_vec, u_len, v_len, color, density, rng,
              reject=None):
    """平面矩形 origin + u·u_vec + v·v_vec (u∈[0,u_len], v∈[0,v_len]) 均匀
    表面点; reject(u, v) -> bool mask 挖洞 (门洞/窗洞留黑)."""
    n = max(1, int(u_len * v_len * density))
    u = rng.uniform(0.0, u_len, n)
    v = rng.uniform(0.0, v_len, n)
    if reject is not None:
        keep = ~reject(u, v)
        u, v = u[keep], v[keep]
    p = (np.asarray(origin, np.float64)[None, :]
         + u[:, None] * np.asarray(u_vec, np.float64)[None, :]
         + v[:, None] * np.asarray(v_vec, np.float64)[None, :])
    return p, np.tile(np.asarray(color, np.float64), (len(p), 1))


def _arch_mask(u, v, uc, w, h):
    """尖券门洞近似: 矩形 (宽 w 高 h-w/2) + 顶部半圆 (r=w/2)."""
    r = w / 2.0
    rect = (np.abs(u - uc) < r) & (v < h - r)
    dome = (u - uc) ** 2 + (v - (h - r)) ** 2 < r * r
    return rect | dome


def _oct_factor(theta):
    """正八边形边界半径因子 (相对外接圆), 用于八棱尖塔截面."""
    seg = np.mod(theta + math.pi / 8.0, math.pi / 4.0) - math.pi / 8.0
    return math.cos(math.pi / 8.0) / np.cos(seg)


def rose_window(center, axis, r, density, rng, spokes=8):
    """玫瑰窗圆盘: 蓝玻璃盘 + 红心 + 红辐条 (点按区域互斥, 保持 1-bit 纯色).
    axis='z' 盘面在 xy 平面 (西立面), axis='x' 盘面在 zy 平面 (翼端)."""
    n = max(64, int(math.pi * r * r * density * 3.0))     # 玫瑰窗加密 3x
    rad = r * np.sqrt(rng.random(n))
    th = rng.uniform(0.0, 2.0 * math.pi, n)
    a, b = rad * np.cos(th), rad * np.sin(th)
    core = rad < r * 0.22
    if spokes > 0:
        seg = np.mod(th + math.pi / spokes, 2.0 * math.pi / spokes) \
            - math.pi / spokes
        spoke = (rad * np.abs(np.sin(seg)) < 0.45) & ~core
    else:
        spoke = np.zeros(n, bool)
    col = np.tile(np.asarray(BLUE, np.float64), (n, 1))
    col[core | spoke] = RED
    cx, cy, cz = center
    p = np.empty((n, 3), np.float64)
    if axis == 'z':
        p[:, 0], p[:, 1], p[:, 2] = cx + a, cy + b, cz
    else:
        p[:, 2], p[:, 1], p[:, 0] = cz + a, cy + b, cx
    return p, col


def _half_cyl(czc, r, y0, y1, color, density, rng):
    """北向半圆柱侧面 (后殿墙): 角 0..π (东→北→西), 圆心 (0, ·, czc)."""
    n = max(8, int(math.pi * r * (y1 - y0) * density))
    a = rng.uniform(0.0, math.pi, n)
    y = rng.uniform(y0, y1, n)
    p = np.stack([r * np.cos(a), y, czc + r * np.sin(a)], axis=1)
    return p, np.tile(np.asarray(color, np.float64), (n, 1))


def _half_cone(czc, r0, y0, y1, color, density, rng):
    """北向半锥面 (后殿顶): 底半径 r0@y0 收到轴点 (0,y1,czc)."""
    n = max(8, int(0.5 * math.pi * r0 * math.hypot(r0, y1 - y0) * density))
    s = 1.0 - np.sqrt(rng.random(n))                      # 面积均匀
    a = rng.uniform(0.0, math.pi, n)
    r = r0 * (1.0 - s)
    p = np.stack([r * np.cos(a), y0 + (y1 - y0) * s, czc + r * np.sin(a)],
                 axis=1)
    return p, np.tile(np.asarray(color, np.float64), (n, 1))


# ================= 组合件 =================

def _facade(out, density, rng):
    """西立面 (朝南 -z): 双塔 + 立面体 + 三门洞留黑 + 玫瑰窗."""
    zf, zb = FACADE_Z0, FACADE_Z0 + FACADE_D             # 前/后脸 z
    zc = (zf + zb) / 2.0
    hw = FACADE_W / 2.0                                  # 22

    # 前脸 (z=-70 平面): 挖三门洞 + 玫瑰窗圆洞, u = x+22, v = y
    def rej(u, v):
        m = _arch_mask(u, v, hw, 10.0, 18.0)             # 中门
        m |= _arch_mask(u, v, hw - 13.0, 7.0, 14.0)      # 侧门 x=-13
        m |= _arch_mask(u, v, hw + 13.0, 7.0, 14.0)      # 侧门 x=+13
        m |= (u - hw) ** 2 + (v - ROSE_Y) ** 2 < (ROSE_R + 0.6) ** 2  # 玫瑰洞
        return m
    out.append(plane_pts((-hw, 0.0, zf), (1, 0, 0), (0, 1, 0),
                         FACADE_W, FACADE_H, WHITE, density, rng, rej))
    # 玫瑰窗盘 (微凸出前脸 0.6)
    out.append(rose_window((0.0, ROSE_Y, zf - 0.6), 'z', ROSE_R, density, rng))

    # 立面体其余面
    out.append(box_surface((0.0, FACADE_H / 2.0, zc),
                           (FACADE_W, FACADE_H, FACADE_D), WHITE, density, rng,
                           faces=('x-', 'x+', 'z+')))
    # 双塔间立面顶条 (塔间露天)
    p, c = quad((-5.0, FACADE_H, zf), (5.0, FACADE_H, zf),
                (5.0, FACADE_H, zb), (-5.0, FACADE_H, zb), WHITE, density, rng)
    out.append((p, c))

    # 双塔上段 (y 56..95, 平顶, 竖长比 (95-56)/17 ≈ 2.3:1)
    th = TOWER_TOP - FACADE_H

    def belfry(u, v, _w=TOWER_W / 2.0):
        # 双钟窗竖缝留黑 (u = 塔内局部 x, v = y-56), 前后脸都挖 → 透光
        slot = (np.abs(u - (_w - 3.5)) < 1.5) | (np.abs(u - (_w + 3.5)) < 1.5)
        return slot & (v > 8.0) & (v < 30.0)
    for sx in (-1, 1):
        cx = TOWER_CX * sx
        for zw in (zf, zb):                              # 前脸 + 后脸
            out.append(plane_pts((cx - TOWER_W / 2.0, FACADE_H, zw),
                                 (1, 0, 0), (0, 1, 0), TOWER_W, th,
                                 WHITE, density, rng, belfry))
        out.append(box_surface((cx, FACADE_H + th / 2.0, zc),
                               (TOWER_W, th, FACADE_D), WHITE, density, rng,
                               faces=('x-', 'x+', 'y+')))


def _nave(out, density, rng):
    """中殿 + 唱诗席长身: 侧廊 + 高侧窗层 (窗洞留黑) + 铜绿坡脊顶."""
    zlen = NAVE_Z1 - NAVE_Z0
    zc = (NAVE_Z0 + NAVE_Z1) / 2.0
    # 侧廊外墙
    out.append(box_surface((0.0, AISLE_H / 2.0, zc), (AISLE_W, AISLE_H, zlen),
                           WHITE, density, rng, faces=('x-', 'x+')))
    # 侧廊坡顶条 (外 19,22.5 → 内 15,26)
    for sx in (-1, 1):
        p, c = quad((sx * AISLE_W / 2.0, AISLE_H + 0.5, NAVE_Z0),
                    (sx * CLER_W / 2.0, 26.0, NAVE_Z0),
                    (sx * CLER_W / 2.0, 26.0, NAVE_Z1),
                    (sx * AISLE_W / 2.0, AISLE_H + 0.5, NAVE_Z1),
                    WHITE, density, rng)
        out.append((p, c))
    # 高侧窗层墙 (y 22..45), 尖窗留黑: 窗心 z ∈ {-46..46}
    win_u = np.asarray([6.0, 19.0, 32.0, 45.0, 72.0, 85.0, 98.0])  # = z+52

    def lancet(u, v):
        m = np.zeros(len(u), bool)
        for uc in win_u:
            m |= _arch_mask(u - uc + 8.0, v - 3.0, 8.0, 3.2, 16.0)
        return m
    for sx in (-1, 1):
        out.append(plane_pts((sx * CLER_W / 2.0, AISLE_H, NAVE_Z0),
                             (0, 0, 1), (0, 1, 0), zlen, WALL_H - AISLE_H,
                             WHITE, density, rng, lancet))
    # 铜绿大坡顶: 脊沿 z, y 45→55
    out.append(hip_roof((0.0, WALL_H, zc), (CLER_W + 2.0, zlen + 2.0),
                        zlen - 16.0, RIDGE_H - WALL_H, GREEN, density, rng,
                        axis='z'))


def _transept(out, density, rng):
    """十字翼: 东西短臂 (x ±32) + 铜绿脊沿 x + 两端小玫瑰窗."""
    d = TRAN_Z1 - TRAN_Z0
    zc = CROSS_Z
    hx = TRAN_XW / 2.0
    out.append(box_surface((0.0, WALL_H / 2.0, zc), (TRAN_XW, WALL_H, d),
                           WHITE, density, rng, faces=('z-', 'z+')))
    # 端墙 (x=±32) 挖玫瑰圆洞, u = z-TRAN_Z0, v = y
    def rej(u, v):
        return (u - d / 2.0) ** 2 + (v - 33.0) ** 2 < 5.6 ** 2
    for sx in (-1, 1):
        out.append(plane_pts((sx * hx, 0.0, TRAN_Z0), (0, 0, 1), (0, 1, 0),
                             d, WALL_H, WHITE, density, rng, rej))
        out.append(rose_window((sx * (hx + 0.6), 33.0, zc), 'x', 5.0,
                               density, rng, spokes=6))
    # 铜绿翼顶: 脊沿 x
    out.append(hip_roof((0.0, WALL_H, zc), (TRAN_XW + 4.0, d + 4.0),
                        TRAN_XW - 14.0, RIDGE_H - WALL_H, GREEN, density, rng,
                        axis='x'))


def _spire(out, density, rng):
    """交叉点尖塔 (2019 前 Viollet-le-Duc 铅塔轮廓): 白八棱鼓座 + 细高
    八棱锥 + 小黄尖 (读作高光). 全模型制高点."""
    k = density / 8.0
    cx, cz = 0.0, CROSS_Z
    # 八棱鼓座 y 50..58 (穿出屋脊)
    n = int(700 * k)
    th = rng.uniform(0.0, 2.0 * math.pi, n)
    r = 5.0 * _oct_factor(th)
    y = rng.uniform(50.0, 58.0, n)
    out.append((np.stack([cx + r * np.cos(th), y, cz + r * np.sin(th)], axis=1),
                np.tile(np.asarray(WHITE), (n, 1))))
    # 白八棱锥 y 56 → 128 (t 均匀采样 → 越往上越密, 保细尖可读)
    n = int(2600 * k)
    t = rng.random(n)
    th = rng.uniform(0.0, 2.0 * math.pi, n)
    r = 4.5 * (1.0 - t) * _oct_factor(th)
    y = 56.0 + (SPIRE_TOP - 56.0) * t
    out.append((np.stack([cx + r * np.cos(th), y, cz + r * np.sin(th)], axis=1),
                np.tile(np.asarray(WHITE), (n, 1))))
    # 黄尖 y 124 → 135
    n = int(400 * k)
    t = rng.random(n)
    th = rng.uniform(0.0, 2.0 * math.pi, n)
    r = 0.9 * (1.0 - 0.8 * t)
    y = 124.0 + (TIP_TOP - 124.0) * t
    out.append((np.stack([cx + r * np.cos(th), y, cz + r * np.sin(th)], axis=1),
                np.tile(np.asarray(YELLOW), (n, 1))))


def _fb_arc(out, p_wall, p_pier, density, rng, width=1.8):
    """飞扶壁弧带: 从墙上沿 p_wall (切线竖直) 扫到柱墩顶 p_pier (切线水平)
    的四分之一椭圆细条, 厚度沿径向 15%."""
    x0, y0, z0 = p_wall
    x1, y1, z1 = p_pier
    dx, dz, dy = x1 - x0, z1 - z0, y0 - y1
    arc_len = 0.25 * 2.0 * math.pi * max(math.hypot(dx, dz), dy)
    n = max(32, int(arc_len * width * density * 1.5))     # 弧带加密 1.5x
    phi = rng.uniform(0.0, math.pi / 2.0, n)
    f = rng.uniform(0.85, 1.0, n)                         # 径向厚度
    s, c = np.sin(phi), np.cos(phi)
    ux, uz = (dz / max(math.hypot(dx, dz), 1e-6),
              -dx / max(math.hypot(dx, dz), 1e-6))        # 横向 (宽度) 方向
    w = rng.uniform(-width / 2.0, width / 2.0, n)
    p = np.stack([x0 + dx * s * f + ux * w,
                  y1 + dy * c * f,
                  z0 + dz * s * f + uz * w], axis=1)
    out.append((p, np.tile(np.asarray(WHITE), (n, 1))))


def _flying_buttresses(out, density, rng):
    """飞扶壁: 中殿/唱诗席每侧 6 (白柱墩 + 小尖顶 + 弧带), 后殿放射 5."""
    x_wall, x_pier, y_wall, y_pier = CLER_W / 2.0 + 0.5, 21.5, 44.0, 28.0
    for sx in (-1, 1):
        for z in (-48.0, -39.5, -26.5, -13.5, 26.5, 39.5):
            out.append(box_surface((sx * x_pier, y_pier / 2.0, z),
                                   (2.5, y_pier, 2.5), WHITE, density, rng,
                                   faces=('x-', 'x+', 'z-', 'z+')))
            out.append(hip_roof((sx * x_pier, y_pier, z), (3.4, 3.4), 0.0,
                                5.0, WHITE, density, rng))
            _fb_arc(out, (sx * x_wall, y_wall, z), (sx * x_pier, y_pier, z),
                    density, rng)
    # 后殿放射扶壁 (绕半圆 5 支)
    for a in (math.pi / 6, math.pi / 3, math.pi / 2,
              2 * math.pi / 3, 5 * math.pi / 6):
        dx, dz = math.cos(a), math.sin(a)
        r0, r1 = APSE_R_HI + 0.5, 21.5
        pw = (r0 * dx, y_wall, APSE_ZC + r0 * dz)
        pp = (r1 * dx, y_pier, APSE_ZC + r1 * dz)
        # 柱墩 (圆柱近似)
        n = max(16, int(2.0 * math.pi * 1.2 * y_pier * density * 0.5))
        th = rng.uniform(0.0, 2.0 * math.pi, n)
        p = np.stack([pp[0] + 1.2 * np.cos(th),
                      rng.uniform(0.0, y_pier, n),
                      pp[2] + 1.2 * np.sin(th)], axis=1)
        out.append((p, np.tile(np.asarray(WHITE), (n, 1))))
        _fb_arc(out, pw, pp, density, rng)


def _apse(out, density, rng):
    """半圆后殿 (北端): 回廊半圆柱 + 环顶 + 高侧层半圆柱 + 铜绿半锥顶."""
    out.append(_half_cyl(APSE_ZC, APSE_R_LO, 0.0, AISLE_H, WHITE, density, rng))
    out.append(_half_cyl(APSE_ZC, APSE_R_HI, AISLE_H, WALL_H, WHITE,
                         density, rng))
    # 回廊环顶 (半环带 y 22.5, r 15..19)
    n = max(8, int(0.5 * math.pi * (APSE_R_LO ** 2 - APSE_R_HI ** 2) * density))
    a = rng.uniform(0.0, math.pi, n)
    r = np.sqrt(rng.uniform(APSE_R_HI ** 2, APSE_R_LO ** 2, n))
    p = np.stack([r * np.cos(a), np.full(n, AISLE_H + 0.5),
                  APSE_ZC + r * np.sin(a)], axis=1)
    out.append((p, np.tile(np.asarray(WHITE), (n, 1))))
    # 铜绿半锥顶: rim r16 @45 → 脊端点 (0, 55, APSE_ZC)
    out.append(_half_cone(APSE_ZC, APSE_R_HI + 1.0, WALL_H, RIDGE_H, GREEN,
                          density, rng))


# ================= 总装 =================

def build_notredame(density=DENSITY, seed=20260709):
    """巴黎圣母院点云. 返回 (xyz float32 (N,3), col float32 (N,3)),
    x/z 关于中轴居中 (西立面朝 -z), y 以包络居中 (喂 gas.voxel_grid 坐标系,
    调用方负责缩放进预算)."""
    rng = np.random.default_rng(seed)
    out = []   # [(pts, cols), ...]
    _facade(out, density, rng)
    _nave(out, density, rng)
    _transept(out, density, rng)
    _spire(out, density, rng)
    _flying_buttresses(out, density, rng)
    _apse(out, density, rng)
    xyz = np.concatenate([p for p, _ in out]).astype(np.float32)
    col = np.concatenate([c for _, c in out]).astype(np.float32)
    ymin, ymax = float(xyz[:, 1].min()), float(xyz[:, 1].max())
    xyz[:, 1] -= (ymin + ymax) / 2.0
    return xyz, col


if __name__ == '__main__':
    from palace import color_stats
    xyz, col = build_notredame()
    print(f'total {len(xyz)} pts')
    names = {WHITE: 'white', GREEN: 'green', BLUE: 'blue',
             RED: 'red', YELLOW: 'yellow'}
    for rgb, n in sorted(color_stats(col).items(), key=lambda kv: -kv[1]):
        tag = names.get(tuple(float(v) for v in rgb), '?')
        print(f'  {rgb} {tag:7s} {n}')
    print(f'bbox x {xyz[:, 0].min():.1f}..{xyz[:, 0].max():.1f}  '
          f'y {xyz[:, 1].min():.1f}..{xyz[:, 1].max():.1f}  '
          f'z {xyz[:, 2].min():.1f}..{xyz[:, 2].max():.1f}  '
          f'r_max {np.hypot(xyz[:, 0], xyz[:, 2]).max():.1f}')
