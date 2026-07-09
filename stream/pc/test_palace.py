#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_palace.py — palace 动画源测试 + 正交投影 preview 生成 (无板无网络).

  python3 test_palace.py            # 全部, 顺便产出 preview_palace/*.png

(a) 点云: 总量 300k-800k / 颜色全是 1-bit 纯色 / x 对称 / 分区结构
    (三大殿区黄顶在红墙上、红墙在白台上; 金水河在南贴地; 树在北) /
    预缩放后 zoom 最大时径向+竖直不出预算
(b) 4 帧渲染: packed 尺寸 = 4,423,680 / 帧互异 / yaw 90° 把北侧绿树
    甩到侧面 (旋转确实作用在体素化之前)
(c) preview: 首帧+第二帧 俯视(顶面首命中)/正视(南面首命中) PNG +
    packed bin unpack 切片拼图, 存 preview_palace/ 供人眼复核
"""
import os
import sys
import argparse
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import povstream                                    # noqa: E402
import palace                                       # noqa: E402
import pack_obs                                     # noqa: E402
import gen_anime_slices as gas                      # noqa: E402

PREVIEW = os.path.join(HERE, 'preview_palace')
ALLOWED = {(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
           (0, 255, 255), (255, 0, 255), (255, 255, 255)}
GR, GH = gas.GR, gas.GH


def mkargs(**kw):
    ap = argparse.ArgumentParser()
    povstream.add_render_opts(ap)
    a = ap.parse_args([])
    a.anim = 'palace'
    a.render_slices = 8
    a.sub = 1
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def mask(col, rgb):
    return np.all(col.astype(np.int32) == np.asarray(rgb, np.int32), axis=1)


# ================= (a) 点云结构 =================

def test_pointcloud():
    xyz, col = palace.build_palace()
    n = len(xyz)
    assert 300_000 <= n <= 800_000, f'total {n} out of 300k-800k'
    stats = palace.color_stats(col)
    assert set(stats) <= ALLOWED, f'non-pure colors: {set(stats) - ALLOWED}'
    print('[a] colors:', {k: v for k, v in sorted(stats.items(), key=lambda kv: -kv[1])})

    red, yel = mask(col, palace.RED), mask(col, palace.YELLOW)
    whi, grn, cyn = mask(col, palace.WHITE), mask(col, palace.GREEN), mask(col, palace.CYAN)
    assert all(m.sum() > 1000 for m in (red, yel, whi, grn, cyn)), 'missing color class'

    # x 对称 (整体 + 各色)
    for name, m in [('all', slice(None)), ('red', red), ('yellow', yel), ('white', whi)]:
        mx = float(xyz[m, 0].mean())
        assert abs(mx) < 2.0, f'{name} mean_x {mx:.2f} not centered'

    # 三大殿区分层: 黄顶 > 红墙 > 白台 (y)
    zone = (np.abs(xyz[:, 0]) < 30) & (xyz[:, 2] > -25) & (xyz[:, 2] < 32)
    y_y = float(xyz[zone & yel, 1].mean())
    y_r = float(xyz[zone & red, 1].mean())
    y_w = float(xyz[zone & whi, 1].mean())
    assert y_y > y_r > y_w, f'hall zone layering broken: yel {y_y:.1f} red {y_r:.1f} whi {y_w:.1f}'
    print(f'[a] hall zone mean-y: yellow {y_y:.1f} > red {y_r:.1f} > white {y_w:.1f}')

    # 金水河: 全在南半场贴地; 树: 全在北半场
    assert xyz[cyn, 2].max() < -25, 'river not in south'
    assert xyz[cyn, 1].max() < xyz[:, 1].min() + 3.0, 'river not at ground'
    assert xyz[grn, 2].min() > 40, 'trees not in north'
    # 中轴白御道贯穿
    path = whi & (np.abs(xyz[:, 0]) <= 3.5)
    assert xyz[path, 2].min() < -70 and xyz[path, 2].max() > 70, 'axis path incomplete'

    # 预缩放 clip-safe: zoom 最大时径向/竖直都在预算内
    p0, _ = povstream.palace_points_prescaled()
    r = float(np.hypot(p0[:, 0], p0[:, 2]).max()) * povstream.PALACE_ZOOM_MAX
    h = float(np.abs(p0[:, 1]).max()) * povstream.PALACE_ZOOM_MAX
    assert r <= gas.R_BUDGET and h <= gas.H_BUDGET, f'clip-unsafe r {r:.1f} h {h:.1f}'
    print(f'[a] OK: {n} pts, prescaled max-zoom r {r:.1f}/{gas.R_BUDGET} h {h:.1f}/{gas.H_BUDGET}')


# ================= 投影 (首命中, 供数值检查 + 人眼 PNG) =================

def topdown(vox):
    """俯视 RGB (行=z 北在上, 列=x): 每 (x,z) 取最高占用体素颜色."""
    occ = vox.max(axis=3) > 0
    ys = np.where(occ, np.arange(GH)[None, :, None], -1).max(axis=1)
    img = np.zeros((GR, GR, 3), np.uint8)
    xi, zi = np.nonzero(ys >= 0)
    img[zi, xi] = vox[xi, ys[xi, zi], zi]
    return img[::-1]                        # z 大 (北) 在上


def frontview(vox):
    """南面正视 RGB (行=y 上在上, 列=x): 每 (x,y) 取 z 最小占用体素颜色."""
    occ = vox.max(axis=3) > 0
    zs = np.where(occ, np.arange(GR)[None, None, :], GR + 1).min(axis=2)
    img = np.zeros((GH, GR, 3), np.uint8)
    xi, yi = np.nonzero(zs <= GR)
    img[yi, xi] = vox[xi, yi, zs[xi, yi]]
    return img[::-1]                        # y 大在上


def save_png(img, name, scale=3):
    os.makedirs(PREVIEW, exist_ok=True)
    p = os.path.join(PREVIEW, name)
    Image.fromarray(img).resize((img.shape[1] * scale, img.shape[0] * scale),
                                Image.NEAREST).save(p)
    print(f'[preview] {p}')


# ================= (b)(c) 渲染 + preview =================

def test_frames_and_previews():
    args = mkargs(frames=4)
    voxes = list(povstream.ANIMS['palace'](args))
    assert len(voxes) == 4

    # yaw: 帧0 绿树在北 (z 大, x 居中), 帧1 (90°) 甩到 x 负侧
    def green_centroid(vox):
        g = (vox[..., 1] > 200) & (vox[..., 0] < 50) & (vox[..., 2] < 50)
        idx = np.argwhere(g)
        assert len(idx) > 50, 'no green voxels'
        return idx.mean(axis=0) - (GR // 2, GH // 2, GR // 2)
    c0, c1 = green_centroid(voxes[0]), green_centroid(voxes[1])
    assert c0[2] > 20 and abs(c0[0]) < 6, f'frame0 trees not north: {c0}'
    assert c1[0] < -20 and abs(c1[2]) < 6, f'frame1 trees not rotated 90°: {c1}'
    print(f'[b] green centroid f0 (x,z)=({c0[0]:.1f},{c0[2]:.1f}) '
          f'-> f1 ({c1[0]:.1f},{c1[2]:.1f})  (yaw OK)')

    # zoom: 帧2 (zoom 1.15) 占用半径 > 帧0 (zoom 0.75)
    def occ_radius(vox):
        idx = np.argwhere(vox.max(axis=3) > 0).astype(np.float64)
        return float(np.hypot(idx[:, 0] - GR // 2, idx[:, 2] - GR // 2).max())
    r0, r2 = occ_radius(voxes[0]), occ_radius(voxes[2])
    assert r2 > r0 * 1.3, f'zoom not visible: r0 {r0:.1f} r2 {r2:.1f}'
    assert r2 <= gas.R_BUDGET + 1.0, f'frame2 clipped: r {r2:.1f}'
    print(f'[b] occupied radius f0 {r0:.1f} -> f2 {r2:.1f} (zoom OK, in budget)')

    # packed 全管线: 尺寸 + 帧互异
    raws = []
    for raw in povstream.gen_packed_frames(mkargs(frames=4)):
        assert len(raw) == povstream.FRAME_RAW
        raws.append(raw)
    for i in range(4):
        for j in range(i + 1, 4):
            assert raws[i] != raws[j], f'frames {i},{j} identical'
    nz = [sum(b != 0 for b in r[:200000]) for r in raws]
    print(f'[b] 4 packed frames OK, differ, nonzero-head {nz}')

    # 俯视/正视投影: 数值检查 + PNG
    top0 = topdown(voxes[0])
    yellow_top = (top0[:, :, 0] > 200) & (top0[:, :, 1] > 200) & (top0[:, :, 2] < 50)
    lit = top0.max(axis=2) > 0
    cx = GR // 2
    band = np.zeros_like(lit)
    band[:, cx - 15:cx + 15] = True          # 中轴带
    frac = yellow_top[band & lit].mean()
    assert frac > 0.25, f'top view axis band yellow frac {frac:.2f} too low'
    # 中轴白御道可见 (中列附近有白像素; 只在建筑间隙露出, 帧0 zoom 0.75
    # 下御道半宽 ~1.4 voxel, 期望几十像素级)
    white_top = np.all(top0 > 200, axis=2)
    n_path = int(white_top[:, cx - 3:cx + 4].sum())
    assert n_path > 30, f'axis path not visible in top view ({n_path} px)'
    print(f'[c] top view: yellow frac in axis band {frac:.2f}, white path {n_path} px')

    for i in (0, 1):
        save_png(topdown(voxes[i]), f'top_f{i}.png')
        save_png(frontview(voxes[i]), f'front_f{i}.png')
    # packed bin unpack 切片拼图 (验证的是打包字节本身)
    for i in (0, 1):
        bufs = [raws[i][k * pack_obs.SLICE_STRIDE:(k + 1) * pack_obs.SLICE_STRIDE]
                for k in range(povstream.N_SLICES)]
        gas.make_preview(bufs, povstream.N_SLICES,
                         os.path.join(PREVIEW, f'slices_f{i}.png'))
    # 单切片 unpack PNG (0° / 90°)
    for i, k in [(0, 0), (0, 90)]:
        img = pack_obs.unpack_slice(
            raws[i][k * pack_obs.SLICE_STRIDE:(k + 1) * pack_obs.SLICE_STRIDE])
        save_png(img.astype(np.uint8) * 255, f'slice_f{i}_a{k}.png')


if __name__ == '__main__':
    test_pointcloud()
    test_frames_and_previews()
    print('ALL PALACE TESTS PASS')
