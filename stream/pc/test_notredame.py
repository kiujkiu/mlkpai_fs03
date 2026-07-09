#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_notredame.py — notredame 动画源测试 + 正交投影 preview 生成 (无板无网络).

  python3 test_notredame.py         # 全部, 顺便产出 preview_notredame/*.png

(a) 点云: 总量 300k-800k / 颜色全是 1-bit 纯色 / |mean_x|<2 / 双塔 =
    南端两个 x 对称高 y 簇 (塔间上空留空) / 尖塔 = 全模型最高点且在
    十字交叉 z 附近 (黄尖) / 玫瑰窗蓝点贴南立面平面 / 绿点最高处成脊线
    (沿 z 的中殿脊 + 沿 x 的翼脊) / 预缩放 zoom 最大时不出预算
(b) 4 帧渲染: packed 尺寸 = 4,423,680 / 帧互异 / yaw 90° 把南立面玫瑰窗
    蓝簇 (z 不对称特征) 甩到 x 侧 / zoom 可见且不出预算
(c) preview: 俯视/正视(南立面)/侧视 PNG + packed bin unpack 切片拼图 x2 +
    单切片 unpack, 存 preview_notredame/ 供人眼复核
"""
import os
import sys
import argparse
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import povstream                                    # noqa: E402
import notredame                                    # noqa: E402
import palace                                       # noqa: E402
import pack_obs                                     # noqa: E402
import gen_anime_slices as gas                      # noqa: E402

PREVIEW = os.path.join(HERE, 'preview_notredame')
ALLOWED = {(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
           (0, 255, 255), (255, 0, 255), (255, 255, 255)}
GR, GH = gas.GR, gas.GH


def mkargs(**kw):
    ap = argparse.ArgumentParser()
    povstream.add_render_opts(ap)
    a = ap.parse_args([])
    a.anim = 'notredame'
    a.render_slices = 8
    a.sub = 1
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def mask(col, rgb):
    return np.all(col.astype(np.int32) == np.asarray(rgb, np.int32), axis=1)


# ================= (a) 点云结构 =================

def test_pointcloud():
    xyz, col = notredame.build_notredame()
    n = len(xyz)
    assert 300_000 <= n <= 800_000, f'total {n} out of 300k-800k'
    stats = palace.color_stats(col)
    assert set(stats) <= ALLOWED, f'non-pure colors: {set(stats) - ALLOWED}'
    print('[a] colors:', {k: v for k, v in sorted(stats.items(), key=lambda kv: -kv[1])})

    whi, grn = mask(col, palace.WHITE), mask(col, palace.GREEN)
    blu, red, yel = (mask(col, notredame.BLUE), mask(col, palace.RED),
                     mask(col, palace.YELLOW))
    assert whi.sum() > 100_000 and grn.sum() > 10_000, 'stone/roof too sparse'
    assert blu.sum() > 2000 and red.sum() > 500 and yel.sum() > 100, \
        'rose/tip too sparse'

    # x 对称
    for name, m in [('all', slice(None)), ('white', whi), ('green', grn)]:
        mx = float(xyz[m, 0].mean())
        assert abs(mx) < 2.0, f'{name} mean_x {mx:.2f} not centered'

    # 双塔: 南端 (z<-45) 高 y 带 (y 5..26, 立面顶 -11.5 以上) 两个 x 对称簇
    band = (xyz[:, 1] > 5.0) & (xyz[:, 1] < 26.0) & (xyz[:, 2] < -45.0)
    east, west = band & (xyz[:, 0] > 3), band & (xyz[:, 0] < -3)
    gap = band & (np.abs(xyz[:, 0]) <= 3)
    ne, nw = int(east.sum()), int(west.sum())
    assert ne > 3000 and nw > 3000, f'tower clusters too sparse {ne}/{nw}'
    assert abs(ne - nw) < 0.1 * max(ne, nw), f'towers unbalanced {ne}/{nw}'
    xe, xw = float(xyz[east, 0].mean()), float(xyz[west, 0].mean())
    assert abs(xe + xw) < 1.0 and 10 < xe < 16, f'tower x centroids {xw:.1f}/{xe:.1f}'
    assert gap.sum() < 0.02 * ne, f'tower gap not open ({int(gap.sum())} pts)'
    print(f'[a] twin towers: {nw}/{ne} pts at x {xw:.1f}/{xe:.1f}, gap {int(gap.sum())}')

    # 尖塔 = 最高点, 在十字交叉 (x≈0, z≈6), 顶为黄
    top = int(np.argmax(xyz[:, 1]))
    tx, ty, tz = (float(v) for v in xyz[top])
    assert abs(tx) < 2.0 and abs(tz - 6.0) < 4.0, f'spire apex at ({tx:.1f},{tz:.1f})'
    assert yel[top], 'spire apex not yellow tip'
    assert ty > xyz[:, 1].max() - 0.5
    # 尖塔明显高过双塔
    assert ty > 25.0 + 30.0, 'spire not dominant'
    print(f'[a] spire apex ({tx:.1f},{ty:.1f},{tz:.1f}) yellow OK')

    # 玫瑰窗: 南立面蓝点贴 z=-70.6 平面, 居中, 心高 ~-31.5
    rose = blu & (xyz[:, 2] < -60.0)
    assert rose.sum() > 1500, f'facade rose too sparse ({int(rose.sum())})'
    assert float(xyz[rose, 2].std()) < 1.0, 'rose not planar'
    assert abs(float(xyz[rose, 0].mean())) < 1.0, 'rose not centered'
    ry = float(xyz[rose, 1].mean())
    assert abs(ry - (-31.5)) < 2.5, f'rose center y {ry:.1f}'
    # 翼端小玫瑰: 两侧 |x|>30 也有蓝
    assert (blu & (xyz[:, 0] > 30)).sum() > 400 and \
           (blu & (xyz[:, 0] < -30)).sum() > 400, 'transept roses missing'
    print(f'[a] rose windows: facade {int(rose.sum())} blue pts @ y {ry:.1f}, '
          f'transept OK')

    # 绿脊线: 绿点最高 1.5 带内全部在 中殿脊 (|x|<3) 或 翼脊 (|z-6|<3)
    gy = xyz[grn, 1]
    ridge = grn & (xyz[:, 1] > float(gy.max()) - 1.5)
    on_ridge = (np.abs(xyz[ridge, 0]) < 3.0) | (np.abs(xyz[ridge, 2] - 6.0) < 3.0)
    assert on_ridge.mean() > 0.98, f'ridge stray {1 - on_ridge.mean():.3f}'
    nave_r = ridge & (np.abs(xyz[:, 0]) < 3.0)
    assert xyz[nave_r, 2].min() < -35 and xyz[nave_r, 2].max() > 30, \
        'nave ridge does not span body'
    print(f'[a] green ridge: max y {gy.max():.1f}, span z '
          f'{xyz[nave_r, 2].min():.1f}..{xyz[nave_r, 2].max():.1f}')

    # 预缩放 clip-safe
    p0, _ = povstream.notredame_points_prescaled()
    r = float(np.hypot(p0[:, 0], p0[:, 2]).max()) * povstream.ORBIT_ZOOM_MAX
    h = float(np.abs(p0[:, 1]).max()) * povstream.ORBIT_ZOOM_MAX
    assert r <= gas.R_BUDGET and h <= gas.H_BUDGET, f'clip-unsafe r {r:.1f} h {h:.1f}'
    print(f'[a] OK: {n} pts, prescaled max-zoom r {r:.1f}/{gas.R_BUDGET} '
          f'h {h:.1f}/{gas.H_BUDGET}')


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

def sideview(vox):
    """东面侧视 RGB (行=y 上在上, 列=z 北在右): 每 (z,y) 取 x 最大占用体素."""
    occ = vox.max(axis=3) > 0
    xs = np.where(occ, np.arange(GR)[:, None, None], -1).max(axis=0)
    img = np.zeros((GH, GR, 3), np.uint8)
    yi, zi = np.nonzero(xs >= 0)
    img[yi, zi] = vox[xs[yi, zi], yi, zi]
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
    voxes = list(povstream.ANIMS['notredame'](args))
    assert len(voxes) == 4

    # yaw: 帧0 南立面玫瑰蓝簇在南 (z<0 显著, x 居中), 帧1 (90°) 甩到 x 正侧
    def blue_centroid(vox):
        b = (vox[..., 2] > 150) & (vox[..., 0] < 100) & (vox[..., 1] < 100)
        idx = np.argwhere(b)
        assert len(idx) > 50, f'only {len(idx)} blue voxels'
        return idx.mean(axis=0) - (GR // 2, GH // 2, GR // 2)
    c0, c1 = blue_centroid(voxes[0]), blue_centroid(voxes[1])
    assert c0[2] < -15 and abs(c0[0]) < 4, f'frame0 rose not south: {c0}'
    assert c1[0] > 15 and abs(c1[2]) < 6, f'frame1 rose not rotated 90°: {c1}'
    print(f'[b] blue centroid f0 (x,z)=({c0[0]:.1f},{c0[2]:.1f}) '
          f'-> f1 ({c1[0]:.1f},{c1[2]:.1f})  (yaw OK)')

    # 尖塔黄尖: 帧0 最高占用体素接近 y 预算, 黄尖存在
    occ0 = voxes[0].max(axis=3) > 0
    ymax0 = int(np.argwhere(occ0)[:, 1].max())
    assert ymax0 - GH // 2 > 40, f'model not tall (ymax {ymax0})'
    yel0 = (voxes[0][..., 0] > 150) & (voxes[0][..., 1] > 150) & (voxes[0][..., 2] < 100)
    yi = np.argwhere(yel0)
    assert len(yi) > 10 and yi[:, 1].max() >= ymax0 - 1, 'yellow tip not topmost'
    print(f'[b] verticality: top occupied y {ymax0 - GH // 2:+d} '
          f'(budget ±{int(gas.H_BUDGET)}), yellow tip on top OK')

    # zoom: 帧2 (zoom 1.15) 占用半径 > 帧0 (zoom 0.75), 且不出预算
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
    print('[b] 4 packed frames OK, differ')

    # 正视图数值检查: 玫瑰窗蓝簇居中可见 + 尖塔白列高耸
    fr0 = frontview(voxes[0])
    bl = (fr0[:, :, 2] > 150) & (fr0[:, :, 0] < 100) & (fr0[:, :, 1] < 100)
    ys, xs = np.nonzero(bl)
    assert len(ys) > 40, f'front view rose only {len(ys)} px'
    assert abs(float(xs.mean()) - GR // 2) < 5, 'front rose not centered'
    print(f'[c] front view rose: {len(ys)} blue px centered OK')

    # 人眼 PNG: 俯视 / 正视 (南立面) / 侧视
    save_png(topdown(voxes[0]), 'top_f0.png')
    save_png(topdown(voxes[1]), 'top_f1.png')
    save_png(frontview(voxes[0]), 'front_f0.png')
    save_png(frontview(voxes[1]), 'front_f1.png')
    save_png(sideview(voxes[0]), 'side_f0.png')
    # packed bin unpack 切片拼图 x2 (验证打包字节本身)
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


def test_globe_regression():
    """globe 源冒烟 (orbit 重构不应影响其它源): 首帧体素有效且纯色."""
    args = mkargs(frames=2)
    args.anim = 'globe'
    vox = next(iter(povstream.ANIMS['globe'](args)))
    occ = vox.max(axis=3) > 0
    assert occ.sum() > 20_000, f'globe too sparse ({int(occ.sum())})'
    cols = vox[occ]
    assert cols[:, 1].max() > 200 and cols[:, 2].max() > 200, 'globe colors off'
    print(f'[reg] globe frame OK: {int(occ.sum())} voxels')


if __name__ == '__main__':
    test_pointcloud()
    test_frames_and_previews()
    test_globe_regression()
    print('ALL NOTREDAME TESTS PASS')
