#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_glb_anims.py — glbseq / glbanim 动画源 headless 测试 + 回归.

  python3 test_glb_anims.py          # 全部 (~1-2 min, 无网络/无板)

(a) glbseq: 3 个程序化平移立方体 GLB → 3 帧, 帧互异 + 尺寸 = 4,423,680
(b) glbanim: 骨骼手臂 armskin.glb → 6 帧质心运动 + 2 帧全管线 packed;
    另测 morph / 旋转立方体数值
(c) 回归: globe + spinpulse 各 1 帧渲染不坏
"""
import os
import sys
import glob
import shutil
import tempfile
import argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'test_assets'))

# 测试专用 npz 缓存目录 (不污染 stream/pc/cache), 必须在 import povstream 前设
_TMP = tempfile.mkdtemp(prefix='povtest_')
os.environ['POVSTREAM_CACHE'] = os.path.join(_TMP, 'cache')

import povstream                                    # noqa: E402
import glb_anim                                     # noqa: E402
import make_test_glb as mk                          # noqa: E402

ASSETS = os.path.join(HERE, 'test_assets')
FRAME_RAW = povstream.FRAME_RAW                     # 4423680


def mkargs(**kw):
    ap = argparse.ArgumentParser()
    povstream.add_render_opts(ap)
    a = ap.parse_args([])
    a.render_slices = 8                             # 8 × 45 槽复制 = 360
    a.sub = 1
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def ensure_assets():
    for name, fn in [('spincube.glb', mk.make_spincube),
                     ('armskin.glb', mk.make_armskin),
                     ('morphcube.glb', mk.make_morphcube)]:
        p = os.path.join(ASSETS, name)
        if not os.path.exists(p):
            fn(p)


def vox_centroid(vox):
    idx = np.argwhere(vox.sum(axis=3) > 0)
    assert len(idx) > 0, 'empty voxel grid'
    return idx.mean(axis=0)


def test_glbseq():
    d = os.path.join(_TMP, 'seq')
    os.makedirs(d, exist_ok=True)
    for i, (dx, c) in enumerate([(-1.0, (255, 0, 0)), (0.0, (0, 255, 0)),
                                 (1.0, (0, 0, 255))]):
        mk.make_offset_cube(os.path.join(d, f'f{i:03d}.glb'), (dx, 0.0, 0.0), c)
    a = mkargs(anim='glbseq', glb_dir=d, samples=20000)
    frames = list(povstream.gen_packed_frames(a))
    assert len(frames) == 3, f'expected 3 frames, got {len(frames)}'
    for f in frames:
        assert len(f) == FRAME_RAW, f'{len(f)} != {FRAME_RAW}'
    assert frames[0] != frames[1] and frames[1] != frames[2] \
        and frames[0] != frames[2], 'glbseq frames identical'
    # 质心沿 x 单调移动 (offset -1/0/+1)
    a2 = mkargs(anim='glbseq', glb_dir=d, samples=20000)
    cents = [vox_centroid(v)[0] for v in povstream.glbseq_frames(a2)]
    assert cents[0] < cents[1] < cents[2], f'centroid x not monotonic: {cents}'
    print(f'[PASS] glbseq: 3 frames x {FRAME_RAW}B, all differ, '
          f'centroid x {[round(c, 1) for c in cents]}')


def test_glbanim_numerics():
    # skinning + SLERP: 90° 关键帧处臂尖 (0,2)→(-1,1)
    s = glb_anim.AnimSampler(os.path.join(ASSETS, 'armskin.glb'),
                             samples=5000, verbose=False)
    assert abs(s.duration - 1.6) < 1e-5
    p0, _ = s.points_at(0.0)
    p90, _ = s.points_at(0.8)
    assert abs(p0[:, 0]).max() < 0.2 and p0[:, 1].max() > 1.9, 'rest pose wrong'
    assert p90[:, 0].min() < -0.8 and p90[:, 1].max() < 1.4, 'skinned 90° pose wrong'
    # 节点 TRS 旋转 + 纹理: 90° 时立方体点动了但 bbox 不变 (对称)
    c = glb_anim.AnimSampler(os.path.join(ASSETS, 'spincube.glb'),
                             samples=5000, verbose=False)
    q0, col = c.points_at(0.0)
    q1, _ = c.points_at(0.375)
    assert np.abs(q0 - q1).max() > 0.3, 'cube did not rotate'
    assert np.allclose(q0.min(0), q1.min(0), atol=0.05), 'cube bbox drifted'
    assert col.std(axis=0).max() > 10, 'texture sampling flat (checker expected)'
    # morph: weights 0→1 顶面 +1
    m = glb_anim.AnimSampler(os.path.join(ASSETS, 'morphcube.glb'),
                             samples=5000, verbose=False)
    m0, _ = m.points_at(0.0)
    m1, _ = m.points_at(0.6)
    assert m1[:, 1].max() > m0[:, 1].max() + 0.7, 'morph target not applied'
    print('[PASS] glbanim numerics: skinning/SLERP + node TRS + texture + morph')


def test_glbanim_pipeline():
    arm = os.path.join(ASSETS, 'armskin.glb')
    # 6 帧体素质心运动
    a = mkargs(anim='glbanim', glb=arm, anim_take='0', frames=6, samples=20000)
    cents = [vox_centroid(v) for v in povstream.glbanim_frames(a)]
    disp = max(np.linalg.norm(c1 - c2) for i, c1 in enumerate(cents)
               for c2 in cents[i + 1:])
    assert disp > 3.0, f'centroid barely moved: max disp {disp:.2f} voxels'
    # take 按名字选
    a_named = mkargs(anim='glbanim', glb=arm, anim_take='wave', frames=2, samples=5000)
    assert len(list(povstream.glbanim_frames(a_named))) == 2
    # 全管线 packed
    a2 = mkargs(anim='glbanim', glb=arm, anim_take='0', frames=2, samples=20000)
    frames = list(povstream.gen_packed_frames(a2))
    assert len(frames) == 2 and all(len(f) == FRAME_RAW for f in frames)
    assert frames[0] != frames[1], 'glbanim packed frames identical'
    print(f'[PASS] glbanim pipeline: 6 帧质心位移 {disp:.1f} voxel, '
          f'2 packed frames x {FRAME_RAW}B differ')


def test_regression():
    a = mkargs(anim='globe', frames=1)
    f = list(povstream.gen_packed_frames(a))
    assert len(f) == 1 and len(f[0]) == FRAME_RAW
    print('[PASS] regression globe: 1 frame ok')
    # spinpulse 用小测试立方体 GLB (避免 anime 1.8M 采样, 只验路径)
    a = mkargs(anim='spinpulse', frames=1,
               glb=os.path.join(ASSETS, 'spincube.glb'), samples=20000)
    f = list(povstream.gen_packed_frames(a))
    assert len(f) == 1 and len(f[0]) == FRAME_RAW
    print('[PASS] regression spinpulse: 1 frame ok')


def main():
    ensure_assets()
    try:
        test_glbseq()
        test_glbanim_numerics()
        test_glbanim_pipeline()
        test_regression()
    finally:
        shutil.rmtree(_TMP, ignore_errors=True)
    print('ALL TESTS PASS')


if __name__ == '__main__':
    main()
