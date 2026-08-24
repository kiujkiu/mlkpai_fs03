#!/usr/bin/env python3
"""把一帧 PVS 数据模拟成"转起来之后人眼看到的样子"，输出 PNG。

原理: POV 的每个切片是一个**过旋转轴的平面**, 观察者看到的是所有角度的切片
在视网膜上的叠加。这里对每片按它的真实角度做 3D 旋转再正交投影, 取 max 叠加
—— 等价于视觉暂留。

⚠ 它反映的是**实际发到板子的字节**(量化/抖动/半屏压缩都在里面), 所以可以用来
判断"颗粒感是 3-bit 固有的还是实现引入的": 如果这张图也有同样的颗粒, 那就是
内容侧的量化/抖动; 如果这张干净而屏上有颗粒, 那是显示链路的问题。

用法: python3 tools/pov_preview.py <frames_dir> [out.png] [--view 正交视角度数]
"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import pack_obs
from PIL import Image


def main():
    d = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/pov_preview.png'
    view = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
    meta = json.load(open(os.path.join(d, 'meta.json'), encoding='utf-8'))
    bpp = meta.get('bpp', 1)
    st = pack_obs.slice_stride(bpp)
    raw = open(os.path.join(d, 'frame_0000.bin'), 'rb').read()
    n = len(raw) // st
    faces = meta.get('faces') or [{'n_slices': n}]
    nA = faces[0]['n_slices']
    H, W = pack_obs.H, pack_obs.W
    acc = np.zeros((H, W, 3), np.float32)          # 观察者视角画布
    cx = W / 2.0
    for i in range(nA):                            # 只画面A (穿心面) 就够看形状
        img = pack_obs.unpack_slice(raw[i * st:(i + 1) * st], bpp=bpp).astype(np.float32)
        if not img.any():
            continue
        th = 2 * math.pi * i / nA + view
        ys, xs = np.nonzero(img.any(axis=2))
        u = xs - cx                                # 屏内横坐标 = 有符号半径
        # 绕 Y 轴转 th 后正交投影到观察者 (看 XY 平面)
        px = np.rint(u * math.cos(th) + cx).astype(np.int32)
        keep = (px >= 0) & (px < W)
        px, py, ys2, xs2 = px[keep], ys[keep], ys[keep], xs[keep]
        np.maximum.at(acc, (py, px), img[ys2, xs2])
    mx = acc.max()
    if mx > 0:
        acc = acc / mx * 255.0
    Image.fromarray(acc.astype(np.uint8)).save(out)
    print(f'{d}: {n} 片 (面A {nA}), bpp={bpp} -> {out}')


main()
