#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_projection.py — GLB → 2D 正交投影 → 屏 fb (11664B) + 预览图。

用途: POV 切片静止时只是薄薄一层截面 (一堆散点, 认不出物体), 电机不转时没法看。
本工具把体素沿深度方向做最大值投影压成平面图, **静止就能认出物体**, 用于:
  - 电机不转时的静态展示 / 演示
  - 上电开机画面 (FSBL 或 BRAM 初值路线)
  - 内容正确性的快速肉眼检查 (比逐片翻切片快得多)

与 gen_anime_slices.py 的区别: 那个出 360 片 POV 切片 (要旋转成像),
本工具出**单张** 2D 图, 直接灌 fb 即可显示。

用法 (Windows python):
  python gen_projection.py --glb "D:\\...\\Robot.glb" --out robot_fb.bin
      [--thresh 110] [--samples 1800000] [--axis z] [--preview x.png]
"""
import os
import sys
import math
import argparse

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_anime_slices as G
from pack_obs import pack_slice, SLICE_DATA


def _project_at(vox, theta):
    """绕竖轴旋转 theta 后沿视线做最大值投影 → [横, 纵, 3]。

    做法: 对屏上每个横坐标 u, 沿视线方向取一整条体素线求 max。
    视线单位向量 d=(sin θ, cos θ), 屏横向单位向量 r=(cos θ, -sin θ)。
    世界点 = u·r + t·d, t 扫过整个深度范围。
    """
    GR, GH = vox.shape[0], vox.shape[1]
    c, s = math.cos(theta), math.sin(theta)
    half = GR // 2
    u = np.arange(GR, dtype=np.float32) - half          # 屏横向
    t = np.arange(GR, dtype=np.float32) - half          # 深度
    # 网格: (u, t) → 体素 x/z 下标
    ux = u[:, None] * c + t[None, :] * s
    uz = -u[:, None] * s + t[None, :] * c
    xi = np.clip(np.rint(ux).astype(np.int32) + half, 0, GR - 1)
    zi = np.clip(np.rint(uz).astype(np.int32) + half, 0, GR - 1)
    # vox[xi, :, zi] → (GR_u, GR_t, GH, 3), 沿深度 t 取 max
    return vox[xi, :, zi].max(axis=1)                   # → (GR_u, GH, 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glb', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--preview', default=None)
    ap.add_argument('--thresh', type=float, default=110,
                    help='1-bit 阈值; 比切片默认 128 低一些, 让投影更饱满')
    ap.add_argument('--samples', type=int, default=1800000)
    ap.add_argument('--axis', choices=['x', 'z'], default='z',
                    help='沿哪个水平轴投影 (换个侧面看); --frames>1 时忽略')
    ap.add_argument('--frames', type=int, default=1,
                    help='>1 = 生成绕竖轴旋转一周的 N 帧动画, 输出 N*11664B 连续帧序列')
    ap.add_argument('--no-dither', action='store_true')
    ap.add_argument('--z-stretch', type=float, default=1.0)
    args = ap.parse_args()

    xyz, col = G.points_from_glb(args.glb, args.samples, 'lambert', 0.7)
    col = G.color_adjust(col)
    vox = G.voxelize(xyz, col, args.z_stretch)      # (GR, GH, GR, 3) = [x, y, z]

    frames = []
    if args.frames <= 1:
        angles = [None]                                 # 单帧: 用 --axis 指定的固定视角
    else:
        angles = [2.0 * math.pi * k / args.frames for k in range(args.frames)]

    for k, th in enumerate(angles):
        if th is None:
            proj = vox.max(axis=2 if args.axis == 'z' else 0)
            if args.axis == 'x':
                proj = proj.transpose(1, 0, 2)
        else:
            # 绕竖轴转 th: 视线方向 (sin th, cos th) —— 沿该方向做最大值投影。
            # 用采样重投影而非旋转整个体素格 (后者内存与耗时都不划算)。
            proj = _project_at(vox, th)
        img = proj.transpose(1, 0, 2)[::-1]              # (H, W, 3)
        on = G.to_1bit(img, args.thresh, not args.no_dither, k)
        frames.append(bytes(pack_slice(on)))
        if k == 0 or args.frames <= 1:
            lit = int(on.any(axis=2).sum())
            print('[proj] 投影 %s  1-bit 亮像素 %d (%.0f%% 全屏)'
                  % (img.shape, lit, 100.0 * lit / (160 * 180)), flush=True)

    with open(args.out, 'wb') as f:
        for b in frames:
            assert len(b) == SLICE_DATA, len(b)
            f.write(b)
    print('[out] %s  %d 帧 x %d B = %d B'
          % (args.out, len(frames), SLICE_DATA, len(frames) * SLICE_DATA), flush=True)

    if args.preview:
        Image.fromarray((on * 255).astype('uint8')).resize(
            (160 * 3, 180 * 3), Image.NEAREST).save(args.preview)
        print('[preview] %s' % args.preview, flush=True)


if __name__ == '__main__':
    main()
