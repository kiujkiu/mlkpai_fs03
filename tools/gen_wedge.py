#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_wedge.py — 8 级灰度楔测试图 (3-bit 色深上板目视用, 05_3bit_bcm.md §7.3).

判据 (上板肉眼看):
  · 8 阶必须**单调**: 从码 0 到码 7 一路变亮, 不许中间有一档反而变暗
    (反了 = plane 权重接错/位序反了, 27/54/108 三个 OE 权重与 plane0/1/2 对不上)
  · 不许**跳阶**: 相邻两档亮度差应当逐档变小 (线性光 1/7 步进 + 人眼 ~2.2 次方
    ⇒ 0→1 那一步天生最大, 6→7 最小); 若某一档"跳"得离谱, 多半是那个 plane
    的 OE 窗口不对 (权重不是 1:2:4) 或 LWAIT 插进来了
  · 三色分开看: R/G/B 各自的 8 阶都要单调 —— 单色不单调 = 那条 lane 的位平面串了

输出 (默认 = 一个可以直接推流的预渲染目录):
  <out-dir>/frame_0000.bin  n_slices 片, 每片同一张图 (旋转起来 = 绕轴的回转体)
  <out-dir>/meta.json       bpp / n_slices / geom_flags, povstream stream --dir 认这个
  可选 --png                预览图 (需要 PIL), 按 sRGB 反 gamma 显示, 近似肉眼所见

用法:
  python3 tools/gen_wedge.py --bpp 3 --n-slices 60 \\
      --out-dir stream/pc/frames_wedge3 --png /tmp/wedge.png
  python3 stream/pc/povstream.py stream --dir stream/pc/frames_wedge3 \\
      --n-slices 60 --host <board> --fps 5 --loop
  # 1-bit 对照组 (同样的目标灰度, 走 Bayer 抖动) —— A/B 看 3-bit 到底赢在哪:
  python3 tools/gen_wedge.py --bpp 1 --n-slices 360 --out-dir stream/pc/frames_wedge1

⚠ 图案与观察者视角的关系 (选 --pattern 时要想清楚):
  · flat  (默认): 8 档沿 **Y (屏高)**, 4 条色带沿 X。屏**不转**、直接看面板时
                  这就是所见即所得的 8×4 方格。
  · rings: 同样 8 档沿 Y, 但 X 方向**关于中心列左右对称**。屏转起来时观察者列 X
           与 160-1-X 落在同一半径上, 不对称的图案会互相叠加成一团;
           对称排布转出来就是干净的同心色环 —— **转起来看用这个**。
  · ramp:  连续渐变 (线性光) 走 to_3bit 的残差抖动, 用来看抖动够不够顺、
           有没有 Bayer 网纹。需要 gen_anime_slices (PIL)。
"""
import os
import sys
import json
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pack_obs
from pack_obs import W, H

LEVELS = 8                      # 码值 0..7
DEFAULT_GAMMA = 2.2             # = gen_anime_slices.LED_GAMMA (只影响 1-bit 对照组与预览)
BANDS = [('W', (1, 1, 1)), ('R', (1, 0, 0)), ('G', (0, 1, 0)), ('B', (0, 0, 1))]


def _band_of_y(Y):
    """屏高 Y → 档位 0..7 (**上亮下暗**: Y=0 是码 7)。"""
    return (LEVELS - 1) - min(Y * LEVELS // H, LEVELS - 1)


def build_codes(pattern):
    """→ (H,W,3) uint8 码值图 0..7。"""
    code = np.zeros((H, W, 3), np.uint8)
    lvl = np.array([_band_of_y(y) for y in range(H)], np.uint8)      # (H,)
    if pattern == 'flat':
        for i, (_, mask) in enumerate(BANDS):
            x0, x1 = i * W // len(BANDS), (i + 1) * W // len(BANDS)
            for c in range(3):
                if mask[c]:
                    code[:, x0:x1, c] = lvl[:, None]
    elif pattern == 'rings':
        # 左右对称: 半宽 80 分 4 段, 由中心向外 W/R/G/B, 另一半镜像
        half = W // 2
        for i, (_, mask) in enumerate(BANDS):
            r0, r1 = i * half // len(BANDS), (i + 1) * half // len(BANDS)
            for c in range(3):
                if mask[c]:
                    code[:, half + r0:half + r1, c] = lvl[:, None]      # 右半
                    code[:, half - r1:half - r0, c] = lvl[:, None]      # 左半 (镜像)
    else:
        raise ValueError(pattern)
    return code


def build_ramp_codes(gamma, phase):
    """连续线性渐变 (码值域 0..7 连续) → to_3bit 残差抖动后的码值图。"""
    import gen_anime_slices as gas
    # 目标线性光沿 Y 从 0 到 1 连续; 反 gamma 回输入域再交给 to_3bit
    lin = np.linspace(1.0, 0.0, H, dtype=np.float32)                 # 上亮下暗
    v = 255.0 * np.power(lin, 1.0 / gamma)
    img = np.repeat(np.repeat(v[:, None, None], W, 1), 3, 2)
    for i, (_, mask) in enumerate(BANDS):                            # 分色带
        x0, x1 = i * W // len(BANDS), (i + 1) * W // len(BANDS)
        for c in range(3):
            if not mask[c]:
                img[:, x0:x1, c] = 0.0
    return gas.to_3bit(img, 128, True, phase, gamma=gamma)


def codes_to_input(code, gamma):
    """码值 0..7 → 输入域 (sRGB) 0..255: v = 255·(code/7)^(1/gamma)。
    1-bit 对照组和 PNG 预览都用它 —— 保证两条路的**目标线性光完全一样**。"""
    return 255.0 * np.power(code.astype(np.float32) / (LEVELS - 1), 1.0 / gamma)


def pack_frame(code, bpp, n_slices, gamma, phase0=0):
    """码值图 → 一帧 (n_slices 片, 每片同一张图)。

    bpp=3: 直接打包码值。
    bpp=1: 先把码值折回输入域, 再走与 povstream 完全同一套 Bayer 抖动
           (相位逐槽变) —— 这样 1-bit 对照组和 3-bit 是同一个目标亮度。"""
    parts = []
    if bpp == 3:
        one = pack_obs.pack_slice(code, bpp=3)
        return one * n_slices
    import gen_anime_slices as gas
    img = codes_to_input(code, gamma)
    for s in range(n_slices):
        on = gas.to_1bit(img, 128, True, phase0 + s)
        parts.append(pack_obs.pack_slice(on, bpp=1, pad=True))
    return b''.join(parts)


def save_png(code, path, gamma):
    from PIL import Image
    px = np.clip(codes_to_input(code, gamma), 0, 255).astype(np.uint8)
    Image.fromarray(px).resize((W * 3, H * 3), Image.NEAREST).save(path)


def main():
    ap = argparse.ArgumentParser(description='8 级灰度楔测试图 (3-bit 上板目视)')
    ap.add_argument('--bpp', type=int, choices=sorted(pack_obs.BPP_MODES), default=3)
    ap.add_argument('--pattern', choices=['flat', 'rings', 'ramp'], default='flat')
    ap.add_argument('--n-slices', type=int, default=60,
                    help='帧里的片数 (默认 60 = 3-bit 方案推荐的每面槽数)')
    ap.add_argument('--gamma', type=float, default=DEFAULT_GAMMA,
                    help=f'反 gamma (默认 {DEFAULT_GAMMA}), 只影响 1-bit 对照组/预览/ramp; '
                         'flat/rings 的 3-bit 码值是**直接写死的 0..7**, 不过 gamma')
    ap.add_argument('--out-dir', default=None,
                    help='预渲染目录 (默认 stream/pc/frames_wedge<bpp>)')
    ap.add_argument('--png', default=None, metavar='PATH', help='另存预览 PNG (需 PIL)')
    a = ap.parse_args()

    stride = pack_obs.slice_stride(a.bpp)
    if not 1 <= a.n_slices <= 8847360 // stride:
        sys.exit(f'--n-slices 须在 1..{8847360 // stride} (bpp={a.bpp} 片距 0x{stride:X})')

    code = (build_ramp_codes(a.gamma, 0) if a.pattern == 'ramp'
            else build_codes(a.pattern))
    raw = pack_frame(code, a.bpp, a.n_slices, a.gamma)
    assert len(raw) == a.n_slices * stride, (len(raw), a.n_slices * stride)

    out = a.out_dir or os.path.join(HERE, '..', 'stream', 'pc', f'frames_wedge{a.bpp}')
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'frame_0000.bin'), 'wb') as f:
        f.write(raw)
    # meta.json: povstream stream --dir 靠 bpp 推片距, 靠 geom_flags 推面拆分
    meta = {'anim': f'wedge-{a.pattern}', 'frames': 1, 'render_slices': a.n_slices,
            'frame_raw': len(raw), 'freeze_phase': True,
            'bpp': a.bpp, 'led_gamma': a.gamma if a.bpp == 3 else None,
            'n_slices': a.n_slices, 'geom_flags': 0,
            'faces': [{'name': 'A', 'axis_off_px': 0.0, 'n_slices': a.n_slices}],
            'note': 'gen_wedge.py 生成的 8 级灰度楔 (单面, 每片同图)'}
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    if a.png:
        save_png(code, a.png, a.gamma)

    used = sorted(set(int(v) for v in np.unique(code)))
    print(f'[wedge] {a.pattern} bpp={a.bpp} 码值 {used} → {out}')
    print(f'[wedge] {a.n_slices} 片 × 0x{stride:X} = {len(raw)}B'
          + (f', 预览 {a.png}' if a.png else ''))
    print(f'[wedge] 推流: python3 stream/pc/povstream.py stream --dir {out} '
          f'--n-slices {a.n_slices}' + (f' --bpp {a.bpp}' if a.bpp != 1 else '')
          + ' --host <board> --fps 5 --loop')


if __name__ == '__main__':
    main()
