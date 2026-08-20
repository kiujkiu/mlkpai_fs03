#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_default_3bit.py — 生成 3-bit 冷启动默认内容 anime_dual3b100.bin (2026-08-20).

背景: pov_boot.sh 的 BPP3=1 分支把 DEFAULT_BIN **原样 memcpy 进 bank A**
(不解压、不走协议), 所以这份 .bin 必须是「解压后的载荷」本身:
    [面A 50 片][面B 50 片] × 片距 0x9000 = 100 × 36864 = 3,686,400 B
面拆分点 50*0x9000 = 0x1C2000 必须与 pov_boot.sh 写进 0x28 的
slice_base_b = BANK_A + N_SLICES*STRIDE 逐字节对齐 —— 对不上就是屏B 显示错位。

为什么是新文件而不是改 gen_anime_slices.py:
    gen_anime_slices.main() 的 CLI 目前只出 1-bit (--bpp 还没接上, 见那边
    to_3bit() 已经就位但没人调)。本文件是**薄驱动**: 几何/采样/体素化/
    render_slice/to_3bit 全部直接 import 它的函数, 一行也没复制。
    等 gen_anime_slices 接上 `--bpp 3 --dual-face --slices 50` 之后,
    本文件就可以删掉, 换成:
        python gen_anime_slices.py --bpp 3 --dual-face --slices 50 --out ...

⚠ 渲染依赖 pygltflib + PIL, WSL 的 python3 没有。用 Windows 那个:
    C:\\Users\\<u>\\AppData\\Local\\Programs\\Python\\Python312\\python.exe \\
        D:\\...\\tools\\gen_default_3bit.py --out D:\\...\\stream\\board\\anime_dual3b100.bin

用法:
    python gen_default_3bit.py [--glb PATH] [--slices 50] [--out PATH]
        [--frames-dir DIR]   # 顺手出一份 frame_0000.bin + meta.json,
                             # 给 make_idle_anim.py / povstream --dir 用
        [--png PATH]         # 预览 (按 sRGB 反 gamma 近似肉眼所见)
"""
import os
import sys
import json
import math
import time
import argparse

import numpy as np

# Windows 控制台默认 GBK, gen_anime_slices 的日志里有 ⇒/★/≡ 之类 → UnicodeEncodeError
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pack_obs
import gen_anime_slices as gas
from pack_obs import W, H

STRIDE_3BIT = pack_obs.SLICE_STRIDE_3BIT          # 0x9000
FRAME_RAW_MAX = 8847360                           # 一个 DDR bank


def render_face(vox, axis_off, n_slices, sub, thresh, dither, mirror_u, gamma, tag):
    """一个面 → [n_slices 片 3-bit 打包字节], 顺带返回码值直方图。"""
    d_step = 2 * math.pi / n_slices
    bufs, hist = [], np.zeros(8, np.int64)
    for i in range(n_slices):
        img = gas.render_slice(vox, i * d_step, sub, d_step, axis_off, mirror_u)
        code = gas.to_3bit(img, thresh, dither, i, gamma)
        hist += np.bincount(code.ravel(), minlength=8)
        buf = pack_obs.pack_slice(code, bpp=3)
        assert len(buf) == STRIDE_3BIT, len(buf)
        bufs.append(buf)
        if i % 10 == 0:
            print(f'  {tag} slice {i}: {int((code > 0).any(axis=2).sum())} lit px',
                  flush=True)
    return bufs, hist


def save_png(bufs, path, n_slices, gamma, scale=3, count=8, cols=4):
    """从**打包字节**反解回观察者视角 (验的是 bin 本身, 不是内存里的 code)。"""
    from PIL import Image
    rows = (count + cols - 1) // cols
    pad = 2
    tw, th = W * scale + pad, H * scale + pad
    canvas = Image.new('RGB', (cols * tw + pad, rows * th + pad), (24, 24, 24))
    for k in range(count):
        i = k * len(bufs) // count
        code = pack_obs.unpack_slice(bufs[i], bpp=3).astype(np.float32) / 7.0
        srgb = np.clip(np.power(code, 1.0 / gamma) * 255.0, 0, 255).astype(np.uint8)
        tile = Image.fromarray(srgb).resize((W * scale, H * scale), Image.NEAREST)
        gas.draw_num(tile.load(), 3, 3, str(i * 360 // n_slices), (255, 255, 0))
        canvas.paste(tile, (pad + (k % cols) * tw, pad + (k // cols) * th))
    canvas.save(path)
    print(f'[preview] {path}', flush=True)


def main():
    ap = argparse.ArgumentParser(description='3-bit 双面冷启动默认内容')
    ap.add_argument('--glb', default=gas.DEFAULT_GLB)
    ap.add_argument('--points', default=None, help='PovPoint .bin 备胎 (跳过 GLB)')
    ap.add_argument('--slices', type=int, default=50,
                    help='**每面**槽数 (= pov_boot.sh 的 N_SLICES, 3-bit 实测每圈 53 个角度 → 50)')
    ap.add_argument('--sub', type=int, default=3)
    ap.add_argument('--samples', type=int, default=1800000)
    ap.add_argument('--thresh', type=float, default=128)
    ap.add_argument('--no-dither', action='store_true')
    ap.add_argument('--gamma-led', type=float, default=gas.LED_GAMMA,
                    help='码值→线性光的解码 gamma (to_3bit)')
    ap.add_argument('--gap-mm', type=float, default=13.8)
    ap.add_argument('--brighten', type=float, default=1.5)
    ap.add_argument('--gamma', type=float, default=0.9)
    ap.add_argument('--saturation', type=float, default=2.0)
    ap.add_argument('--lighting', default='lambert')
    ap.add_argument('--ambient', type=float, default=0.7)
    ap.add_argument('--z-stretch', type=float, default=1.0)
    ap.add_argument('--no-mirror-u', dest='mirror_u', action='store_false', default=True)
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'stream', 'board',
                                                  'anime_dual3b100.bin'))
    ap.add_argument('--frames-dir', default=None,
                    help='另存 frame_0000.bin + meta.json (make_idle_anim.py 的输入)')
    ap.add_argument('--png', default=None)
    a = ap.parse_args()

    n = a.slices
    total = 2 * n
    if total * STRIDE_3BIT > FRAME_RAW_MAX:
        sys.exit(f'{total} 片 × 0x{STRIDE_3BIT:X} = {total * STRIDE_3BIT}B '
                 f'> bank 上限 {FRAME_RAW_MAX}B')

    if a.points:
        xyz, col = gas.points_from_bin(a.points)
    else:
        xyz, col = gas.points_from_glb(a.glb, a.samples, a.lighting, a.ambient)
    print(f'[pts] {len(xyz)} points', flush=True)
    col = gas.color_adjust(col, a.brighten, a.gamma, a.saturation)
    vox = gas.voxelize(xyz, col, a.z_stretch)

    # 面几何 = gen_anime_slices 的 --dual-face: A 穿心 0mm / B 偏 13.4mm。
    # 渲染约定与 1-bit 那份**完全一致** (面B 仍按 +13.4 同手性渲), 所以板端
    # 照旧靠 PHASE_B = 半圈 (=n/2=25) 补偿面B 的符号/手性 —— 见 pov_rxd.c
    # PHASE_B_DUAL 上方那段推导。两边约定必须成对, 别只改一边。
    faces = gas.plan_faces(None, a.gap_mm, n, dual_face=True)
    bufs, hists = [], []
    for name, axis_off, n_out, how in faces:
        gas.print_geom(axis_off, a.mirror_u, f'面{name}: {how}')
        b, h = render_face(vox, axis_off, n_out, a.sub, a.thresh,
                           not a.no_dither, a.mirror_u, a.gamma_led, f'面{name}')
        bufs += b
        hists.append(h)

    blob = b''.join(bufs)
    assert len(blob) == total * STRIDE_3BIT, (len(blob), total * STRIDE_3BIT)
    out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'wb') as f:
        f.write(blob)
    print(f'[out] {out}: {len(blob)} B = {total} 片 × 0x{STRIDE_3BIT:X} '
          f'(面A {n} + 面B {n}); 面拆分点 0x{n * STRIDE_3BIT:X}', flush=True)
    for (name, _off, _n, _how), h in zip(faces, hists):
        tot = int(h.sum())
        print(f'[code] 面{name} 码值分布 ' +
              ' '.join(f'{c}:{100.0 * int(h[c]) / tot:.1f}%' for c in range(8)), flush=True)

    if a.frames_dir:
        d = os.path.abspath(a.frames_dir)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'frame_0000.bin'), 'wb') as f:
            f.write(blob)
        meta = {'anim': 'boot-default-3bit', 'frames': 1, 'render_slices': n,
                'frame_raw': len(blob), 'freeze_phase': True,
                'bpp': 3, 'led_gamma': a.gamma_led,
                'n_slices': total, 'geom_flags': 0x08,          # DUAL_FACE
                'faces': [{'name': nm, 'axis_off_px': round(off, 4), 'n_slices': k}
                          for nm, off, k, _ in faces],
                'generated': time.strftime('%Y-%m-%d %H:%M:%S')}
        with open(os.path.join(d, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        print(f'[frames] {d} (frame_0000.bin + meta.json)', flush=True)

    if a.png:
        save_png(bufs[:n], a.png, n, a.gamma_led)


if __name__ == '__main__':
    main()
