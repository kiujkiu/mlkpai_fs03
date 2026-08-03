#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_idle_anim.py — 把一套预渲染帧打成板端空闲动画容器 anim.pvs (2026-08-03).

用途: pov_rxd --idle-anim anim.pvs 在**没有客户端连接**时循环播这个容器,
一有推流自动让位 => "上电就有画面, 有推就显示推的内容"。

容器格式 (全小端):
    'PVSA' | u32 n_frames | u32 n_slices | u32 flags | n×(u32 off, u32 len) | 载荷…

🔴 载荷必须是 **PVS1 线上 payload 原样**, 因为板端复用同一条解码路径 (零特例)。
   所以 DUAL_FACE 帧必须是**两条独立压缩流** + 4 字节前缀:
       [u32 comp_len_A][zlib(面A)][zlib(面B)]
   ⚠ 2026-08-03 踩过: 我图省事用 zlib.compress(整帧) 压成一条流, 板端按双流
     解析读到垃圾 clen_a, 每帧静默 return, 表现为"加载成功但一帧不播"。

用法:
    python3 make_idle_anim.py frames_groot_dual720 anim.pvs
    scp anim.pvs uisrc@<board>:/home/uisrc/
"""
import os
import sys
import glob
import zlib
import struct

SLICE_STRIDE = 0x3000
N_SLICES_FULL = 360
N_SLICES_FOLD = 180
FLAG_ZLIB, FLAG_DUAL_FACE, FLAG_FOLD_A = 1 << 1, 1 << 3, 1 << 4


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    files = sorted(glob.glob(os.path.join(src, 'frame_*.bin')))
    if not files:
        sys.exit(f'没有找到 {src}/frame_*.bin')

    raw_len = os.path.getsize(files[0])
    n_slices = raw_len // SLICE_STRIDE
    if raw_len % SLICE_STRIDE:
        sys.exit(f'帧长 {raw_len} 不是 0x3000 的整数倍')

    # 从帧长反推布局 (与 povstream 的 meta.json 一致, 但不依赖它)
    if n_slices == N_SLICES_FULL:
        flags, fbo = FLAG_ZLIB, 0                                  # 单面老帧
    elif n_slices == N_SLICES_FULL * 2:
        flags, fbo = FLAG_ZLIB | FLAG_DUAL_FACE, N_SLICES_FULL * SLICE_STRIDE
    elif n_slices == N_SLICES_FOLD + N_SLICES_FULL:
        flags = FLAG_ZLIB | FLAG_DUAL_FACE | FLAG_FOLD_A
        fbo = N_SLICES_FOLD * SLICE_STRIDE
    else:
        sys.exit(f'不认识的片数 {n_slices} (支持 360 / 540 / 720)')

    payloads = []
    for f in files:
        d = open(f, 'rb').read()
        if fbo:                                   # 双面 -> 两条独立流 + 4B 前缀
            a, b = zlib.compress(d[:fbo], 6), zlib.compress(d[fbo:], 6)
            payloads.append(struct.pack('<I', len(a)) + a + b)
        else:
            payloads.append(zlib.compress(d, 6))

    with open(dst, 'wb') as out:
        out.write(b'PVSA')
        out.write(struct.pack('<III', len(files), n_slices, flags))
        off = 16 + len(files) * 8
        for c in payloads:
            out.write(struct.pack('<II', off, len(c)))
            off += len(c)
        for c in payloads:
            out.write(c)

    sz = os.path.getsize(dst)
    print(f'{dst}: {len(files)} 帧 n_slices={n_slices} flags=0x{flags:x} '
          f'({sz/1048576:.1f} MB, 平均 {sz//len(files)//1024} KB/帧)')
    if fbo:
        ca = struct.unpack('<I', payloads[0][:4])[0]
        print(f'  首帧校验: 4 + 面A {ca} + 面B {len(payloads[0])-4-ca} = {len(payloads[0])} B')


if __name__ == '__main__':
    main()
