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

🔴 flags 里的**片距位**同样是协议原样 (2026-08-20 v3.4):
   PVS_FLAG_3BIT (1<<7) 一置, 板端 PVS_STRIDE(flags) 就从 0x3000 跳到 0x9000,
   面拆分点、n_slices 上限跟着一起变 (idle_anim_step 与网络帧同一套规则)。
   ⚠ 光看帧字节数**分不清色深**: 3686400B 既可以是 300 片 1-bit 也可以是
     100 片 3-bit。所以色深必须由 meta.json 的 "bpp" (或 --bpp) 明说,
     猜错 = 片距错一倍 = 整帧错位 (而且板端一句日志都不会有)。

布局来源 (优先级从高到低):
   ① 命令行 --bpp / --dual-face / --fold-a / --single
   ② 源目录里的 meta.json ("bpp" + "geom_flags", povstream/gen_wedge 都写这个)
   ③ 帧字节数反推 (仅 1-bit 老目录: 360 单面 / 540 fold / 720 双面)

用法:
    python3 make_idle_anim.py frames_groot_dual720 anim.pvs
    python3 make_idle_anim.py stream/pc/frames_wedge3 anim3.pvs --bpp 3
    python3 make_idle_anim.py anime_dual3b100.bin anim3.pvs --bpp 3 --dual-face
    scp anim.pvs uisrc@<board>:/home/uisrc/

自检: python3 tools/test_idle_anim_3bit.py
"""
import os
import sys
import glob
import json
import zlib
import struct
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pack_obs

# ---- 协议常量 (stream/protocol.h 的镜像; 这里不 import 板端头文件) ----------
PLANE_STRIDE = 0x3000                       # 一个位平面 = 12288B
SLICE_STRIDE = PLANE_STRIDE                 # 1-bit 片距
SLICE_STRIDE_3BIT = 3 * PLANE_STRIDE        # 0x9000 = 36864
FRAME_RAW_MAX = pack_obs.FRAME_RAW_MAX      # 唯一定义在 pack_obs, 别在这里抄常量
N_SLICES_FULL = 360
N_SLICES_FOLD = 180

FLAG_RLE, FLAG_ZLIB, FLAG_DELTA = 1 << 0, 1 << 1, 1 << 2
FLAG_DUAL_FACE, FLAG_FOLD_A = 1 << 3, 1 << 4
FLAG_LZ4, FLAG_MSTREAM, FLAG_3BIT = 1 << 5, 1 << 6, 1 << 7
FLAG_GEOM = FLAG_DUAL_FACE | FLAG_FOLD_A    # 描述**面拆分**的位
FLAG_LAYOUT = FLAG_GEOM | FLAG_3BIT         # 面拆分 + 片距 = 板端还原布局要的全部


def slice_stride(bpp):
    if bpp == 1:
        return SLICE_STRIDE
    if bpp == 3:
        return SLICE_STRIDE_3BIT
    raise ValueError(f'bpp={bpp} 不支持 (只有 1 / 3)')


def face_a_slices(n_slices, flags):
    """面A 片数 (板端 idle_anim_step / serve_client 的同一条算式)。0 = 单面。"""
    if not (flags & FLAG_DUAL_FACE):
        return 0
    div = 3 if (flags & FLAG_FOLD_A) else 2
    if n_slices % div:
        raise SystemExit(f'n_slices={n_slices} 不是 {div} 的整数倍, '
                         f'与 flags=0x{flags:x} 的面拆分不自洽')
    return n_slices // div


# ---- 布局解析 --------------------------------------------------------------
def read_meta(src):
    """源目录里的 meta.json → dict (没有就 {})。povstream 与 gen_wedge 都写它。"""
    if not os.path.isdir(src):
        return {}
    path = os.path.join(src, 'meta.json')
    if not os.path.exists(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:                       # 坏 meta 不该炸掉整条流水线
        print(f'[warn] {path} 读不动 ({e}), 忽略', flush=True)
        return {}


def resolve_layout(raw_len, meta, args):
    """(帧字节数, meta.json, CLI) → (bpp, layout_flags)。

    ⚠ geom_flags 在两个生产者之间不一致: povstream 把 FLAG_3BIT 也 OR 进
    geom_flags, gen_wedge 只写 bpp 而 geom_flags 恒 0。所以这里一律
    「几何位取 geom_flags & FLAG_GEOM, 色深位只认 bpp (退化时才看 3BIT 位)」。"""
    # --- 色深 ---
    if args.bpp is not None:
        bpp = args.bpp
    elif meta.get('bpp') in (1, 3):
        bpp = meta['bpp']
    elif int(meta.get('geom_flags', 0)) & FLAG_3BIT:
        bpp = 3
    else:
        bpp = 1
    stride = slice_stride(bpp)

    # --- 几何 (面拆分) ---
    if args.single or args.dual_face or args.fold_a:
        geom = ((FLAG_DUAL_FACE if args.dual_face else 0)
                | (FLAG_FOLD_A if args.fold_a else 0))
    elif 'geom_flags' in meta:
        geom = int(meta['geom_flags']) & FLAG_GEOM
    else:
        geom = infer_geom_1bit(raw_len, bpp)

    flags = geom | (FLAG_3BIT if bpp == 3 else 0)

    # --- 一致性 (对不上就停, 别让板端静默丢帧) ---
    if raw_len % stride:
        raise SystemExit(f'帧长 {raw_len} 不是片距 0x{stride:X} (bpp={bpp}) 的整数倍 '
                         f'—— 色深/片距推错了? 用 --bpp 明说')
    n_slices = raw_len // stride
    n_max = FRAME_RAW_MAX // stride
    if not 1 <= n_slices <= n_max:
        raise SystemExit(f'n_slices={n_slices} 越界 (bpp={bpp} 上限 {n_max})')
    if meta.get('n_slices') not in (None, n_slices):
        raise SystemExit(f'meta.json 说 n_slices={meta["n_slices"]}, 帧长反推是 '
                         f'{n_slices} (片距 0x{stride:X}) —— 目录被拼过?')
    face_a_slices(n_slices, flags)               # 只为触发面拆分自检
    return bpp, flags, n_slices, stride


def infer_geom_1bit(raw_len, bpp):
    """老 1-bit 目录 (没 meta.json) 的片数→几何反推, 与 2026-08-03 版逐字节一致。"""
    if bpp != 1:
        raise SystemExit('3-bit 源没有 meta.json 时必须显式给 --dual-face/--single '
                         '—— 片数反推那张表只对 1-bit 的 360/540/720 有效')
    n = raw_len // SLICE_STRIDE
    if n == N_SLICES_FULL:
        return 0                                                   # 单面老帧
    if n == N_SLICES_FULL * 2:
        return FLAG_DUAL_FACE
    if n == N_SLICES_FOLD + N_SLICES_FULL:
        return FLAG_DUAL_FACE | FLAG_FOLD_A
    raise SystemExit(f'不认识的片数 {n} (无 meta.json 时只支持 360 / 540 / 720); '
                     f'请补 meta.json 或显式给 --bpp/--dual-face/--fold-a')


# ---- 打包 ------------------------------------------------------------------
def frame_files(src):
    if os.path.isfile(src):
        return [src]                                   # 单个 .bin = 一帧静态画面
    files = sorted(glob.glob(os.path.join(src, 'frame_*.bin')))
    if not files:
        raise SystemExit(f'没有找到 {src}/frame_*.bin')
    return files


def encode_payload(raw, fbo, level):
    """一帧 raw → PVS1 线上 payload 原样。

    fbo = 面B 的字节偏移 (0 = 单面)。双面**必须**两条独立流 + 4B 前缀, 见
    文件头那段血泪: 压成一条流板端会读到垃圾 clen_a 然后每帧静默 return。"""
    if not fbo:
        return zlib.compress(raw, level)
    a = zlib.compress(raw[:fbo], level)
    b = zlib.compress(raw[fbo:], level)
    return struct.pack('<I', len(a)) + a + b


def build_container(files, n_slices, flags, stride, level):
    fbo = face_a_slices(n_slices, flags) * stride
    raw_len = n_slices * stride
    payloads = []
    for f in files:
        d = open(f, 'rb').read()
        if len(d) != raw_len:
            raise SystemExit(f'{f}: {len(d)}B != {raw_len}B (n_slices={n_slices} '
                             f'× 0x{stride:X}) —— 目录里帧长不一致')
        payloads.append(encode_payload(d, fbo, level))
    blob = bytearray(b'PVSA')
    blob += struct.pack('<III', len(files), n_slices, flags)
    off = 16 + len(files) * 8
    for c in payloads:
        blob += struct.pack('<II', off, len(c))
        off += len(c)
    for c in payloads:
        blob += c
    return bytes(blob), fbo


def main():
    ap = argparse.ArgumentParser(
        description='预渲染帧 → 板端空闲动画容器 anim.pvs',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('src', help='帧目录 (frame_*.bin [+ meta.json]) 或单个 .bin')
    ap.add_argument('dst', help='输出容器路径')
    ap.add_argument('--bpp', type=int, choices=(1, 3), default=None,
                    help='色深 (片距 0x3000/0x9000)。默认读 meta.json 的 bpp; '
                         '两者都没有才按 1-bit 反推')
    ap.add_argument('--dual-face', action='store_true',
                    help='载荷 = [面A][面B] (板端两个 DDR 基址)')
    ap.add_argument('--fold-a', action='store_true', help='面A 折叠 (需同时 --dual-face)')
    ap.add_argument('--single', action='store_true', help='强制单面 (覆盖 meta.json)')
    ap.add_argument('--level', type=int, default=6, help='zlib 压缩级 (默认 6)')
    args = ap.parse_args()

    files = frame_files(args.src)
    raw_len = os.path.getsize(files[0])
    meta = read_meta(args.src)
    bpp, flags, n_slices, stride = resolve_layout(raw_len, meta, args)
    flags |= FLAG_ZLIB                    # 容器只出 zlib (板端 idle 路径不解 MSTREAM)

    blob, fbo = build_container(files, n_slices, flags, stride, args.level)
    with open(args.dst, 'wb') as out:
        out.write(blob)

    sz = len(blob)
    names = [n for b, n in ((FLAG_ZLIB, 'ZLIB'), (FLAG_DUAL_FACE, 'DUAL_FACE'),
                            (FLAG_FOLD_A, 'FOLD_A'), (FLAG_3BIT, '3BIT'))
             if flags & b]
    print(f'{args.dst}: {len(files)} 帧 n_slices={n_slices} bpp={bpp} '
          f'片距=0x{stride:X} flags=0x{flags:x} ({"|".join(names)}) '
          f'({sz / 1048576:.1f} MB, 平均 {sz // len(files) // 1024} KB/帧)')
    print(f'  raw {n_slices * stride}B/帧 (bank 上限 {FRAME_RAW_MAX}B, '
          f'占 {n_slices * stride * 100.0 / FRAME_RAW_MAX:.0f}%)')
    if fbo:
        first = open(files[0], 'rb').read()
        p0 = encode_payload(first, fbo, args.level)
        ca = struct.unpack('<I', p0[:4])[0]
        print(f'  首帧校验: 4 + 面A {ca} + 面B {len(p0) - 4 - ca} = {len(p0)} B; '
              f'面拆分点 = {fbo // stride} 片 = 0x{fbo:X}')


if __name__ == '__main__':
    main()
