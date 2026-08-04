#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
budget_scan.py — 链路预算参数扫描 (纯离线, x86 本地跑, 不碰板子)。

目的: 在「30fps 体帧率下每一级余量 ≥2×」的目标下, 扫编解码器 × 载荷形态 ×
分流方式, 量出**压缩后字节数**(决定链路余量) 和**各流字节数**(决定双核解码
makespan), 供 budget_table.py 算余量。

载荷形态 (从 720 片双面原始帧现构):
  raw720   = 面A 360 片 + 面B 360 片            = 8,847,360 B
  fold540  = 面A 前 180 片 + 面B 360 片          = 6,635,520 B   (--fold-a)
    ⚠ 折叠合法性依赖「面A 穿心 ⇒ slice_i ≡ mirror(slice_{i+180})」。本脚本
      --check 会用非零字节数做弱校验 (抖动相位不同 ⇒ 不会逐字节相等)。
      povstream.py 渲染时前 180 槽与整圈渲染逐字节相同, 故这样构造的
      fold540 就是 --fold-a 的真实载荷。

分流方式 (protocol.h: DUAL_FACE 载荷 = [u32 comp_len_A][面A 流][面B 流],
每条流独立压缩, 目的是板端两核并行解压):
  s1     一条流 (无法并行)
  s2     按面切 (现协议)
  s3bal  三条流, 切点按**解码负载均衡**选 (见 --split-plan)
  s4     四条流 (两核各拿两条, 可完美均衡)

压缩用 ctypes 直调 liblz4/libzstd 的 **block 格式** (LZ4_compress_HC /
LZ4_compress_default / ZSTD_compress), 与板端 C 接收方 LZ4_decompress_safe /
ZSTD_decompress 的用法一致 —— 不是 lz4/zstd CLI 的 frame 格式 (CLI 每 64KB 块
多 4B 块头 + 帧头/校验, 8.85MB 上多 ~800B)。

用法:
  python3 tools/budget_scan.py --check
  python3 tools/budget_scan.py --json /tmp/budget_scan.json
"""
import os
import sys
import json
import time
import ctypes
import argparse

SLICE_STRIDE = 0x3000          # 12288, = pack_obs.SLICE_STRIDE
N_FACE = 360                   # 单面整圈片数
N_FOLD = 180                   # --fold-a 折叠后面A 片数
DEFAULT_SRC = '/mnt/d/claude_workspace/hls_scratch/rle/anime_dual720.bin'

# ---------------- 编解码器 (ctypes, block 格式) ----------------

_lz4 = ctypes.CDLL('liblz4.so.1')
_lz4.LZ4_compressBound.argtypes = [ctypes.c_int]
_lz4.LZ4_compressBound.restype = ctypes.c_int
_lz4.LZ4_compress_HC.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                 ctypes.c_int, ctypes.c_int, ctypes.c_int]
_lz4.LZ4_compress_HC.restype = ctypes.c_int
_lz4.LZ4_compress_default.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                      ctypes.c_int, ctypes.c_int]
_lz4.LZ4_compress_default.restype = ctypes.c_int
_lz4.LZ4_decompress_safe.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                     ctypes.c_int, ctypes.c_int]
_lz4.LZ4_decompress_safe.restype = ctypes.c_int

_zstd = ctypes.CDLL('libzstd.so.1')
_zstd.ZSTD_compressBound.argtypes = [ctypes.c_size_t]
_zstd.ZSTD_compressBound.restype = ctypes.c_size_t
_zstd.ZSTD_compress.argtypes = [ctypes.c_char_p, ctypes.c_size_t,
                                ctypes.c_char_p, ctypes.c_size_t, ctypes.c_int]
_zstd.ZSTD_compress.restype = ctypes.c_size_t
_zstd.ZSTD_isError.argtypes = [ctypes.c_size_t]
_zstd.ZSTD_isError.restype = ctypes.c_uint


def lz4_hc(data, level):
    dst = ctypes.create_string_buffer(_lz4.LZ4_compressBound(len(data)))
    n = _lz4.LZ4_compress_HC(data, dst, len(data), len(dst), level)
    if n <= 0:
        raise RuntimeError('LZ4_compress_HC failed')
    return dst.raw[:n]


def lz4_fast(data, accel=1):
    dst = ctypes.create_string_buffer(_lz4.LZ4_compressBound(len(data)))
    n = _lz4.LZ4_compress_default(data, dst, len(data), len(dst))
    if n <= 0:
        raise RuntimeError('LZ4_compress_default failed')
    return dst.raw[:n]


def lz4_verify(comp, orig):
    dst = ctypes.create_string_buffer(len(orig))
    n = _lz4.LZ4_decompress_safe(comp, dst, len(comp), len(dst))
    return n == len(orig) and dst.raw[:n] == orig


def zstd(data, level):
    cap = _zstd.ZSTD_compressBound(len(data))
    dst = ctypes.create_string_buffer(cap)
    n = _zstd.ZSTD_compress(dst, cap, data, len(data), level)
    if _zstd.ZSTD_isError(n):
        raise RuntimeError('ZSTD_compress failed')
    return dst.raw[:n]


def zlib_c(data, level):
    import zlib
    return zlib.compress(data, level)


def rle_encode(data):
    """协议内置零游程 RLE (povstream.rle_encode 的等价纯 py 实现)."""
    out = bytearray()
    pos = 0
    n = len(data)
    while pos < n:
        j = data.find(b'\x00', pos)
        if j < 0:
            out += data[pos:]
            return bytes(out)
        out += data[pos:j]
        k = j
        while k < n and data[k] == 0:
            k += 1
        run = k - j
        while run > 0:
            r = min(run, 65535)
            out += b'\x00' + r.to_bytes(2, 'little')
            run -= r
        pos = k
    return bytes(out)


CODECS = {}
for _l in range(1, 13):
    CODECS[f'lz4hc{_l}'] = (lambda d, l=_l: lz4_hc(d, l))
CODECS['lz4fast'] = lz4_fast
for _l in (6,):
    CODECS[f'zlib{_l}'] = (lambda d, l=_l: zlib_c(d, l))
for _l in (9, 19, 22):
    CODECS[f'zstd{_l}'] = (lambda d, l=_l: zstd(d, l))
CODECS['rle'] = rle_encode

# ---------------- 载荷构造 ----------------


def load_slices(path):
    raw = open(path, 'rb').read()
    n, rem = divmod(len(raw), SLICE_STRIDE)
    if rem or n != 2 * N_FACE:
        sys.exit(f'{path}: 期望 {2*N_FACE} 片 × {SLICE_STRIDE}B, 实得 {len(raw)}B')
    return [raw[i * SLICE_STRIDE:(i + 1) * SLICE_STRIDE] for i in range(n)]


def build_payloads(sl):
    """→ {形态名: (面A 片列表, 面B 片列表)}"""
    a, b = sl[:N_FACE], sl[N_FACE:]
    return {
        'raw720': (a, b),                  # 8,847,360 B
        'fold540': (a[:N_FOLD], b),        # 6,635,520 B
    }


def check_fold(sl, verbose=True):
    """弱校验面A 穿心镜像恒等式: nz(slice_i) ≈ nz(slice_{i+180})。
    抖动相位随 slot 变化 ⇒ 不会逐字节相等, 只能比统计量。面B 作为对照组。"""
    def nz(s):
        return len(s) - s.count(0)
    res = {}
    for name, off in (('faceA', 0), ('faceB', N_FACE)):
        v = [nz(sl[off + i]) for i in range(N_FACE)]
        d = [abs(v[i] - v[i + N_FOLD]) for i in range(N_FOLD)]
        mean = sum(v) / N_FACE
        res[name] = {'nz_mean': mean, 'dev_mean': sum(d) / N_FOLD,
                     'dev_max': max(d), 'rel': sum(d) / N_FOLD / mean}
    if verbose:
        for k, v in res.items():
            print(f'[check] {k}: 片均非零 {v["nz_mean"]:.1f}B, '
                  f'|nz(i)-nz(i+180)| 均 {v["dev_mean"]:.1f} 最大 {v["dev_max"]} '
                  f'→ 相对偏差 {100*v["rel"]:.2f}%')
        ok = res['faceA']['rel'] < 0.02 and res['faceB']['rel'] > 0.05
        print(f'[check] 面A 可折叠 = {ok} (面A 偏差应 <2%, 面B 作为对照应 >5%)')
    return res

# ---------------- 分流 ----------------


def splits(nA, nB, plan):
    """→ 每条流的 (面, 起片, 止片) 列表。nA/nB = 两面片数."""
    A = ('A', 0, nA)
    if plan == 's1':
        return [A, ('B', 0, nB)]          # 语义上一条流, 实际拼接后整体压
    if plan == 's2':
        return [A, ('B', 0, nB)]
    if plan == 's3nai':                   # 面B 对半切 (朴素)
        return [A, ('B', 0, nB // 2), ('B', nB // 2, nB)]
    if plan == 's3bal':                   # 切点使两核负载完全均衡
        # 两核各 (nA+nB)/2 片: 核1 = 面A + 面B 前 k 片, 核2 = 面B 剩下
        k = (nA + nB) // 2 - nA
        return [A, ('B', 0, k), ('B', k, nB)]
    if plan == 's4':                      # 面A 对半 + 面B 对半
        return [('A', 0, nA // 2), ('A', nA // 2, nA),
                ('B', 0, nB // 2), ('B', nB // 2, nB)]
    raise ValueError(plan)


PLANS = ['s1', 's2', 's3nai', 's3bal', 's4']


def concat(faces, seg):
    f, i, j = seg
    return b''.join(faces[0 if f == 'A' else 1][i:j])

# ---------------- 扫描 ----------------


def scan(payloads, codecs, plans, reps=1):
    rows = []
    for pname, faces in payloads.items():
        nA, nB = len(faces[0]), len(faces[1])
        raw_total = (nA + nB) * SLICE_STRIDE
        for plan in plans:
            segs = splits(nA, nB, plan)
            if plan == 's1':
                bufs = [b''.join(faces[0]) + b''.join(faces[1])]
                seg_slices = [nA + nB]
            else:
                bufs = [concat(faces, s) for s in segs]
                seg_slices = [s[2] - s[1] for s in segs]
            for cname in codecs:
                fn = CODECS[cname]
                t0 = time.perf_counter()
                for _ in range(reps):
                    comp = [fn(b) for b in bufs]
                enc_ms = (time.perf_counter() - t0) * 1000 / reps
                sizes = [len(c) for c in comp]
                # 线上字节: 16B 帧头 + (流数-1) × 4B 长度前缀 + 各流
                wire = 16 + 4 * (len(sizes) - 1) + sum(sizes)
                if cname.startswith('lz4'):
                    for c, b in zip(comp, bufs):
                        assert lz4_verify(c, b), f'{cname}/{pname}/{plan} 解压回环失败'
                rows.append(dict(payload=pname, plan=plan, codec=cname,
                                 raw=raw_total, streams=len(sizes),
                                 sizes=sizes, seg_slices=seg_slices,
                                 comp=sum(sizes), wire=wire,
                                 ratio=raw_total / sum(sizes), enc_ms=enc_ms))
                print(f'{pname:8} {plan:6} {cname:9} streams={len(sizes)} '
                      f'comp={sum(sizes):>9} wire={wire:>9} '
                      f'ratio={raw_total/sum(sizes):6.2f}x enc={enc_ms:8.1f}ms '
                      f'sizes={sizes}', flush=True)
    return rows


def extra(sl):
    """两个被验证过 + 被否掉的数据侧点子, 留着免得再问一遍。"""
    A, B = sl[:N_FOLD], sl[N_FACE:]
    print('\n[extra-1] 去掉每片 624B 的 DDR 对齐填充 (11664 vs 12288) 值多少字节?')
    for lvl in (9, 12):
        for pad in (True, False):
            ca = lz4_hc(b''.join(s if pad else s[:SLICE_DATA] for s in A), lvl)
            cb = lz4_hc(b''.join(s if pad else s[:SLICE_DATA] for s in B), lvl)
            print(f'  fold540 s2 lz4hc{lvl} pad={int(pad)}: '
                  f'A={len(ca)} B={len(cb)} wire={20+len(ca)+len(cb)}')
    print('  → 省 ~1.3KB (0.5%), 且要改协议 (raw_len 变 n×11664, 板端散射写)。')

    print('\n[extra-2] 相邻切片异或 (沿 θ 做空间 delta) 能不能提高压缩比?')
    for nm, seq in (('面A 180 片', A), ('面B 360 片', B)):
        plain = lz4_hc(b''.join(seq), 12)
        prev = seq[0]
        parts = [seq[0]]
        for s in seq[1:]:
            parts.append(bytes(x ^ y for x, y in zip(s, prev)))
            prev = s
        xd = lz4_hc(b''.join(parts), 12)
        print(f'  {nm}: 原序 hc12={len(plain)} → 异或后 {len(xd)} '
              f'({100*len(xd)/len(plain):.1f}%)')
    print('  → 更差。抖动相位逐槽变 (phase = slot + frame*7), 相邻片的 Bayer 图案'
          '不同, 异或反而制造熵。同理不要指望帧间 DELTA 在不冻结相位时有红利。')


SLICE_DATA = 11664


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=DEFAULT_SRC)
    ap.add_argument('--check', action='store_true', help='只跑折叠恒等式自检')
    ap.add_argument('--extra', action='store_true', help='只跑去填充/相邻异或两个附加实验')
    ap.add_argument('--codecs', default='all')
    ap.add_argument('--plans', default=','.join(PLANS))
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    sl = load_slices(a.src)
    print(f'[src] {a.src} {len(sl)} 片 × {SLICE_STRIDE}B = {len(sl)*SLICE_STRIDE}B')
    chk = check_fold(sl)
    if a.check:
        return
    if a.extra:
        extra(sl)
        return
    codecs = list(CODECS) if a.codecs == 'all' else a.codecs.split(',')
    rows = scan(build_payloads(sl), codecs, a.plans.split(','))
    if a.json:
        json.dump({'src': a.src, 'check': chk, 'rows': rows},
                  open(a.json, 'w'), indent=1)
        print('[out]', a.json)


if __name__ == '__main__':
    main()
