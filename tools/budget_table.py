#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
budget_table.py — 由 budget_scan.py 的压缩实测 + A9 解压实测吞吐, 算每一级余量。

判据: 30 fps 体帧率下, **链路余量 ≥2×** 且 **解码余量 ≥2×**。

── 输入常量的来源 (谁是实测, 谁是外推) ──────────────────────────────
[实测·A9] 单核解压吞吐 (输出字节/耗时, 8.85MB 载荷上量的):
          lz4 204.6 / zstd9 53.6 / zstd19 52.5 / zlib6 51.6 MiB/s
[实测·链路] 125 Mbps 可用吞吐
[实测·本机] 各配置压缩后字节数 (budget_scan.py, ctypes 直调 block 格式)
[外推]     解压耗时 = 输出字节 / 单核吞吐 —— 假设吞吐与分块大小无关
           (A9 L2 只有 512KB, 块都是 MB 级, 是流式访存, 假设合理但没验)
[外推]     双核 makespan = 两核各自分到的流的耗时之和取大者 (最优二分)
[实测校准] 并行折损: 已知 fold+按面切 实测加速 1.42×, 而本模型理想值 1.50×
           ⇒ 折损系数 1.056。derate 列 = 理想 makespan × 1.056。
           注: 原始账里的「720 片双核 48 fps」= 本模型理想值 48.5 fps,
           即那个数没含折损。两个数据点只够定一个常数, 所以两列都给。

用法:
  python3 tools/budget_table.py --scan /tmp/budget_scan.json
  python3 tools/budget_table.py --scan ... --hybrid     # 混合编解码器搜索
"""
import os
import sys
import json
import argparse
import itertools

MIB = 1048576.0
SLICE_STRIDE = 0x3000

# ---- 实测常量 ----
DEC_MIBPS = {          # A9 单核解压吞吐 (MiB/s), 实测
    'lz4': 204.6, 'zstd9': 53.6, 'zstd19': 52.5, 'zstd22': 52.5, 'zlib6': 51.6,
    'rle': None,       # 未测
}
LINK_MBPS = 125.0      # 实测可用链路吞吐
FPS = 30.0             # 目标体帧率
CORES = 2              # Zynq-7020 双 A9
DERATE = 1.056         # 并行折损, 由 fold 实测 1.42x / 模型 1.50x 反解
TARGET = 2.0


def dec_class(codec):
    if codec.startswith('lz4'):
        return 'lz4'
    return codec


def work_ms(out_bytes, codec):
    t = DEC_MIBPS.get(dec_class(codec))
    if not t:
        return None
    return out_bytes / MIB / t * 1000.0


def makespan(works, cores=CORES):
    """把若干条流分给 cores 个核, 求最优 (最小) makespan。流不可再分。"""
    if cores == 1 or len(works) == 1:
        return sum(works)
    best = sum(works)
    n = len(works)
    for mask in range(1 << n):                       # 只做 2 核, 暴力子集
        a = sum(w for i, w in enumerate(works) if mask >> i & 1)
        best = min(best, max(a, sum(works) - a))
    return best


def link_mbps(wire_bytes, fps=FPS):
    return wire_bytes * 8 * fps / 1e6


def evaluate(row):
    """row = budget_scan 的一条 → 余量字典。"""
    out_per_stream = [n * SLICE_STRIDE for n in row['seg_slices']]
    if row['plan'] == 's1':                          # 一条流 = 单核
        works = [work_ms(sum(out_per_stream), row['codec'])]
        ms_ideal = works[0] if works[0] else None
    else:
        works = [work_ms(b, row['codec']) for b in out_per_stream]
        ms_ideal = None if None in works else makespan(works)
    mb = link_mbps(row['wire'])
    r = dict(row)
    r['mbps'] = mb
    r['link_margin'] = LINK_MBPS / mb
    if ms_ideal is None:
        r['dec_ms'] = r['dec_ms_derate'] = r['dec_fps'] = r['dec_margin'] = None
    else:
        r['dec_ms'] = ms_ideal
        r['dec_ms_derate'] = ms_ideal * DERATE
        r['dec_fps'] = 1000.0 / r['dec_ms_derate']
        r['dec_margin'] = r['dec_fps'] / FPS
        r['dec_margin_ideal'] = 1000.0 / ms_ideal / FPS
    # 端到端「处处 2x」能撑到的帧率 = 两级 2x 帧率上限取小
    fl = LINK_MBPS / TARGET / 8 / r['wire'] * 1e6
    r['fps2x'] = fl if r['dec_ms'] is None else min(fl, r['dec_fps'] / TARGET)
    return r


def fmt(r):
    d = '    n/a' if r['dec_ms'] is None else f"{r['dec_ms']:7.1f}"
    dd = '    n/a' if r['dec_ms'] is None else f"{r['dec_ms_derate']:7.1f}"
    dm = '  n/a ' if r['dec_margin'] is None else f"{r['dec_margin']:5.2f}x"
    ok = ''
    if r['dec_margin'] is not None:
        both = r['link_margin'] >= TARGET and r['dec_margin'] >= TARGET
        ok = ' <== 双达标' if both else ''
    return (f"{r['payload']:8} {r['plan']:6} {r['codec']:8} {r['streams']}流 "
            f"{r['wire']:>8}B {r['mbps']:6.2f}Mbps {r['link_margin']:5.2f}x | "
            f"{d}ms {dd}ms {dm} {r['fps2x']:5.1f}fps{ok}")


def main_table(rows, codecs, plans, payloads):
    print(f"\n{'载荷':8} {'分流':6} {'编码':8} 流数 {'线上':>9} {'链路需求':>7} 链路余量 |"
          f" 解码ms 折损后 解码余量 处处2x帧率")
    print('-' * 104)
    for r in rows:
        if r['codec'] not in codecs or r['plan'] not in plans or r['payload'] not in payloads:
            continue
        print(fmt(evaluate(r)))


# ---------------- 混合编解码器搜索 ----------------

def hybrid(src, nchunk_slices=90, cache=None):
    """把 fold540 切成 90 片一块 (面A 2 块 + 面B 4 块), 每块独立选编解码器。
    问: **存不存在**一种分配, 让链路和解码同时 ≥2×?
    这是给「做不到的话差多少」定量的上界搜索 —— 需要协议支持逐流编码器标志,
    现协议没有, 属于「要改才可能」的方案。"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import budget_scan as BS
    if cache and os.path.exists(cache):
        chunks = json.load(open(cache))
    else:
        sl = BS.load_slices(src)
        segs = ([('A', i, i + nchunk_slices) for i in range(0, 180, nchunk_slices)] +
                [('B', i, i + nchunk_slices) for i in range(0, 360, nchunk_slices)])
        faces = (sl[:360], sl[360:])
        chunks = []
        for f, i, j in segs:
            buf = b''.join(faces[0 if f == 'A' else 1][i:j])
            e = {'seg': f'{f}{i}-{j}', 'out': len(buf)}
            for cd in ('lz4hc12', 'lz4hc9', 'zstd9', 'zstd19', 'zstd22'):
                e[cd] = len(BS.CODECS[cd](buf))
            chunks.append(e)
            print('[hybrid] chunk', e, flush=True)
        if cache:
            json.dump(chunks, open(cache, 'w'), indent=1)
    cands = ['lz4hc12', 'zstd9', 'zstd19', 'zstd22']
    n = len(chunks)
    best = None
    for combo in itertools.product(cands, repeat=n):
        works = [work_ms(chunks[i]['out'], combo[i]) for i in range(n)]
        ms = makespan(works) * DERATE
        dec_m = (1000.0 / ms) / FPS
        wire = 16 + 4 * (n - 1) + sum(chunks[i][combo[i]] for i in range(n))
        lnk_m = LINK_MBPS / link_mbps(wire)
        key = (min(dec_m, lnk_m), lnk_m)
        if best is None or key > best[0]:
            best = (key, combo, wire, lnk_m, ms, dec_m)
    (_, combo, wire, lnk_m, ms, dec_m) = best
    print(f'\n[hybrid] 6 块 × {len(cands)} 编码器穷举 (含折损 {DERATE}):')
    print(f'[hybrid] 最优 (最大化两级余量里的较小者): {list(zip([c["seg"] for c in chunks], combo))}')
    print(f'[hybrid] 线上 {wire}B → {link_mbps(wire):.2f} Mbps 链路余量 {lnk_m:.3f}x | '
          f'解码 {ms:.1f}ms 余量 {dec_m:.3f}x')
    print(f'[hybrid] 双 ≥2x = {lnk_m >= TARGET and dec_m >= TARGET}')
    # 再问: 强制解码 >=2x 下, 链路最好能到多少
    best2 = None
    for combo in itertools.product(cands, repeat=n):
        works = [work_ms(chunks[i]['out'], combo[i]) for i in range(n)]
        ms = makespan(works) * DERATE
        if (1000.0 / ms) / FPS < TARGET:
            continue
        wire = 16 + 4 * (n - 1) + sum(chunks[i][combo[i]] for i in range(n))
        if best2 is None or wire < best2[0]:
            best2 = (wire, combo, ms)
    if best2:
        wire, combo, ms = best2
        lm = LINK_MBPS / link_mbps(wire)
        print(f'[hybrid] 约束解码≥2x 下链路最优: {wire}B {link_mbps(wire):.2f} Mbps '
              f'余量 {lm:.3f}x (解码 {ms:.1f}ms) 组合={list(combo)}')
        need = LINK_MBPS / TARGET / 8 / FPS * 1e6
        print(f'[hybrid] 链路 2x 需要 ≤{need:.0f}B/帧, 还差 {wire-need:.0f}B '
              f'({100*(wire-need)/need:.1f}%); 或链路需 {link_mbps(wire)*TARGET:.1f} Mbps')
    else:
        print('[hybrid] 没有任何组合能让解码 ≥2x')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scan', default='budget_scan.json')
    ap.add_argument('--codecs', default='lz4hc9,lz4hc11,lz4hc12,lz4fast,zlib6,zstd9,zstd19,rle')
    ap.add_argument('--plans', default='s1,s2,s3nai,s3bal,s4')
    ap.add_argument('--payloads', default='raw720,fold540')
    ap.add_argument('--hybrid', action='store_true')
    ap.add_argument('--hybrid-cache', default='')
    a = ap.parse_args()
    d = json.load(open(a.scan))
    print(f"[in] {a.scan} src={d['src']}")
    print(f"[cfg] 链路 {LINK_MBPS} Mbps, 目标 {FPS} fps, {CORES} 核, "
          f"并行折损 {DERATE}, 判据 ≥{TARGET}x")
    main_table(d['rows'], set(a.codecs.split(',')), set(a.plans.split(',')),
               set(a.payloads.split(',')))
    print('\n注: 解码ms = 理想 makespan (外推), 折损后 = ×1.056 (由 1.42x 实测反解); '
          '解码余量按折损后算。')
    if a.hybrid:
        hybrid(d['src'], cache=a.hybrid_cache or None)


if __name__ == '__main__':
    main()
