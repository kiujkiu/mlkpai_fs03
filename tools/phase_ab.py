#!/usr/bin/env python3
"""phase_ab.py — 多配置**交错** A/B/C 端到端测量台 (2026-08-06)

phase_bench.py 一次只测一个配置; 想比较两个配置只能先跑完 A 再跑完 B, 而这条
链路的环境漂移足以把结论反过来 (上一轮实测: 同一配置两次 20 s 窗口能报出
7.49 与 10.14 帧/秒)。本脚本把 N 个配置**在同一轮里依次各跑一次**, 跑 R 轮,
再按配置汇总分布 —— 漂移被均摊到所有配置上, 不再记到某一个头上。

支持切 WiFi 频段 (band=5 / 24), 因为 2.4 GHz ch6 上挤了 9 个以上 BSS,
换 5 GHz 是本轮最大的单项改动, 必须和别的改动一起交错验证。

用法:
  tools/phase_ab.py --rounds 4 --secs 20 \\
      --cfg '5G/win2:band=5' --cfg '5G/win4:band=5,stream=--window 4' \\
      --cfg '2.4G/win2:band=24'
"""
import argparse
import json
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import phase_bench as PB                                   # noqa: E402


def set_band(band):
    if not band:
        return
    out = PB.sudo(f"/home/uisrc/wifi_band.sh {band}", timeout=120)
    freq = ''
    for ln in out.splitlines():
        if ln.startswith('freq='):
            freq = ln.strip()
    print(f"    [band {band}] {freq}")
    return freq


def parse_cfg(s):
    """'标签:k=v,k=v' -> dict。stream=/rxd= 里可以带空格。"""
    label, _, rest = s.partition(':')
    d = dict(label=label, band='', stream='', rxd='', lz4='', dec0='', pre='')
    for kv in rest.split(','):
        if not kv:
            continue
        k, _, v = kv.partition('=')
        d[k.strip()] = v.strip()
    return d


def main():
    global BOARD
    ap = argparse.ArgumentParser()
    ap.add_argument('--cfg', action='append', required=True)
    ap.add_argument('--rounds', type=int, default=4)
    ap.add_argument('--secs', type=int, default=20)
    ap.add_argument('--rps', type=float, default=15.0)
    ap.add_argument('--fps', type=float, default=40.0)
    ap.add_argument('--dir', default=PB.DEFAULT_DIR)
    ap.add_argument('--lz4-level', type=int, default=9)
    ap.add_argument('--out', default='')
    ap.add_argument('--no-restore', action='store_true')
    a = ap.parse_args()

    PB.BOARD = PB.board_ip()
    print('board =', PB.BOARD)
    cfgs = [parse_cfg(c) for c in a.cfg]
    acc = {c['label']: [] for c in cfgs}
    meta = {c['label']: dict(cfg=c, runs=[]) for c in cfgs}

    class A:                              # phase_bench.run_once 要的参数壳
        pass
    try:
        for r in range(a.rounds):
            print(f'--- 第 {r+1}/{a.rounds} 轮 (交错) ---')
            for c in cfgs:
                set_band(c['band'])
                args = A()
                args.rps, args.fps, args.dir = a.rps, a.fps, a.dir
                args.lz4_level = int(c['lz4']) if c['lz4'] else a.lz4_level
                args.secs = a.secs
                args.rxd_args, args.stream_args = c['rxd'], c['stream']
                args.warm, args.tail = 2, 1
                # pre=: 每次 run 前在板上跑一条命令。存在的理由是 sysctl 类改动
                # **会留在系统里**, 不复位的话后一个配置会继承前一个的状态,
                # 交错就白交错了。
                if c['pre']:
                    PB.sudo(c['pre'])
                PB.ALLOW_DEC0 = bool(c['dec0'])
                print(f'  [{c["label"]}]', end=' ')
                res = PB.run_once(args, r + 1)
                acc[c['label']].extend(res['samples'])
                meta[c['label']]['runs'].append(
                    dict(n=len(res['samples']), drop=res['drop'],
                         sender_fps=res['sender_fps']))
    finally:
        if not a.no_restore:
            PB.restore()

    print('\n===== 汇总 (交错 %d 轮 x %ds) =====' % (a.rounds, a.secs))
    keys = ('flip', 'rx', 'dec', 'cpy', 'wait', 'hdr', 'body')
    rows = {}
    for lab, pool in acc.items():
        rows[lab] = {k: PB.dist([s[k] for s in pool]) for k in keys}
        rows[lab]['n_pool'] = len(pool)
    for lab in acc:
        print(f'\n### {lab}   (n={rows[lab]["n_pool"]} 个秒窗)')
        for k in keys:
            print(f'  {k:6s} {PB.fmt(rows[lab][k])}')
    base = cfgs[0]['label']
    for lab in acc:
        if lab == base:
            continue
        print(f'\n--- {base} -> {lab} ---')
        for k in keys:
            da, db = rows[base][k], rows[lab][k]
            if not da.get('n') or not db.get('n'):
                continue
            sp = ((da['sd'] ** 2 + db['sd'] ** 2) / 2) ** 0.5 or 1e-9
            print(f'  {k:6s} {da["median"]:7.2f} -> {db["median"]:7.2f}  '
                  f'({db["median"]-da["median"]:+.2f}, σ {da["sd"]:.2f}/{db["sd"]:.2f}, '
                  f'效应量 {(db["median"]-da["median"])/sp:+.2f})')
    if a.out:
        json.dump(dict(rows=rows, meta=meta, samples=acc), open(a.out, 'w'),
                  ensure_ascii=False, indent=1)
        print('  ->', a.out)


if __name__ == '__main__':
    main()
