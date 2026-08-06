#!/usr/bin/env python3
"""rxgap_send.py — 收包阶梯测试的发送端 (配 tools/host/rxgap_sink.py, 2026-08-06)

一次跑一组阶梯并**交错重复**, 汇报分布 (中位/p5/p95/σ)。
交错是必须的: 这条 WiFi 链路上单次测量分不开 8 和 12 (上一轮的假阳性教训),
先跑完 A 再跑完 B 会把环境漂移记到配置头上。

用法:
  tools/host/rxgap_send.py --host $B --ladder --reps 5 --secs 8
  tools/host/rxgap_send.py --host $B --mode frame --window 2 --secs 10
"""
import argparse
import json
import math
import os
import socket
import statistics
import sys
import time


def pct(xs, q):
    if not xs:
        return float('nan')
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    i = q / 100.0 * (len(s) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def dist(xs):
    if not xs:
        return dict(n=0)
    return dict(n=len(xs), median=statistics.median(xs), mean=statistics.fmean(xs),
                p5=pct(xs, 5), p95=pct(xs, 95),
                sd=statistics.pstdev(xs) if len(xs) > 1 else 0.0,
                min=min(xs), max=max(xs))


def fmt(d, unit=''):
    if not d.get('n'):
        return '(无样本)'
    return (f"中位 {d['median']:.2f}{unit} p5 {d['p5']:.2f} p95 {d['p95']:.2f} "
            f"σ {d['sd']:.2f} n={d['n']}")


def run(host, port, mode, chunk, window, secs, fps=0.0, nodelay=True,
        sndbuf=0, verbose=False):
    """跑一次, 返回 (MB/s, 每帧耗时样本 ms, 帧数)。

    frame 模式: 发 chunk 字节, 窗口满才回收 1 字节 ACK —— 与 povstream 的
    --window 语义完全一致。每帧耗时 = 该帧 sendall 起到它的 ACK 回来 (窗口
    满时) 的墙钟, 取的是"稳态每帧周期", 所以用相邻帧完成时刻的差分。
    """
    payload = os.urandom(chunk) if mode == 'frame' else os.urandom(262144)
    c = socket.create_connection((host, port), timeout=20)
    if nodelay:
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if sndbuf:
        c.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sndbuf)
    c.settimeout(20)
    t0 = time.perf_counter()
    n = 0
    per = []
    acked = 0
    inflight = 0
    last = t0
    period = 1.0 / fps if fps > 0 else 0.0
    next_send = t0
    try:
        while time.perf_counter() - t0 < secs:
            if period:
                now = time.perf_counter()
                if now < next_send:
                    time.sleep(next_send - now)
                next_send += period
            c.sendall(payload)
            n += len(payload)
            if mode == 'frame':
                inflight += 1
                while inflight >= window:
                    b = c.recv(1)
                    if not b:
                        raise ConnectionResetError('sink closed')
                    inflight -= 1
                    acked += 1
                    now = time.perf_counter()
                    per.append((now - last) * 1000.0)
                    last = now
    except (socket.timeout, ConnectionResetError, BrokenPipeError, OSError) as e:
        print('  ! send err', e, file=sys.stderr)
    dt = time.perf_counter() - t0
    try:
        c.shutdown(socket.SHUT_WR)
    except OSError:
        pass
    c.close()
    mbs = n / dt / 1e6
    return mbs, per, acked


LADDER = [
    # (标签, mode, window, work_ms_on_sink, fps)
    ('L1 裸流 (无 ACK)',            'bulk',  0, 0.0, 0),
    ('L2 分帧272K+ACK win=2',       'frame', 2, 0.0, 0),
    ('L3 分帧272K+ACK win=1',       'frame', 1, 0.0, 0),
    ('L4 分帧272K+ACK win=4',       'frame', 4, 0.0, 0),
    ('L5 win=2 + 26ms 解码',        'frame', 2, 26.0, 0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--port', type=int, default=9600)
    ap.add_argument('--mode', choices=('bulk', 'frame'), default='bulk')
    ap.add_argument('--chunk', type=int, default=272 * 1024)
    ap.add_argument('--window', type=int, default=2)
    ap.add_argument('--secs', type=float, default=8.0)
    ap.add_argument('--fps', type=float, default=0.0)
    ap.add_argument('--reps', type=int, default=1)
    ap.add_argument('--nagle', action='store_true', help='不设 TCP_NODELAY')
    ap.add_argument('--sndbuf', type=int, default=0)
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    res = {}
    for r in range(a.reps):
        mbs, per, k = run(a.host, a.port, a.mode, a.chunk, a.window, a.secs,
                          a.fps, not a.nagle, a.sndbuf)
        res.setdefault('x', []).append(mbs)
        res.setdefault('per', []).extend(per)
        print(f'  rep {r+1}: {mbs:.2f} MB/s ({mbs*8:.0f} Mbps) 帧={k}')
        time.sleep(1.0)
    print(f'\nMB/s  {fmt(dist(res["x"]))}')
    if res.get('per'):
        print(f'每帧ms {fmt(dist(res["per"]))}')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(dict(cfg=vars(a), mbs=res['x'], per=res['per']), f)


if __name__ == '__main__':
    main()
