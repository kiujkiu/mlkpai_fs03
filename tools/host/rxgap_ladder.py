#!/usr/bin/env python3
"""rxgap_ladder.py — "收包为什么只有 3 MB/s" 的归因阶梯 (2026-08-06)

端到端只知道两个数: 裸链路 14.5 MB/s, 实际收一帧 272 KB 只有 2.83 MB/s。
中间隔着好几样东西, 一次全加上就没法归因。本脚本把它们**一样一样加回去**,
每加一样测一次分布, 掉在哪一级一目了然。

阶梯 (每一级只比上一级多一样东西):
  L1 bulk        纯灌流, sink 只 recv 不回 ACK, 板上没有别的负载
  L2 frame w2    切成 272 KB 一帧, 每帧回 1 字节 ACK, 发送窗口 2 (= povstream 默认)
  L3 frame w1    发送窗口 1 (stop-and-wait) —— 量 ACK 往返的代价
  L4 frame w4    发送窗口 4 —— 窗口开大能不能补回来
  L5 frame w2+work  sink 收完忙等 26 ms 再 ACK (= 板端解码, 与收包串行)
  L6 bulk+load   纯灌流, 但板上同时跑满 CPU 的负载 (= pov_rxd 解码/翻页在抢核)

🔴 交错: 所有级别在**同一轮**里依次跑一遍, 跑 --reps 轮。绝不先跑完一级再跑
   下一级 —— 这条 WiFi 链路的环境漂移能把 8 伪装成 12 (上一轮的假阳性教训)。
"""
import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rxgap_send as S                                    # noqa: E402

PLINK = os.environ.get('PLINK', '/mnt/c/Program Files/PuTTY/plink.exe')
HK = os.environ.get('HK', 'SHA256:u14U8c0RuKnVinQuaGH5ey6OKScaPOlRF3vMNqSnEGI')
SINK = '/home/uisrc/rxgap_sink.py'


def sudo(host, cmd, timeout=90):
    p = subprocess.run([PLINK, '-ssh', '-batch', '-hostkey', HK, '-pw', 'root',
                        f'uisrc@{host}', "echo root | sudo -S sh -c '" +
                        cmd.replace("'", "'\\''") + "' 2>/dev/null"],
                       capture_output=True, text=True, timeout=timeout,
                       errors='replace')
    return p.stdout


def start_sink(host, mode, chunk, work):
    sudo(host, f"systemctl stop rxsink 2>/dev/null; "
               f"systemctl reset-failed rxsink 2>/dev/null; "
               f"systemd-run --unit=rxsink python3 {SINK} --mode {mode} "
               f"--chunk {chunk} --work {work}")
    time.sleep(1.5)


def start_load(host, n):
    """板上起 n 个满核 spinner (模拟 pov_rxd 抢核)。"""
    sudo(host, "systemctl stop rxload 2>/dev/null; "
               "systemctl reset-failed rxload 2>/dev/null; " +
               (f"systemd-run --unit=rxload sh -c 'for i in $(seq {n}); do "
                f"(while :; do :; done) & done; wait'" if n else ""))
    time.sleep(0.5)


def stop_load(host):
    sudo(host, "systemctl stop rxload 2>/dev/null; "
               "systemctl reset-failed rxload 2>/dev/null")


LADDER = [
    # key,        标签,                  mode,   window, work, load
    ('L1', '裸流 bulk (板子空闲)',        'bulk',  0, 0.0, 0),
    ('L2', '272K 分帧 +ACK, window=2',    'frame', 2, 0.0, 0),
    ('L3', '272K 分帧 +ACK, window=1',    'frame', 1, 0.0, 0),
    ('L4', '272K 分帧 +ACK, window=4',    'frame', 4, 0.0, 0),
    ('L5', 'window=2 + 26ms 解码(串行)',  'frame', 2, 26.0, 0),
    ('L6', '裸流 bulk + 板上 2 核满载',   'bulk',  0, 0.0, 2),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--secs', type=float, default=8.0)
    ap.add_argument('--chunk', type=int, default=272 * 1024)
    ap.add_argument('--only', default='', help='逗号分隔的级别 key, 如 L1,L2')
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    steps = [s for s in LADDER
             if not a.only or s[0] in a.only.split(',')]
    acc = {s[0]: dict(mbs=[], per=[]) for s in steps}
    for r in range(a.reps):
        print(f'--- 第 {r+1}/{a.reps} 轮 (交错) ---')
        for key, label, mode, win, work, load in steps:
            start_sink(a.host, mode, a.chunk, work)
            if load:
                start_load(a.host, load)
            try:
                mbs, per, k = S.run(a.host, 9600, mode, a.chunk, max(win, 1),
                                    a.secs)
            finally:
                if load:
                    stop_load(a.host)
            acc[key]['mbs'].append(mbs)
            acc[key]['per'].extend(per)
            print(f'  {key} {label:<28} {mbs:6.2f} MB/s  帧={k}')
            time.sleep(0.5)

    print('\n===== 汇总 (交错 %d 轮 x %.0fs) =====' % (a.reps, a.secs))
    for key, label, *_ in steps:
        d = S.dist(acc[key]['mbs'])
        line = f'{key} {label:<28} {S.fmt(d, " MB/s")}'
        if acc[key]['per']:
            line += f'\n     每帧周期 {S.fmt(S.dist(acc[key]["per"]), " ms")}'
        print(line)
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(acc, f)
    sudo(a.host, "systemctl stop rxsink rxload 2>/dev/null; "
                 "systemctl reset-failed rxsink rxload 2>/dev/null")


if __name__ == '__main__':
    main()
