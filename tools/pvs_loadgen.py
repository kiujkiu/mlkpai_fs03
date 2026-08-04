#!/usr/bin/env python3
"""pvs_loadgen.py — PVS1 推流负载发生器 / 端到端帧率量具 (纯标准库)。

为什么不是直接用 povstream.py:
  povstream 依赖 numpy/trimesh 现渲或预渲目录, 本机没有这套环境; 更重要的是
  **量帧率时不想把 PC 的渲染/压缩耗时算进去** —— 那会把板端瓶颈和 PC 瓶颈
  搅在一起。本工具直接重放板上那个 anim.pvs 容器里**已经压好**的载荷, 与
  povstream 走**逐字节相同**的线格式 (PVS1 头 + payload + 1 字节 ACK), 所以
  板端走的是完全同一条代码路径, 只是发送端零计算开销 = 纯链路+板端量具。

容器格式 (与 pov_rxd.c 的 idle-anim 一致, 小端):
  'PVSA' | u32 n_frames | u32 n_slices | u32 flags | n×(u32 off,u32 len) | 载荷…
  载荷就是 PVS1 的 payload 原样。

量什么:
  - ACK 节拍 = 板端 **rx (解码) 帧率**, 逐帧时间戳 -> 均值/标准差/p50/p95/max
  - 线速 Mbps
  - 可选 --window (在途帧数), --fps (发送节奏上限, 0 = 全速)
  ⚠ ACK 节拍**不是显示帧率**。显示帧率 = 板端 STAT 行里的 flip, 必须去板上读
    (pov_rxd 每秒一行 rx/flip/drop)。两者的差就是 drop, 见报告。

用法:
  python3 tools/pvs_loadgen.py --host 10.10.20.239 --file anim.pvs --seconds 30
  python3 tools/pvs_loadgen.py --host ... --fps 0 --window 3 --seconds 30
"""
import argparse
import socket
import struct
import sys
import time

MAGIC = b'PVS1'
HDR = struct.Struct('<4sIIHH')          # magic, comp_len, raw_len, n_slices, flags
ACK, NAK = 0x06, 0x15
SLICE_STRIDE = 0x3000


def load_container(path):
    """anim.pvs -> (n_slices, flags, [payload bytes, ...])"""
    with open(path, 'rb') as f:
        blob = f.read()
    if blob[:4] != b'PVSA':
        sys.exit(f'{path}: 不是 PVSA 容器')
    n, n_slices, flags = struct.unpack_from('<3I', blob, 4)
    idx = struct.unpack_from(f'<{2 * n}I', blob, 16)
    frames = [blob[idx[2 * i]:idx[2 * i] + idx[2 * i + 1]] for i in range(n)]
    return n_slices, flags, frames


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    k = min(len(sorted_vals) - 1, int(round(p / 100.0 * (len(sorted_vals) - 1))))
    return sorted_vals[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--port', type=int, default=9500)
    ap.add_argument('--file', required=True, help='anim.pvs 容器')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--frames', type=int, default=0, help='>0: 发满 N 帧就停')
    ap.add_argument('--fps', type=float, default=0.0,
                    help='发送节奏上限 (0 = 全速, 由 ACK/窗口自然限流)')
    ap.add_argument('--window', type=int, default=2,
                    help='在途未 ACK 帧数上限 (povstream 默认 2)')
    ap.add_argument('--tag', default='', help='结果行前缀, 方便多组对比')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    n_slices, flags, frames = load_container(a.file)
    raw_len = n_slices * SLICE_STRIDE
    if not a.quiet:
        print(f'[gen] {a.file}: {len(frames)} 帧 n_slices={n_slices} '
              f'flags=0x{flags:x} raw={raw_len} 平均载荷 '
              f'{sum(map(len, frames)) // len(frames)} B', flush=True)

    s = socket.create_connection((a.host, a.port), timeout=30)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    win = max(1, a.window)
    inflight = 0
    ack_t = []                 # 每个 ACK 的时刻
    sent_wire = 0
    i = 0
    t0 = time.perf_counter()
    t_next = t0
    try:
        while True:
            now = time.perf_counter()
            if now - t0 >= a.seconds:
                break
            if a.frames and len(ack_t) >= a.frames:
                break
            p = frames[i % len(frames)]
            i += 1
            s.sendall(HDR.pack(MAGIC, len(p), raw_len, n_slices, flags) + p)
            sent_wire += HDR.size + len(p)
            inflight += 1
            if inflight >= win:
                b = s.recv(1)
                if not b or b[0] != ACK:
                    sys.exit(f'[gen] 收到 {b!r} (NAK/断连), 中止')
                inflight -= 1
                ack_t.append(time.perf_counter())
            if a.fps > 0:
                t_next = max(t_next + 1.0 / a.fps, time.perf_counter() - 1.0 / a.fps)
                d = t_next - time.perf_counter()
                if d > 0:
                    time.sleep(d)
        # 排空在途 (不排空的话板端 ACK 无人接收, 且最后 win-1 帧不进统计)
        while inflight:
            b = s.recv(1)
            if not b or b[0] != ACK:
                break
            inflight -= 1
            ack_t.append(time.perf_counter())
    finally:
        s.close()

    n = len(ack_t)
    dt = ack_t[-1] - ack_t[0] if n > 1 else 0.0
    gaps = sorted((ack_t[k + 1] - ack_t[k]) * 1000.0 for k in range(n - 1))
    mean = sum(gaps) / len(gaps) if gaps else 0.0
    var = sum((g - mean) ** 2 for g in gaps) / len(gaps) if gaps else 0.0
    tag = f'{a.tag} ' if a.tag else ''
    print(f'[gen] {tag}ACK {n} 帧 / {dt:.2f}s = {(n - 1) / dt if dt else 0:.2f} fps'
          f' | 线速 {sent_wire * 8 / (time.perf_counter() - t0) / 1e6:.1f} Mbps'
          f' | ACK 间隔 ms: 均值 {mean:.1f} σ {var ** 0.5:.1f}'
          f' p50 {pct(gaps, 50):.1f} p95 {pct(gaps, 95):.1f} max {gaps[-1] if gaps else 0:.1f}',
          flush=True)


if __name__ == '__main__':
    main()
