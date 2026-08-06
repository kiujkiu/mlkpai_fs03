#!/usr/bin/env python3
"""rxgap_sink.py — 板端收包阶梯测试的 sink (2026-08-06)

为什么要它: 端到端只知道"收一帧 272 KB 要 96 ms (2.83 MB/s)", 而裸链路
能跑 14.5 MB/s。中间隔着三样东西 —— ①分帧 ②每帧 ACK 往返 ③收完还要解码
(与收包串行)。**把这三样一样一样加回去**, 每加一样测一次, 掉落就能归因。

本 sink 只做网络侧, 不碰 povmem/PL, 所以可以和 pov_rxd 独立地反复跑。

模式:
  bulk            —— 只 recv, 从不回 ACK   (= 裸链路上限)
  frame           —— 收 CHUNK 字节 → 回 1 字节 ACK  (加上"分帧+ACK 往返")
  frame --work M  —— 收完再忙等 M ms 再 ACK (加上"解码与收包串行")

⚠ 忙等而不是 sleep: 板子只有两个 A9 核, sleep 不占 CPU, 而真正的解码是占的。
   要复现串行开销就得真的占住核。

协议 (与 pov_rxd 无关, 自成一套):
  连接后 sink 先发 4 字节 chunk 大小的确认? —— 不, 保持最简: 大小由命令行
  两端约定, 对不上直接看得出来 (吞吐会是垃圾)。
"""
import argparse
import socket
import struct
import threading
import time


def recv_exact(c, n):
    b = b''
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d:
            return None
        b += d
    return b


def busy_ms(ms):
    if ms <= 0:
        return
    t_end = time.perf_counter() + ms / 1000.0
    x = 0
    while time.perf_counter() < t_end:
        for _ in range(200):
            x += 1
    return x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=9600)
    ap.add_argument('--mode', choices=('bulk', 'frame', 'pvs'), default='bulk')
    ap.add_argument('--chunk', type=int, default=272 * 1024)
    ap.add_argument('--work', type=float, default=0.0, help='每帧忙等 ms (模拟解码)')
    ap.add_argument('--rcvbuf', type=int, default=0, help='>0 时设 SO_RCVBUF')
    a = ap.parse_args()

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', a.port))
    s.listen(16)
    print(f'sink ready :{a.port} mode={a.mode} chunk={a.chunk} work={a.work}ms',
          flush=True)
    # 🔴 必须多线程 accept: 单连接串行 accept 时并行流测试里只有第一条流被
    #    服务, 其余全 timeout —— 会把"并行没有增益"这个**错误结论**做实。
    while True:
        c, addr = s.accept()
        threading.Thread(target=serve, args=(c, addr, a), daemon=True).start()


def serve(c, addr, a):
    if True:
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if a.rcvbuf:
            try:
                c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUFFORCE, a.rcvbuf)
            except OSError:
                c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, a.rcvbuf)
        eff = c.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        n = 0
        t0 = time.perf_counter()
        # 每帧耗时样本 (frame 模式)
        per = []
        try:
            if a.mode == 'bulk':
                while True:
                    d = c.recv(262144)
                    if not d:
                        break
                    n += len(d)
            elif a.mode == 'pvs':
                # 🔴 说 PVS1 协议的最小 sink: 收 16B 头 -> 收 comp_len 字节 -> ACK。
                # 为什么要它: pov_rxd --diag-rxonly 只收+ACK 也只有 3.2 MB/s, 而本
                # 文件的 frame 模式在同一条链路上有 5 MB/s。两者协议形状一样但
                # **发送端不同** (povstream vs rxgap_send), 没法直接归因。加了这个
                # 模式以后, 同一个 povstream 可以分别喂 C 的 pov_rxd 和这个 Python
                # sink, 变量就只剩"接收端"一个了。
                hdrs = struct.Struct('<4sIIHH')
                body = bytearray(16 << 20)
                view = memoryview(body)
                while True:
                    t_f = time.perf_counter()
                    h = recv_exact(c, 16)
                    if h is None:
                        break
                    magic, comp_len, raw_len, nsl, flags = hdrs.unpack(h)
                    if magic != b'PVS1':
                        print('坏帧头', magic, flush=True)
                        break
                    got = 0
                    while got < comp_len:
                        r = c.recv_into(view[got:], comp_len - got)
                        if r == 0:
                            break
                        got += r
                    if got < comp_len:
                        break
                    n += 16 + got
                    per.append((time.perf_counter() - t_f) * 1000.0)
                    busy_ms(a.work)
                    c.sendall(b'\x06')
            else:
                buf = bytearray(a.chunk)
                view = memoryview(buf)
                while True:
                    got = 0
                    t_f = time.perf_counter()
                    while got < a.chunk:
                        r = c.recv_into(view[got:], a.chunk - got)
                        if r == 0:
                            break
                        got += r
                    if got < a.chunk:
                        break
                    n += got
                    per.append((time.perf_counter() - t_f) * 1000.0)
                    busy_ms(a.work)
                    c.sendall(b'\x06')
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            print('conn err', e, flush=True)
        dt = max(time.perf_counter() - t0, 1e-6)
        c.close()
        per_s = ''
        if per:
            p = sorted(per)
            per_s = (f' | 每帧 {len(p)} 个: 中位 {p[len(p)//2]:.1f}ms '
                     f'p5 {p[max(0,int(0.05*len(p)))]:.1f} '
                     f'p95 {p[min(len(p)-1,int(0.95*len(p)))]:.1f} '
                     f'min {p[0]:.1f} max {p[-1]:.1f}')
        print('RX %.2f MB in %.2fs = %.2f MB/s (%.0f Mbps) rcvbuf=%d from %s%s'
              % (n / 1e6, dt, n / dt / 1e6, n / dt * 8 / 1e6, eff, addr[0], per_s),
              flush=True)


if __name__ == '__main__':
    main()
