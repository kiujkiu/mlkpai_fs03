#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_board.py — PVS1 协议回环测试服务器 (零硬件测 PC 侧全链路).

收帧 → 按 flags 解压 → (FLAG_DELTA: 与上一帧 ACK 的 raw XOR 重建) →
校验 raw_len==4,423,680 → sha256 → ACK. 参考帧按连接维护, 连接首帧带
DELTA 按协议 NAK (无参考帧), 连接保持让发送端降级重发 keyframe.
可选 --save-png 每帧 unpack slice 0 (pack_obs.unpack_slice) 存 PNG 目验.
可选 --md5 DIR: 与 DIR 内 sorted frame_*.bin 逐帧 md5 比对 (帧号 mod N
回绕), 校验 delta 重建正确性, 连接结束打 PASS/FAIL 汇总.

  python3 fake_board.py --once --save-png out/           # 收一条连接后退出
  python3 fake_board.py --once --md5 frames_spinpulse    # md5 比对模式
  python3 povstream.py stream --dir frames_spinpulse --host 127.0.0.1
"""
import os
import sys
import glob
import socket
import hashlib
import argparse
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
sys.path.insert(0, HERE)
import pack_obs
from povstream import (MAGIC, HDR, FRAME_RAW, N_SLICES, ACK, NAK,
                       FLAG_DELTA, decompress_frame, xor_frames)


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def save_slice0_png(raw, path):
    from PIL import Image
    import numpy as np
    img = pack_obs.unpack_slice(raw[:pack_obs.SLICE_DATA]).astype(np.uint8) * 255
    Image.fromarray(img).resize((pack_obs.W * 3, pack_obs.H * 3), Image.NEAREST).save(path)


def handle(conn, args, ref_md5=None):
    n = 0
    t0 = time.time()
    wire = 0
    prev_raw = None                                 # DELTA 参考帧 (按连接)
    key_n = key_wire = delta_n = delta_wire = 0
    md5_pass = md5_fail = 0
    while True:
        hdr = recv_exact(conn, HDR.size)
        if hdr is None:
            break                                   # 对端关连接 = 流结束
        magic, comp_len, raw_len, n_slices, flags = HDR.unpack(hdr)
        if magic != MAGIC or raw_len != FRAME_RAW or n_slices != N_SLICES:
            print(f'[fake_board] BAD HDR magic={magic!r} raw_len={raw_len} '
                  f'n_slices={n_slices}', flush=True)
            conn.sendall(bytes([NAK]))
            return False
        payload = recv_exact(conn, comp_len)
        if payload is None:
            print('[fake_board] EOF mid-payload', flush=True)
            return False
        raw = decompress_frame(payload, flags & ~FLAG_DELTA)
        if len(raw) != raw_len:
            print(f'[fake_board] LEN MISMATCH {len(raw)} != {raw_len}', flush=True)
            conn.sendall(bytes([NAK]))
            return False
        if flags & FLAG_DELTA:
            if prev_raw is None:                    # 协议: 无参考帧的 DELTA → NAK
                print(f'[fake_board] frame {n}: DELTA 无参考帧, NAK '
                      f'(等发送端降级 keyframe)', flush=True)
                conn.sendall(bytes([NAK]))
                continue                            # 连接保持, 等重发
            raw = xor_frames(prev_raw, raw)
            delta_n += 1; delta_wire += HDR.size + comp_len
        else:
            key_n += 1; key_wire += HDR.size + comp_len
        sha = hashlib.sha256(raw).hexdigest()
        md5s = ''
        if ref_md5:
            ok = hashlib.md5(raw).hexdigest() == ref_md5[n % len(ref_md5)]
            md5_pass += ok
            md5_fail += not ok
            md5s = f', md5={"OK" if ok else "MISMATCH!"}'
        print(f'[fake_board] frame {n}: {comp_len}B wire, flags=0x{flags:x}'
              f'{" D" if flags & FLAG_DELTA else " K"}, '
              f'{raw_len / comp_len:.1f}x, sha256={sha[:16]}{md5s}', flush=True)
        if args.save_png:
            os.makedirs(args.save_png, exist_ok=True)
            save_slice0_png(raw, os.path.join(args.save_png, f'frame_{n:04d}_slice0.png'))
        conn.sendall(bytes([ACK]))
        prev_raw = raw                              # 参考帧 = 上一 ACK 的 raw
        n += 1
        wire += HDR.size + comp_len
    dt = max(time.time() - t0, 1e-6)
    print(f'[fake_board] conn done: {n} frames, {wire / dt / 1e6:.2f} MB/s wire, '
          f'{n / dt:.2f} fps', flush=True)
    if n:
        avg = wire / n
        line = (f'[fake_board] avg wire {avg:.0f} B/帧 → est '
                f'{avg * 26 * 8 / 1e6:.2f} Mbps @26fps')
        if key_n:
            line += f' | key {key_n} 帧 avg {key_wire / key_n:.0f}B'
        if delta_n:
            line += f' | delta {delta_n} 帧 avg {delta_wire / delta_n:.0f}B'
        print(line, flush=True)
    if ref_md5:
        print(f'[fake_board] md5 check: {md5_pass}/{md5_pass + md5_fail} '
              f'{"PASS" if md5_fail == 0 and md5_pass else "FAIL"}', flush=True)
    return True


def main():
    ap = argparse.ArgumentParser(description='PVS1 loopback test server')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=9500)
    ap.add_argument('--once', action='store_true', help='收完一条连接就退出')
    ap.add_argument('--save-png', default=None, metavar='DIR',
                    help='每帧 unpack slice 0 存 PNG 到 DIR')
    ap.add_argument('--md5', default=None, metavar='DIR',
                    help='与 DIR 内 sorted frame_*.bin 逐帧 md5 比对 (帧号 mod N)')
    args = ap.parse_args()

    ref_md5 = None
    if args.md5:
        files = sorted(glob.glob(os.path.join(args.md5, 'frame_*.bin'))
                       or glob.glob(os.path.join(args.md5, '*.bin')))
        if not files:
            sys.exit(f'--md5: no .bin frames in {args.md5}')
        ref_md5 = [hashlib.md5(open(p, 'rb').read()).hexdigest() for p in files]
        print(f'[fake_board] md5 参考 {len(ref_md5)} 帧 from {args.md5}', flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f'[fake_board] listening {args.host}:{args.port}', flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f'[fake_board] client {addr}', flush=True)
        try:
            handle(conn, args, ref_md5)
        finally:
            conn.close()
        if args.once:
            break
    srv.close()


if __name__ == '__main__':
    main()
