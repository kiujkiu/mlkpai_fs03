#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_board.py — PVS 协议回环测试服务器 (零硬件测 PC 侧全链路, 含 v2 delta).

收帧 → 按 flags 解压 (delta 帧对连接内影子帧 XOR 应用; 无锚点 NAK 但保持
连接, 等 sender 重发关键帧) → 校验 raw_len==4,423,680 → sha256 → ACK.
可选 --save-png 每帧 unpack slice 0 (pack_obs.unpack_slice) 存 PNG 目验.

  python3 fake_board.py --once --save-png out/           # 收一条连接后退出
  python3 povstream.py stream --dir frames_spinpulse --host 127.0.0.1
"""
import os
import sys
import socket
import hashlib
import argparse
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
sys.path.insert(0, HERE)
import pack_obs
from povstream import (MAGIC, HDR, FRAME_RAW, N_SLICES, ACK, NAK,
                       FLAG_DELTA, FLAG_ZLIB, decompress_frame, rle_decode)


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


def handle(conn, args):
    n = 0
    t0 = time.time()
    wire = 0
    shadow = None                       # v2 delta 锚点, 按连接失效
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
        if flags & FLAG_DELTA:
            if shadow is None:          # 无锚点: NAK 但保持连接 (v2)
                print('[fake_board] delta without anchor -> NAK (conn kept)',
                      flush=True)
                conn.sendall(bytes([NAK]))
                continue
            import numpy as np
            if flags & FLAG_ZLIB:       # 0x7: zlib 包 RLE 流
                import zlib
                payload = zlib.decompress(payload)
            mask = np.frombuffer(rle_decode(payload), np.uint8)
            raw = (np.frombuffer(shadow, np.uint8) ^ mask).tobytes()
        else:
            raw = decompress_frame(payload, flags)
        if len(raw) != raw_len:
            print(f'[fake_board] LEN MISMATCH {len(raw)} != {raw_len}', flush=True)
            conn.sendall(bytes([NAK]))
            return False
        sha = hashlib.sha256(raw).hexdigest()
        print(f'[fake_board] frame {n}: {comp_len}B wire, flags=0x{flags:x}, '
              f'{raw_len / comp_len:.1f}x, sha256={sha[:16]}', flush=True)
        if args.save_png:
            os.makedirs(args.save_png, exist_ok=True)
            save_slice0_png(raw, os.path.join(args.save_png, f'frame_{n:04d}_slice0.png'))
        conn.sendall(bytes([ACK]))
        shadow = raw
        n += 1
        wire += HDR.size + comp_len
    dt = max(time.time() - t0, 1e-6)
    print(f'[fake_board] conn done: {n} frames, {wire / dt / 1e6:.2f} MB/s wire, '
          f'{n / dt:.2f} fps', flush=True)
    return True


def main():
    ap = argparse.ArgumentParser(description='PVS1 loopback test server')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=9500)
    ap.add_argument('--once', action='store_true', help='收完一条连接就退出')
    ap.add_argument('--save-png', default=None, metavar='DIR',
                    help='每帧 unpack slice 0 存 PNG 到 DIR')
    args = ap.parse_args()

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
            handle(conn, args)
        finally:
            conn.close()
        if args.once:
            break
    srv.close()


if __name__ == '__main__':
    main()
