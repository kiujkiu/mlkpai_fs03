#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_delta.py — v2 delta 协议 PC 侧测试 (x86 无板, 无 GUI).

覆盖:
  1. rle_encode 向量化版 vs 参考实现逐字节一致 + 往返
  2. DeltaEncoder 100 帧合成动画往返 (影子模型逐帧核对) + GOP 关键帧计数
  3. force_keyframe / delta 退化自动回退关键帧
  4. Streamer NAK → 关键帧重发 (v2 恢复协议, 假板保持连接)
  5. Streamer 断线重连 → 新连接首帧自动关键帧
  6. e2e: 真 pov_rxd_sim (stream/board, --bench) 收 delta 流, CRC 逐帧核对
     (需先 make -C ../board sim; 缺 binary 则 skip)

跑法: python3 test_delta.py -v
"""
import os
import sys
import signal
import socket
import struct
import subprocess
import threading
import time
import unittest
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from povstream import (FRAME_RAW, N_SLICES, MAGIC, HDR, ACK, NAK,
                       FLAG_RLE, FLAG_ZLIB, FLAG_DELTA,
                       rle_encode, rle_decode, DeltaEncoder,
                       Streamer, StreamerError)

BOARD = os.path.join(HERE, '..', 'board')


def rle_encode_ref(data):
    """协议参考实现 (v1 povstream 的逐字节循环), 用来钉死线格式."""
    a = np.frombuffer(data, np.uint8)
    out = bytearray()
    pos = 0
    for j in np.flatnonzero(a):
        j = int(j)
        run = j - pos
        while run > 0:
            r = min(run, 65535)
            out += b'\x00' + r.to_bytes(2, 'little')
            run -= r
        out.append(int(a[j]))
        pos = j + 1
    run = len(a) - pos
    while run > 0:
        r = min(run, 65535)
        out += b'\x00' + r.to_bytes(2, 'little')
        run -= r
    return bytes(out)


def synth_frame(seed, base=None, n_mut=3000):
    """合成帧: base=None 生成 ~40K 非零稀疏帧; 否则 base 上 n_mut 处突变
    (含归零), 模拟相邻动画帧."""
    rng = np.random.default_rng(seed)
    if base is None:
        f = np.zeros(FRAME_RAW, np.uint8)
        pos = rng.integers(0, FRAME_RAW, 40000)
        val = rng.integers(1, 256, 40000).astype(np.uint8)
        f[pos] = val
        return f.tobytes()
    f = np.frombuffer(base, np.uint8).copy()
    pos = rng.integers(0, FRAME_RAW, n_mut)
    f[pos] = rng.integers(0, 256, n_mut).astype(np.uint8)   # 1/256 归零
    return f.tobytes()


class TestRLE(unittest.TestCase):
    def test_matches_reference_and_roundtrips(self):
        rng = np.random.default_rng(7)
        cases = [
            b'\x00' * 100000,                                # 全零
            b'\x01' * 999,                                   # 全字面
            b'\x00' * 70000 + b'\x42' + b'\x00' * 70000,     # 跨 65535 游程
            synth_frame(1)[:200000],                         # 稀疏
            bytes(rng.integers(0, 4, 50000, dtype=np.uint8)),  # 密集混合
            b'',
        ]
        for i, buf in enumerate(cases):
            enc = rle_encode(buf)
            self.assertEqual(enc, rle_encode_ref(buf), f'case {i}: != 参考实现')
            self.assertEqual(rle_decode(enc), buf, f'case {i}: 往返失败')


class TestDeltaEncoder(unittest.TestCase):
    def _decode(self, shadow, payload, flags):
        """接收端影子模型 (支持 0x5 裸 RLE 和 0x7 zlib 包 RLE)."""
        if flags & FLAG_DELTA:
            self.assertIsNotNone(shadow, 'delta 无锚点')
            self.assertTrue(flags & FLAG_RLE)
            if flags & FLAG_ZLIB:
                payload = zlib.decompress(payload)
            mask = np.frombuffer(rle_decode(payload), np.uint8)
            return (np.frombuffer(shadow, np.uint8) ^ mask).tobytes()
        self.assertEqual(flags, FLAG_ZLIB)
        return zlib.decompress(payload)

    def test_100_frames_roundtrip_gop(self):
        enc = DeltaEncoder(gop=10)
        shadow = None
        raw = synth_frame(100)
        kf = 0
        for i in range(100):
            if i:
                raw = synth_frame(200 + i, raw)
            payload, flags = enc.encode(raw)
            if not flags & FLAG_DELTA:
                kf += 1
                self.assertEqual(i % 10, 0, f'frame {i}: 非 GOP 位置关键帧')
            shadow = self._decode(shadow, payload, flags)
            self.assertEqual(shadow, raw, f'frame {i}: 影子 != 原始帧')
        self.assertEqual(kf, 10)          # gop=10, 100 帧 → 10 关键帧
        self.assertEqual(enc.deltas, 90)

    def test_force_keyframe_and_fallback(self):
        enc = DeltaEncoder(gop=0)          # 无周期关键帧
        raw = synth_frame(1)
        _, flags = enc.encode(raw)
        self.assertEqual(flags, FLAG_ZLIB, '首帧必须关键帧')
        raw2 = synth_frame(2, raw)
        _, flags = enc.encode(raw2)
        self.assertTrue(flags & FLAG_DELTA)
        self.assertTrue(flags & FLAG_RLE)
        enc.force_keyframe()               # NAK / 重连路径
        raw3 = synth_frame(3, raw2)
        _, flags = enc.encode(raw3)
        self.assertEqual(flags, FLAG_ZLIB, 'force_keyframe 未生效')
        # 场景切换: 与上一帧完全不相关 → delta 退化 → 自动关键帧
        rng = np.random.default_rng(9)
        wild = rng.integers(0, 256, FRAME_RAW, dtype=np.uint8).tobytes()
        _, flags = enc.encode(wild)
        self.assertEqual(flags, FLAG_ZLIB, 'delta 退化未回退关键帧')


class FakeBoard(threading.Thread):
    """极简假板: 单线程 accept 循环, 每帧回 ACK; 可注入 NAK / 断连.

    nak_first_delta=True: 第一个 delta 帧回 NAK 但保持连接 (v2 语义).
    drop_after=N: ACK N 帧后断开连接一次 (测重连关键帧).
    记录 (conn_id, flags) 序列 + 每帧解码影子, CRC 存 self.crcs.
    """

    def __init__(self, port, nak_first_delta=False, drop_after=0):
        super().__init__(daemon=True)
        self.port = port
        self.nak_first_delta = nak_first_delta
        self.drop_after = drop_after
        self.seen = []                   # (conn_id, flags)
        self.crcs = []                   # 每 ACK 帧的解码 CRC
        self.stop = threading.Event()
        self.lsock = socket.socket()
        self.lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.lsock.bind(('127.0.0.1', port))
        self.lsock.listen(1)
        self.lsock.settimeout(0.2)

    def _recv_full(self, c, n):
        buf = b''
        while len(buf) < n:
            chunk = c.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def run(self):
        conn_id = 0
        naked_delta = False
        while not self.stop.is_set():
            try:
                c, _ = self.lsock.accept()
            except socket.timeout:
                continue
            conn_id += 1
            shadow = None                # 板端 shadow 按连接失效
            acked = 0
            with c:
                c.settimeout(5)
                while not self.stop.is_set():
                    try:
                        hdr = self._recv_full(c, HDR.size)
                    except (socket.timeout, OSError):
                        break
                    if not hdr:
                        break
                    magic, comp_len, raw_len, n_slices, flags = HDR.unpack(hdr)
                    assert magic == MAGIC and raw_len == FRAME_RAW
                    payload = self._recv_full(c, comp_len)
                    if payload is None:
                        break
                    self.seen.append((conn_id, flags))
                    if flags & FLAG_DELTA:
                        if shadow is None or (self.nak_first_delta and not naked_delta):
                            if self.nak_first_delta and shadow is not None:
                                naked_delta = True
                            c.sendall(bytes([NAK]))   # 连接保持
                            continue
                        if flags & FLAG_ZLIB:         # 0x7: zlib 包 RLE 流
                            payload = zlib.decompress(payload)
                        mask = np.frombuffer(rle_decode(payload), np.uint8)
                        shadow = (np.frombuffer(shadow, np.uint8) ^ mask).tobytes()
                    elif flags & FLAG_ZLIB:
                        shadow = zlib.decompress(payload)
                    elif flags & FLAG_RLE:
                        shadow = rle_decode(payload)
                    else:
                        shadow = payload
                    self.crcs.append(zlib.crc32(shadow))
                    c.sendall(bytes([ACK]))
                    acked += 1
                    if self.drop_after and acked == self.drop_after:
                        self.drop_after = 0          # 只断一次
                        break                        # 关闭连接
        self.lsock.close()

    def shutdown(self):
        self.stop.set()
        self.join(timeout=5)


def stream_frames(streamer, frames):
    it_done = []

    def make_iter():
        if it_done:                      # loop=False, 只走一遍
            return iter(())
        it_done.append(1)
        return iter(frames)
    streamer.run(make_iter)


class TestStreamerV2(unittest.TestCase):
    PORT = 9541

    def _frames(self, n, seed=50):
        frames = [synth_frame(seed)]
        for i in range(1, n):
            frames.append(synth_frame(seed + i, frames[-1]))
        return frames

    def test_nak_triggers_keyframe(self):
        fb = FakeBoard(self.PORT, nak_first_delta=True)
        fb.start()
        try:
            frames = self._frames(5)
            s = Streamer('127.0.0.1', self.PORT, fps=0, codec='delta', gop=0)
            stream_frames(s, frames)
            self.assertEqual(s.frames, 5)
            self.assertEqual(fb.crcs, [zlib.crc32(f) for f in frames],
                             '假板影子序列 != 发送帧')
            flags = [f for _, f in fb.seen]
            # kf, delta(NAK), kf 重发, delta, delta, delta
            self.assertEqual(flags[0], FLAG_ZLIB)
            self.assertTrue(flags[1] & FLAG_DELTA)
            self.assertEqual(flags[2], FLAG_ZLIB, 'NAK 后未重发关键帧')
            self.assertTrue(all(f & FLAG_DELTA for f in flags[3:]))
            self.assertEqual(len(flags), 6)
            self.assertEqual(s.naks, 1)
        finally:
            fb.shutdown()

    def test_reconnect_sends_keyframe(self):
        fb = FakeBoard(self.PORT + 1, drop_after=2)
        fb.start()
        try:
            frames = self._frames(5, seed=70)
            s = Streamer('127.0.0.1', self.PORT + 1, fps=0, codec='delta',
                         gop=0, reconnect=True, retry_interval=0.1,
                         ack_timeout=3)
            stream_frames(s, frames)
            self.assertEqual(s.frames, 5)
            self.assertEqual(fb.crcs, [zlib.crc32(f) for f in frames])
            # 新连接首帧必须是关键帧
            conn2 = [f for cid, f in fb.seen if cid == 2]
            self.assertTrue(conn2, '没有发生重连')
            self.assertFalse(conn2[0] & FLAG_DELTA, '重连首帧不是关键帧')
            self.assertGreaterEqual(s.reconnects, 1)
        finally:
            fb.shutdown()

    def test_pvs1_zlib_regression(self):
        """老式 zlib 整帧 sender 仍工作 (假板 = 纯 PVS1 接收)."""
        fb = FakeBoard(self.PORT + 2)
        fb.start()
        try:
            frames = self._frames(3, seed=90)
            s = Streamer('127.0.0.1', self.PORT + 2, fps=0, codec='zlib')
            stream_frames(s, frames)
            self.assertEqual(s.frames, 3)
            self.assertEqual(fb.crcs, [zlib.crc32(f) for f in frames])
            self.assertTrue(all(f == FLAG_ZLIB for _, f in fb.seen))
        finally:
            fb.shutdown()


@unittest.skipUnless(os.path.exists(os.path.join(BOARD, 'pov_rxd_sim')),
                     'pov_rxd_sim 未编译 (make -C ../board sim)')
class TestE2ESimDaemon(unittest.TestCase):
    """真板端 daemon (x86 SIM 编译) 收 python delta 流, CRC 逐帧核对."""
    PORT = 9551

    def test_delta_stream_to_sim(self):
        proc = subprocess.Popen(
            [os.path.join(BOARD, 'pov_rxd_sim'), '--port', str(self.PORT),
             '--fake', '20', '--crc', '--bench'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            time.sleep(0.3)
            frames = [synth_frame(400)]
            for i in range(1, 8):
                frames.append(synth_frame(400 + i, frames[-1]))
            s = Streamer('127.0.0.1', self.PORT, fps=0, codec='delta', gop=3)
            stream_frames(s, frames)
            self.assertEqual(s.frames, 8)
            self.assertGreater(s.delta_frames, 0)
            self.assertGreater(s.kf_frames, 1)      # gop=3 → 多个关键帧
        finally:
            proc.send_signal(signal.SIGINT)
            out, _ = proc.communicate(timeout=10)
        crcs = []
        for line in out.splitlines():
            if 'FRAME ' in line and 'crc=' in line:
                crcs.append(int(line.split('crc=')[1].split()[0], 16))
        self.assertEqual(crcs, [zlib.crc32(f) for f in frames],
                         'daemon 解码 CRC != 发送帧')


if __name__ == '__main__':
    unittest.main(verbosity=2)
