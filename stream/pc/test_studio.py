#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_studio.py — POV Studio headless 测试 (WSL python3, 无 tkinter 也能跑).

  python3 stream/pc/test_studio.py

1. 协议往返: render_job 渲 mini globe 帧集 → Streamer → fake_board (--once),
   sha256 逐帧比对.
2. 自动重连: 推流中 kill fake_board → 重启 → 断言续推 + reconnects 计数.
3. 配置存取: save/load 往返 + 坏文件回默认.
4. 预览: packed bin → preview_image 尺寸/亮点断言.
"""
import os
import sys
import time
import json
import glob
import shutil
import hashlib
import tempfile
import threading
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import pov_studio
import povstream
from povstream import Streamer, StreamerError, FRAME_RAW

PORT = 19571
PY = sys.executable

_TMP = None
_FRAMES_DIR = None


def setUpModule():
    """渲一次 mini 帧集 (globe 2 帧, 8 渲染角), 全部测试共用."""
    global _TMP, _FRAMES_DIR
    _TMP = tempfile.mkdtemp(prefix='pov_studio_test_')
    _FRAMES_DIR = os.path.join(_TMP, 'frames_mini')
    t0 = time.time()
    out = pov_studio.render_job('globe_spin', 'globe', 2, '', render_slices=8,
                                out_dir=_FRAMES_DIR)
    assert out == _FRAMES_DIR
    print(f'[setup] mini render 2 frames in {time.time() - t0:.1f}s -> {out}')


def tearDownModule():
    shutil.rmtree(_TMP, ignore_errors=True)


def start_fake_board(*extra):
    """起 fake_board, 等它打印 listening (不能 TCP 探活 — 会被 --once 吃掉连接)."""
    proc = subprocess.Popen(
        [PY, os.path.join(HERE, 'fake_board.py'), '--port', str(PORT)] + list(extra),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    line = [None]
    t = threading.Thread(target=lambda: line.__setitem__(0, proc.stdout.readline()),
                         daemon=True)
    t.start()
    t.join(timeout=10)
    if line[0] is None or 'listening' not in line[0]:
        proc.kill()
        raise RuntimeError(f'fake_board never listened: {line[0]!r}')
    return proc


class TestProtocolRoundTrip(unittest.TestCase):
    def test_stream_to_fake_board(self):
        proc = start_fake_board('--once')
        try:
            s = Streamer('127.0.0.1', PORT, fps=50, loop=False)
            s.run(lambda: povstream.frame_iter_from_dir(_FRAMES_DIR))
            self.assertEqual(s.frames, 2)
            self.assertEqual(s.sent_raw, 2 * FRAME_RAW)
            self.assertGreater(s.ratio(), 2.0)          # zlib 至少 2x
            out, _ = proc.communicate(timeout=15)
        finally:
            if proc.poll() is None:
                proc.kill()
        self.assertEqual(proc.returncode, 0, out)
        # 逐帧 sha256 比对 (fake_board 打印前 16 hex)
        shas = [line.split('sha256=')[1].strip()
                for line in out.splitlines() if 'sha256=' in line]
        self.assertEqual(len(shas), 2, out)
        for i, f in enumerate(sorted(glob.glob(os.path.join(_FRAMES_DIR, 'frame_*.bin')))):
            want = hashlib.sha256(open(f, 'rb').read()).hexdigest()[:16]
            self.assertEqual(shas[i], want, f'frame {i} sha mismatch')

    def test_fake_board_rejects_short_frame(self):
        """坏帧 (raw_len 错) → NAK → StreamerError."""
        proc = start_fake_board('--once')
        try:
            s = Streamer('127.0.0.1', PORT, fps=50, loop=False, ack_timeout=5)
            with self.assertRaises(StreamerError):
                s.run(lambda: iter([b'\0' * 1024]))     # 假装一帧但 raw_len != FRAME_RAW
        finally:
            proc.kill()
            proc.wait()


class TestAutoReconnect(unittest.TestCase):
    def test_reconnect_resumes(self):
        proc = start_fake_board()
        s = Streamer('127.0.0.1', PORT, fps=20, loop=True, reconnect=True,
                     retry_interval=0.3, ack_timeout=3)
        th = threading.Thread(
            target=lambda: s.run(lambda: povstream.frame_iter_from_dir(_FRAMES_DIR)),
            daemon=True)
        th.start()
        try:
            deadline = time.time() + 15
            while s.frames < 3 and time.time() < deadline:
                time.sleep(0.05)
            self.assertGreaterEqual(s.frames, 3, 'never started streaming')

            # 模拟板重启: kill 服务器
            proc.kill()
            proc.wait()
            at_kill = s.frames
            time.sleep(1.0)                              # 让 streamer 撞断线

            proc = start_fake_board()                    # "板"回来了
            deadline = time.time() + 20
            while s.frames < at_kill + 3 and time.time() < deadline:
                time.sleep(0.05)
            self.assertGreaterEqual(s.frames, at_kill + 3, '重连后没续推')
            self.assertGreaterEqual(s.reconnects, 1, 'reconnects 没计数')
        finally:
            s.stop.set()
            th.join(timeout=10)
            self.assertFalse(th.is_alive(), 'streamer 线程没停')
            if proc.poll() is None:
                proc.kill()
                proc.wait()

    def test_no_reconnect_raises(self):
        """reconnect=False 时连不上直接 StreamerError (老 CLI 行为)."""
        s = Streamer('127.0.0.1', PORT + 7, fps=20, loop=False, reconnect=False)
        with self.assertRaises(StreamerError):
            s.run(lambda: povstream.frame_iter_from_dir(_FRAMES_DIR))


class TestConfig(unittest.TestCase):
    def test_roundtrip(self):
        p = os.path.join(_TMP, 'cfg.json')
        cfg = pov_studio.load_config(p)                  # 不存在 → 默认
        self.assertEqual(cfg['ip'], '10.10.20.234')
        self.assertEqual(cfg['frames'], 8)
        self.assertEqual(cfg['fps'], 4)
        self.assertTrue(cfg['loop'])
        self.assertTrue(cfg['reconnect'])
        cfg['ip'] = '10.10.21.99'
        cfg['preset'] = 'globe_spin'
        cfg['frames'] = 16
        pov_studio.save_config(cfg, p)
        cfg2 = pov_studio.load_config(p)
        self.assertEqual(cfg2, cfg)

    def test_corrupt_file_falls_back(self):
        p = os.path.join(_TMP, 'cfg_bad.json')
        with open(p, 'w') as f:
            f.write('{oops')
        cfg = pov_studio.load_config(p)
        self.assertEqual(cfg, pov_studio.DEFAULT_CONFIG | {})

    def test_unknown_keys_ignored(self):
        p = os.path.join(_TMP, 'cfg_extra.json')
        with open(p, 'w') as f:
            json.dump({'ip': '1.2.3.4', 'bogus': 1}, f)
        cfg = pov_studio.load_config(p)
        self.assertEqual(cfg['ip'], '1.2.3.4')
        self.assertNotIn('bogus', cfg)


class TestPreview(unittest.TestCase):
    def test_preview_from_packed_bin(self):
        import numpy as np
        fb = pov_studio.first_frame_bin(_FRAMES_DIR)
        self.assertIsNotNone(fb)
        im = pov_studio.preview_image(fb, slice_idx=0, scale=1)
        self.assertEqual(im.size, (160, 180))
        a = np.array(im)
        self.assertGreater(int((a > 0).sum()), 100, 'slice 0 预览全黑?')
        # scale=2
        im2 = pov_studio.preview_image(fb, slice_idx=0, scale=2)
        self.assertEqual(im2.size, (320, 360))

    def test_render_dir_reuse(self):
        """meta 匹配时 render_job 直接复用不重渲."""
        t0 = time.time()
        out = pov_studio.render_job('globe_spin', 'globe', 2, '', render_slices=8,
                                    out_dir=_FRAMES_DIR)
        self.assertEqual(out, _FRAMES_DIR)
        self.assertLess(time.time() - t0, 1.0, '复用路径不该重渲')


if __name__ == '__main__':
    unittest.main(verbosity=2)
