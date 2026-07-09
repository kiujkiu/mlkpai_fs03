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
5. GLB 动画源: render_args glbanim/glbseq 参数映射 + take 入 hash +
   glb_take_names + spincube glbanim/glbseq mini 全管线渲染.
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
    os.environ['POVSTREAM_CACHE'] = os.path.join(_TMP, 'cache')  # 不污染 pc/cache
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


def _spincube_glb():
    """test_assets/spincube.glb (带 'spin' take 的动画立方体), 缺则现做."""
    p = os.path.join(HERE, 'test_assets', 'spincube.glb')
    if not os.path.exists(p):
        sys.path.insert(0, os.path.join(HERE, 'test_assets'))
        import make_test_glb as mk
        mk.make_spincube(p)
    return p


class TestGlbSources(unittest.TestCase):
    """GLB自带动画 (glbanim) + GLB 序列目录 (glbseq) 接线."""

    def test_render_args_glbanim(self):
        a = pov_studio.render_args('glb_anim', 4, '/tmp/x.glb', 24,
                                   anim_take='wave')
        self.assertEqual(a.anim, 'glbanim')
        self.assertEqual(a.frames, 4)
        self.assertEqual(a.glb, '/tmp/x.glb')
        self.assertEqual(a.anim_take, 'wave')
        self.assertEqual(a.render_slices, 24)
        # 空 take 回默认 '0'
        self.assertEqual(pov_studio.render_args('glb_anim', 2, '', 24,
                                                anim_take='  ').anim_take, '0')

    def test_render_args_glbseq_frames_from_dir(self):
        d = os.path.join(_TMP, 'seq_args')
        os.makedirs(d, exist_ok=True)
        for n in ('a.glb', 'b.glb'):
            open(os.path.join(d, n), 'wb').close()
        a = pov_studio.render_args('glbseq', 99, '', 24, glb_dir=d)
        self.assertEqual(a.anim, 'glbseq')
        self.assertEqual(a.glb_dir, d)
        self.assertEqual(a.frames, 2, '帧数应 = 目录 *.glb 文件数')
        self.assertEqual(pov_studio.glb_seq_count(d), 2)
        self.assertEqual(pov_studio.glb_seq_count(''), 0)
        self.assertEqual(pov_studio.glb_seq_count(d + '_nope'), 0)

    def test_render_args_static_unchanged(self):
        a = pov_studio.render_args('static', 8, '', 24)
        self.assertEqual((a.frames, a.anim), (1, 'spinpulse'))

    def test_render_dir_name_take_in_hash(self):
        d0 = pov_studio.render_dir_name('glb_anim', 'glb', 2, 'a.glb', 24,
                                        anim_take='0')
        d1 = pov_studio.render_dir_name('glb_anim', 'glb', 2, 'a.glb', 24,
                                        anim_take='1')
        self.assertNotEqual(d0, d1, 'anim_take 必须参与 hash')
        self.assertTrue(os.path.basename(d0).startswith(
            'frames_studio_a_glb_anim_2f_'), d0)

    def test_glb_take_names(self):
        self.assertEqual(pov_studio.glb_take_names(_spincube_glb()), ['spin'])
        sys.path.insert(0, os.path.join(HERE, 'test_assets'))
        import make_test_glb as mk
        static = os.path.join(_TMP, 'static_cube.glb')
        mk.make_offset_cube(static, (0.0, 0.0, 0.0), (255, 0, 0))
        self.assertEqual(pov_studio.glb_take_names(static), [],
                         '无动画 GLB take 列表应为空')

    def test_glbanim_mini_render_two_frames_differ(self):
        out = os.path.join(_TMP, 'render_glbanim')
        d = pov_studio.render_job('glb_anim', 'glb', 2, _spincube_glb(),
                                  render_slices=24, out_dir=out)
        bins = sorted(glob.glob(os.path.join(d, 'frame_*.bin')))
        self.assertEqual(len(bins), 2)
        datas = [open(b, 'rb').read() for b in bins]
        for x in datas:
            self.assertEqual(len(x), FRAME_RAW)         # 4,423,680
        self.assertNotEqual(datas[0], datas[1], 'glbanim 两帧应不同 (立方体在转)')
        meta = json.load(open(os.path.join(d, 'meta.json')))
        self.assertEqual(meta['anim_take'], '0')

    def test_glbseq_mini_render_two_files(self):
        seq = os.path.join(_TMP, 'seq_render')
        os.makedirs(seq, exist_ok=True)
        for n in ('f000.glb', 'f001.glb'):
            shutil.copy(_spincube_glb(), os.path.join(seq, n))
        out = os.path.join(_TMP, 'render_glbseq')
        d = pov_studio.render_job('glbseq', 'glbdir', 0, '', render_slices=24,
                                  out_dir=out, glb_dir=seq)
        bins = sorted(glob.glob(os.path.join(d, 'frame_*.bin')))
        self.assertEqual(len(bins), 2, '帧数 = 序列目录文件数')
        for b in bins:
            self.assertEqual(os.path.getsize(b), FRAME_RAW)


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
