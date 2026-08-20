#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_idle_anim_3bit.py — 3-bit 冷启动通路的**离线**自检 (2026-08-20, 不碰板子).

覆盖三块, 每块都尽量做成"板端那条路的等价重放", 而不是重新发明一套算式:

  ① 容器 (tools/make_idle_anim.py)
     打 → 按 **pov_rxd.c idle_anim_load / idle_anim_step 的逐行等价实现**拆 →
     unpack_slice(bpp=3) 回读码值 → 与打进去的码值逐像素比。
     顺带钉死 1-bit 老路径**逐字节不变** (与 2026-08-03 版算法对拍)。

  ② 默认内容 (stream/board/anime_dual3b100.bin)
     帧长 / 片距 / 片数 / 面拆分点 / 码值域 / 两面确实不同。

  ③ 引导脚本 (stream/board/pov_boot.sh)
     把它第一段 python 原样抠出来, **换掉 mmap/os/open 后真的执行一遍**,
     BPP3=0 和 BPP3=1 各跑一次, 记录每一次寄存器写, 再逐条对账:
       0x0C sub10 的 oe_window / 0x0C sub01 的 oe_w1,oe_w2,bpp_mode /
       0x10 的 n_slices / 0x18 / 0x1C(PHASE_B) / 0x28(slice_base_b) /
       bank A 实际灌了多少字节。
     ⇒ "改了一半"(位序改了权重没改、N_SLICES 改了默认内容没改) 会在这里当场炸。

跑法:  python3 tools/test_idle_anim_3bit.py
只需要 numpy + zlib (WSL 的 python3 就够, 不需要 PIL/pygltflib)。
"""
import io
import os
import re
import sys
import zlib
import types
import struct
import shutil
import tempfile
import subprocess
import contextlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import pack_obs
import make_idle_anim as mia
from pack_obs import W, H

BOOT_SH = os.path.join(ROOT, 'stream', 'board', 'pov_boot.sh')
BOARD_DIR = os.path.join(ROOT, 'stream', 'board')
DEFAULT_3BIT = os.path.join(BOARD_DIR, 'anime_dual3b100.bin')

FLAG_ZLIB, FLAG_DUAL_FACE, FLAG_FOLD_A = 0x02, 0x08, 0x10
FLAG_MSTREAM, FLAG_3BIT = 0x40, 0x80
FRAME_RAW_MAX = 8847360

_fails = []
_n_ok = 0


def check(cond, what, extra=''):
    global _n_ok
    if cond:
        _n_ok += 1
        print(f'  [ok] {what}' + (f'  {extra}' if extra else ''))
    else:
        _fails.append(what)
        print(f'  [FAIL] {what}' + (f'  {extra}' if extra else ''))


def eq(got, want, what):
    check(got == want, what, f'got={got!r} want={want!r}')


# ===========================================================================
# pov_rxd.c 的等价重放: idle_anim_load + idle_anim_step
# ===========================================================================
def emu_load(blob):
    """= idle_anim_load()。返回 (n_frames, n_slices, flags, idx) 或抛 AssertionError。"""
    assert len(blob) >= 16, 'container < 16B'
    assert blob[:4] == b'PVSA', 'magic 不对'
    n, n_slices, flags = struct.unpack('<III', blob[4:16])
    stride = 0x9000 if (flags & FLAG_3BIT) else 0x3000
    n_max = FRAME_RAW_MAX // stride                      # PVS_N_SLICES_MAX_F(flags)
    assert n, 'n_frames = 0'
    assert n_slices <= n_max, f'n_slices={n_slices} > {n_max}'
    assert not (flags & FLAG_MSTREAM), 'idle 路径不吃 MSTREAM'
    idx = struct.unpack(f'<{2 * n}I', blob[16:16 + 8 * n])
    return n, n_slices, flags, idx


def emu_step(blob, n_slices, flags, idx, k):
    """= idle_anim_step() 的解码部分。返回 (raw, stride, fbo)。"""
    off, ln = idx[2 * k], idx[2 * k + 1]
    assert off + ln <= len(blob), 'off+len 越界 (板端会重头播)'
    stride = 0x9000 if (flags & FLAG_3BIT) else 0x3000
    raw_len = n_slices * stride
    n_a = 0
    if flags & FLAG_DUAL_FACE:
        div = 3 if (flags & FLAG_FOLD_A) else 2
        assert n_slices % div == 0, '容器不自洽 → 板端停播'
        n_a = n_slices // div
    fbo = n_a * stride
    p = blob[off:off + ln]
    if fbo:
        clen_a = struct.unpack('<I', p[:4])[0]
        p = p[4:]
        assert 0 < clen_a < len(p), f'clen_a={clen_a} 不自洽 → 板端每帧静默 return'
        a = zlib.decompress(p[:clen_a])
        b = zlib.decompress(p[clen_a:])
        assert len(a) == fbo, f'面A {len(a)} != {fbo}'
        assert len(b) == raw_len - fbo, f'面B {len(b)} != {raw_len - fbo}'
        raw = a + b
    else:
        raw = zlib.decompress(p)
    assert len(raw) == raw_len, f'raw {len(raw)} != {raw_len}'
    return raw, stride, fbo


# ===========================================================================
# ① 容器往返
# ===========================================================================
def synth_codes(seed, n):
    """n 张互不相同的码值图 (180,160,3) 0..7, 确定性。"""
    ys, xs = np.mgrid[0:H, 0:W]
    out = []
    for i in range(n):
        c = np.stack([(xs // 7 + ys // 11 + i * 3 + ch * 5 + seed) % 8
                      for ch in range(3)], axis=2).astype(np.uint8)
        out.append(c)
    return out


def run_mia(args):
    r = subprocess.run([sys.executable, os.path.join(HERE, 'make_idle_anim.py')] + args,
                       capture_output=True, text=True)
    return r


def test_container_3bit(tmp):
    print('\n① 3-bit 双面容器往返 (打包 → pov_rxd 等价解码 → unpack_slice(bpp=3))')
    n_face, n_frames = 50, 3
    src = os.path.join(tmp, 'frames3b')
    os.makedirs(src)
    all_codes = []
    for fi in range(n_frames):
        codes = synth_codes(fi * 17, 2 * n_face)          # 面A 50 + 面B 50
        all_codes.append(codes)
        raw = b''.join(pack_obs.pack_slice(c, bpp=3) for c in codes)
        eq(len(raw), 2 * n_face * 0x9000, f'帧{fi} raw 长度')
        open(os.path.join(src, f'frame_{fi:04d}.bin'), 'wb').write(raw)
    open(os.path.join(src, 'meta.json'), 'w').write(
        '{"bpp": 3, "n_slices": 100, "geom_flags": 8}')

    dst = os.path.join(tmp, 'anim3b.pvs')
    r = run_mia([src, dst])
    check(r.returncode == 0, 'make_idle_anim 退出码 0', r.stderr.strip()[-200:])
    if r.returncode:
        return
    print('   ' + r.stdout.strip().replace('\n', '\n   '))

    blob = open(dst, 'rb').read()
    n, n_slices, flags, idx = emu_load(blob)
    eq(n, n_frames, '容器帧数')
    eq(n_slices, 2 * n_face, '容器 n_slices')
    eq(hex(flags), hex(FLAG_ZLIB | FLAG_DUAL_FACE | FLAG_3BIT), 'flags = ZLIB|DUAL_FACE|3BIT')
    check(bool(flags & FLAG_3BIT), 'PVS_FLAG_3BIT (1<<7) 已置位 → 板端片距 0x9000')
    check(not (flags & FLAG_MSTREAM), '没有 MSTREAM (板端 idle 路径不支持)')
    # 索引表连续且落在文件内
    exp_off = 16 + 8 * n
    okidx = True
    for k in range(n):
        okidx &= idx[2 * k] == exp_off and idx[2 * k] + idx[2 * k + 1] <= len(blob)
        exp_off += idx[2 * k + 1]
    check(okidx and exp_off == len(blob), '(off,len) 表连续且正好覆盖到文件末尾')

    for k in range(n):
        raw, stride, fbo = emu_step(blob, n_slices, flags, idx, k)
        eq(stride, 0x9000, f'帧{k} 片距')
        eq(fbo, n_face * 0x9000, f'帧{k} 面拆分点 (= 板端 slice_base_b 偏移)')
        bad = 0
        for i in range(n_slices):
            back = pack_obs.unpack_slice(raw[i * stride:(i + 1) * stride], bpp=3)
            if not np.array_equal(back, all_codes[k][i]):
                bad += 1
        eq(bad, 0, f'帧{k} 全 {n_slices} 片码值逐像素回读一致')


def test_container_1bit_regression(tmp):
    print('\n①-b 1-bit 老路径回归 (容器必须与 2026-08-03 版逐字节相同)')
    n_slices = 720
    src = os.path.join(tmp, 'frames1b')
    os.makedirs(src)
    rng = np.random.default_rng(7)
    raw = bytearray()
    for i in range(n_slices):                     # 随机但可压缩的 1-bit 片
        on = rng.random((H, W, 3)) < 0.02
        raw += pack_obs.pack_slice(on) + b'\0' * (0x3000 - pack_obs.SLICE_DATA)
    raw = bytes(raw)
    eq(len(raw), n_slices * 0x3000, '1-bit 720 片帧长')
    open(os.path.join(src, 'frame_0000.bin'), 'wb').write(raw)   # 无 meta.json = 反推

    dst = os.path.join(tmp, 'anim1b.pvs')
    r = run_mia([src, dst])
    check(r.returncode == 0, 'make_idle_anim 退出码 0', r.stderr.strip()[-200:])
    blob = open(dst, 'rb').read()

    # 2026-08-03 版算法, 原样重写一遍作对拍基准
    fbo = 360 * 0x3000
    a = zlib.compress(raw[:fbo], 6)
    b = zlib.compress(raw[fbo:], 6)
    pay = struct.pack('<I', len(a)) + a + b
    ref = (b'PVSA' + struct.pack('<III', 1, 720, FLAG_ZLIB | FLAG_DUAL_FACE)
           + struct.pack('<II', 24, len(pay)) + pay)
    check(blob == ref, '1-bit 720 片容器与老算法逐字节相同',
          f'{len(blob)}B vs {len(ref)}B')

    n, n_slices2, flags, idx = emu_load(blob)
    eq(hex(flags), hex(FLAG_ZLIB | FLAG_DUAL_FACE), '1-bit flags 不含 3BIT')
    got, stride, fbo2 = emu_step(blob, n_slices2, flags, idx, 0)
    eq(stride, 0x3000, '1-bit 片距')
    check(got == raw, '1-bit 解码回来与原帧逐字节相同')


def test_guards(tmp):
    print('\n① -c 防呆: 该拒的必须拒 (静默丢帧是这条链上最贵的失败模式)')
    d = os.path.join(tmp, 'g1')
    os.makedirs(d)
    # 3686400B: 300 片 1-bit 还是 100 片 3-bit? 没 meta / 没 --bpp 必须拒绝
    open(os.path.join(d, 'frame_0000.bin'), 'wb').write(b'\0' * 3686400)
    r = run_mia([d, os.path.join(tmp, 'x.pvs')])
    check(r.returncode != 0 and '不认识的片数' in (r.stdout + r.stderr),
          '3686400B 且无 meta/无 --bpp → 拒绝 (片距歧义)')
    # 给了 --bpp 3 --dual-face 就该过
    r = run_mia([d, os.path.join(tmp, 'x.pvs'), '--bpp', '3', '--dual-face'])
    check(r.returncode == 0, '同一份数据 + --bpp 3 --dual-face → 通过')
    # meta 与帧长打架
    d2 = os.path.join(tmp, 'g2')
    os.makedirs(d2)
    open(os.path.join(d2, 'frame_0000.bin'), 'wb').write(b'\0' * 3686400)
    open(os.path.join(d2, 'meta.json'), 'w').write('{"bpp":3,"n_slices":120,"geom_flags":8}')
    r = run_mia([d2, os.path.join(tmp, 'y.pvs')])
    check(r.returncode != 0 and 'meta.json 说' in (r.stdout + r.stderr),
          'meta.json n_slices 与帧长不符 → 拒绝')
    # 双面但片数是奇数
    d3 = os.path.join(tmp, 'g3')
    os.makedirs(d3)
    open(os.path.join(d3, 'frame_0000.bin'), 'wb').write(b'\0' * (99 * 0x9000))
    r = run_mia([d3, os.path.join(tmp, 'z.pvs'), '--bpp', '3', '--dual-face'])
    check(r.returncode != 0, '双面 + 奇数片 → 拒绝 (板端会当场停播)')
    # 3-bit 超过 240 片上限
    d4 = os.path.join(tmp, 'g4')
    os.makedirs(d4)
    open(os.path.join(d4, 'frame_0000.bin'), 'wb').write(b'\0' * (242 * 0x9000))
    r = run_mia([d4, os.path.join(tmp, 'w.pvs'), '--bpp', '3', '--dual-face'])
    check(r.returncode != 0, '3-bit 242 片 (> 240 上限) → 拒绝')


# ===========================================================================
# ② 默认内容
# ===========================================================================
def test_default_bin(tmp):
    print('\n② 冷启动默认内容 stream/board/anime_dual3b100.bin')
    if not os.path.exists(DEFAULT_3BIT):
        check(False, 'anime_dual3b100.bin 存在')
        return None
    raw = open(DEFAULT_3BIT, 'rb').read()
    n_face = 50
    eq(len(raw), 2 * n_face * 0x9000, '帧长 = (50+50) × 0x9000')
    check(len(raw) <= FRAME_RAW_MAX, '装得进一个 DDR bank',
          f'{len(raw)} <= {FRAME_RAW_MAX} ({100.0*len(raw)/FRAME_RAW_MAX:.0f}%)')
    fbo = n_face * 0x9000
    eq(hex(fbo), hex(0x1C2000), '面拆分点 = 50*0x9000')

    # 每片 3 个 plane, plane 内 11664B 有效 + 624B padding 必须是 0
    padbad = sum(1 for i in range(2 * n_face) for p in range(3)
                 if any(raw[i * 0x9000 + p * 0x3000 + pack_obs.SLICE_DATA:
                            i * 0x9000 + (p + 1) * 0x3000]))
    eq(padbad, 0, '全部 300 个 plane 的 624B padding 都是 0')

    lit_a = lit_b = 0
    codes_seen = set()
    diff = 0
    for i in range(0, n_face, 5):
        ca = pack_obs.unpack_slice(raw[i * 0x9000:(i + 1) * 0x9000], bpp=3)
        j = n_face + i
        cb = pack_obs.unpack_slice(raw[j * 0x9000:(j + 1) * 0x9000], bpp=3)
        codes_seen |= set(np.unique(ca).tolist()) | set(np.unique(cb).tolist())
        lit_a += int((ca > 0).sum()); lit_b += int((cb > 0).sum())
        diff += int((ca != cb).sum())
    check(max(codes_seen) <= 7 and min(codes_seen) >= 0, '码值域 0..7', f'{sorted(codes_seen)}')
    eq(sorted(codes_seen), list(range(8)), '八级码值全部出现过 (真在用 3-bit)')
    check(lit_a > 0 and lit_b > 0, '两面都有点亮像素', f'A={lit_a} B={lit_b}')
    check(diff > 0, '面A 与面B 数据不同 (v3.1 偏心屏两面必须各渲一份)')
    return raw


def test_default_bin_container(tmp, raw):
    print('\n② -b 默认内容打成容器再拆回来 (给 --idle-anim 用的那条路)')
    if raw is None:
        return
    dst = os.path.join(tmp, 'boot3b.pvs')
    r = run_mia([DEFAULT_3BIT, dst, '--bpp', '3', '--dual-face'])
    check(r.returncode == 0, 'make_idle_anim 吃单个 .bin', r.stderr.strip()[-200:])
    if r.returncode:
        return
    print('   ' + r.stdout.strip().replace('\n', '\n   '))
    blob = open(dst, 'rb').read()
    n, n_slices, flags, idx = emu_load(blob)
    eq(n_slices, 100, '容器 n_slices')
    eq(hex(flags), hex(FLAG_ZLIB | FLAG_DUAL_FACE | FLAG_3BIT), 'flags')
    got, stride, fbo = emu_step(blob, n_slices, flags, idx, 0)
    check(got == raw, '解码回来与 .bin 逐字节相同')
    check(len(blob) < len(raw) // 4, 'zlib 压得动',
          f'{len(blob)} / {len(raw)} = {100.0*len(blob)/len(raw):.1f}%')


# ===========================================================================
# ③ pov_boot.sh 对账 (真的执行它, 只把 mmap/os/open 换掉)
# ===========================================================================
class FakeMap:
    """mmap.mmap 的替身: 记录所有 [off:off+4] = 4B 的写 (寄存器页) 与整段写 (bank)。"""
    def __init__(self, length, offset):
        self.offset, self.length = offset, length
        self.writes = []            # 寄存器页: [(off, u32), ...]
        self.filled = 0             # bank: 写进来的字节数 (非零段)
        self.data = bytearray(length) if length <= 0x900000 else None

    def __setitem__(self, k, v):
        if isinstance(k, slice):
            start = k.start or 0
            if self.length <= 4096 and len(v) == 4:
                self.writes.append((start, int.from_bytes(v, 'little')))
            else:
                if self.data is not None:
                    self.data[start:start + len(v)] = v
                if any(v):
                    self.filled = max(self.filled, start + len(v))
        else:
            raise AssertionError('unexpected scalar write')

    def close(self):
        pass


def run_boot_block(bpp3, redirect):
    """抠出 pov_boot.sh 第一段 python, 把 BPP3 改成 bpp3, 执行, 返回 (maps, stdout)。"""
    sh = open(BOOT_SH, encoding='utf-8').read()
    blocks = re.findall(r"python3 - <<'PY'\n(.*?)\nPY\n", sh, re.S)
    assert blocks, 'pov_boot.sh 里没找到 python heredoc'
    src = blocks[0]
    src, n = re.subn(r'^BPP3 = \d+', f'BPP3 = {bpp3}', src, count=1, flags=re.M)
    assert n == 1, '没找到 BPP3 那一行'

    maps = []

    def fake_mmap(fd, length, offset=0):
        m = FakeMap(length, offset)
        maps.append(m)
        return m

    mmap_mod = types.ModuleType('mmap')
    mmap_mod.mmap = fake_mmap
    os_mod = types.ModuleType('os')
    os_mod.open = lambda *a, **k: 3
    os_mod.O_RDWR = os_mod.O_SYNC = 0

    real_open = open

    def my_open(path, *a, **k):
        return real_open(redirect.get(path, path), *a, **k)

    saved = {k: sys.modules.get(k) for k in ('mmap', 'os')}
    sys.modules['mmap'], sys.modules['os'] = mmap_mod, os_mod
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            exec(compile(src, BOOT_SH, 'exec'), {'open': my_open, '__name__': '__main__'})
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return maps, buf.getvalue()


def regs_of(maps):
    reg = [m for m in maps if m.length == 4096][0]
    last, seq = {}, []
    for off, v in reg.writes:
        last[off] = v
        seq.append((off, v))
    return last, seq, reg


def test_boot(tmp):
    print('\n③ pov_boot.sh 逐项对账 (抠出 python 段真执行, mmap/os/open 换成假的)')
    # 板上 /home/uisrc/*.bin → 仓库里的同名文件
    redirect = {'/home/uisrc/anime_dual3b100.bin': DEFAULT_3BIT,
                '/home/uisrc/anime_dual720.bin': os.path.join(BOARD_DIR, 'anime_dual720.bin')}

    for bpp3, want in ((0, dict(n=360, stride=0x3000, oe0=111, bpp=0)),
                       (1, dict(n=50, stride=0x9000, oe0=184, bpp=1))):
        print(f'  --- BPP3={bpp3} ---')
        try:
            maps, out = run_boot_block(bpp3, redirect)
        except AssertionError as e:
            check(False, f'BPP3={bpp3} 执行 pov_boot.sh 的 python 段', str(e))
            continue
        last, seq, reg = regs_of(maps)
        n, stride, oe0 = want['n'], want['stride'], want['oe0']
        BANK_A, BANK_BYTES = 0x10000000, 0x870000

        eq(last.get(0x10), (n << 16) | 0x5, f'0x10 POV_CTRL = n_slices({n})<<16 | dual_en|pov_en')
        eq(last.get(0x1C), n // 2, f'0x1C PHASE_B = 半圈 ({n}//2)')
        check(last.get(0x1C, 1 << 30) < n, '🔴 PHASE_B < 引擎每圈片数 (否则屏B 索引越界读野地址)',
              f'{last.get(0x1C)} < {n}')
        eq(last.get(0x18), BANK_A, '0x18 slice_base = bank A')
        eq(last.get(0x28), BANK_A + n * stride, f'0x28 slice_base_b = bank A + {n}*0x{stride:X}')

        # 0x0C 的四次写: 复位 / sub10(CFG_MISC) / 0xC1000003 / sub01(BCM)
        c = [v for o, v in seq if o == 0x0C]
        eq(len(c), 4, '0x0C 一共写 4 次 (复位 / sub10 / 0xC1000003 / sub01)')
        cfg = c[1]
        eq(cfg >> 30, 0b10, 'CFG_MISC 是 subcmd=10')
        eq((cfg >> 8) & 0xFF, oe0, f'CFG_MISC[15:8] oe_window = oe_w0 = {oe0}')
        eq((cfg >> 29) & 1, 0, 'bit29 ddr_slow = 0 (fast; 回滚逃生门是改回 1)')
        sub01 = c[3]
        eq(sub01 >> 30, 0b01, '最后一次 0x0C 是 subcmd=01 (BCM)')
        oe1, oe2 = sub01 & 0xFF, (sub01 >> 8) & 0xFF
        eq((oe1, oe2), (92, 46), 'sub01 oe_w1 / oe_w2')
        eq((sub01 >> 16) & 1, want['bpp'], 'sub01 bpp_mode')

        # 🔴 位序 × 权重 必须成对: plane0=MSB 且 4:2:1, 否则灰阶非单调
        wgt = {0: oe0, 1: oe1, 2: oe2}                 # plane p 的 OE 沿数
        bits = {0: 2, 1: 1, 2: 0}                      # pack_obs: plane p 装 bit(2-p)
        edges = [sum(wgt[p] for p in range(3) if (code >> bits[p]) & 1) for code in range(8)]
        if bpp3:
            eq((oe0, oe1, oe2), (184, 92, 46), '权重 = 184/92/46 (精确 4:2:1)')
            check(all(edges[i] < edges[i + 1] for i in range(7)),
                  '🔴 码值 0..7 → OE 沿数**严格单调递增**', str(edges))
            check(edges == [46 * k for k in range(8)],
                  '码值与发光时间严格成正比 (线性, gamma 全在 host 侧)', str(edges))
            check(oe1 <= 187 and oe2 <= 111,
                  'plane2 (最后一个 plane) 的权重 <= 111 → 不插 LWAIT',
                  f'oe_w2={oe2}')

        # bank A 的默认内容
        bank = [m for m in maps if m.offset == BANK_A][0]
        want_bytes = 2 * n * stride if bpp3 else 8847360
        got_bytes = None
        for line in out.splitlines():
            if line.startswith('bank A <- default'):
                got_bytes = int(line.split()[-1])
        check('no default anime' not in out, '默认内容文件找得到且读得进来',
              out.strip().replace('\n', ' | '))
        eq(got_bytes, want_bytes, f'灌进 bank A 的字节数 = {"(50+50)" if bpp3 else "720"} 片')
        check(want_bytes <= BANK_BYTES, 'bank 装得下', f'{want_bytes} <= {BANK_BYTES}')
        if bpp3 and bank.data is not None:
            raw = open(DEFAULT_3BIT, 'rb').read()
            fbo = n * stride
            check(bytes(bank.data[:len(raw)]) == raw, 'bank A 内容 = .bin 逐字节')
            check(bytes(bank.data[fbo:fbo + 0x9000]) == raw[fbo:fbo + 0x9000],
                  f'0x28 指向的那一片 (bank+0x{fbo:X}) 正是面B 第 0 片')
            check(not any(bank.data[len(raw):]), '载荷之后全部清零 (不留上电垃圾)')
        print('   boot 日志: ' + ' | '.join(l for l in out.splitlines() if l))


def test_boot_cross(tmp):
    print('\n③ -b 与 pov_rxd.c / povmem 的交叉一致性')
    sh = open(BOOT_SH, encoding='utf-8').read()
    rxd = open(os.path.join(BOARD_DIR, 'pov_rxd.c'), encoding='utf-8').read()

    m = re.search(r'#define OE_W1_DEFAULT\s+(\d+)u', rxd)
    m2 = re.search(r'#define OE_W2_DEFAULT\s+(\d+)u', rxd)
    m3 = re.search(r'#define OE_W0_3BIT_HINT\s+(\d+)u', rxd)
    b = re.search(r'^OE_W1, OE_W2 = (\d+), (\d+)', sh, re.M)
    b0 = re.search(r'^OE_W0 = (\d+) if BPP3', sh, re.M)
    check(m and b and (int(m.group(1)), int(m2.group(1))) == (int(b.group(1)), int(b.group(2))),
          'pov_boot.sh 的 oe_w1/oe_w2 == pov_rxd.c 的 OE_W*_DEFAULT',
          f'boot=({b.group(1)},{b.group(2)}) rxd=({m.group(1)},{m2.group(1)})')
    check(m3 and b0 and int(m3.group(1)) == int(b0.group(1)),
          'pov_boot.sh 的 3-bit oe_w0 == pov_rxd.c 的 OE_W0_3BIT_HINT',
          f'boot={b0.group(1)} rxd={m3.group(1)}')

    check('n_eng / 2u' in rxd or 'n_eng / 2' in rxd,
          'pov_rxd.c 的 flip 线程用 n_eng/2 而不是写死 180 (推流时同样不越界)')

    # povmem 映射窗 >= 三缓冲所需
    mm = re.search(r'povmem\.ko base=0x([0-9A-Fa-f]+) size=0x([0-9A-Fa-f]+)', sh)
    size = int(mm.group(2), 16)
    need = 2 * 0x1000000 + 0x870000
    check(size >= need, 'povmem 映射窗 >= 三缓冲所需 0x2870000',
          f'0x{size:X} >= 0x{need:X}')

    # 默认内容路径与实际生成的文件名一致
    paths = re.findall(r"'(/home/uisrc/[A-Za-z0-9_.]+\.bin)'", sh)
    for p in paths:
        local = os.path.join(BOARD_DIR, os.path.basename(p))
        check(os.path.exists(local), f'{p} 在仓库里有对应文件 ({os.path.basename(p)})')


def main():
    tmp = tempfile.mkdtemp(prefix='idle3b_')
    try:
        test_container_3bit(tmp)
        test_container_1bit_regression(tmp)
        test_guards(tmp)
        raw = test_default_bin(tmp)
        test_default_bin_container(tmp, raw)
        test_boot(tmp)
        test_boot_cross(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print(f'\n==== {_n_ok} 项通过, {len(_fails)} 项失败 ====')
    for f in _fails:
        print('  FAIL:', f)
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
