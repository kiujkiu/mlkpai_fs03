#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_pack_obs.py — pack_obs 打包/解包自检 (1-bit 与 3-bit 都要过).

  python3 tools/test_pack_obs.py            # 直接跑, 只依赖 numpy
  python3 -m pytest tools/test_pack_obs.py  # 有 pytest 也能跑

覆盖:
  ① 随机图 pack → unpack → 逐像素比对码值 (1-bit bool / 3-bit 0..7)
  ② 与**独立写的朴素参考实现**对拍 (直接照 pack_obs 文件头的映射规则逐像素写,
     不共用任何向量化代码) —— 防的是"向量化和文档一起错"
  ③ 片内布局: plane p 必须落在 p*0x3000, plane 内部与 1-bit 逐字节相同,
     每 plane 尾部 624B 补零
  ④ 1-bit 黄金哈希: 固定种子的输出 sha256 必须等于 3-bit 改造**之前**的值
     (2026-08-20 在 feature/3bit-color 改动前实测录下), 这是回归硬判据
  ⑤ 3-bit 量化器 (gen_anime_slices.to_3bit): 单调 / 无偏 / 相位随槽变
     (需要 PIL, 缺了自动跳过)
"""
import os
import sys
import hashlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pack_obs
from pack_obs import W, H, LANES, ROWS, WORDS, SLICE_DATA, PLANE_STRIDE

# ---- ④ 改造前 (1-bit 时代) 的输出哈希, 固定种子 np.random.RandomState(20260820) ----
GOLD_1BIT_U8 = '69edfe2e0f9ec5eab47de2f68dbaa902118f00d19aec192e4be017d114837ac5'
GOLD_1BIT_BOOL = '4926b9e503b64f00d2c2e421009dd287d91fc5b2afe50c4a8fce8b66b468c621'


# ================= 朴素参考实现 (照文件头规则逐像素写, 不共用向量化代码) =================

def ref_lane_row(X):
    for x0, w, base in ((0, 53, 6), (53, 53, 3), (106, 54, 0)):
        if x0 <= X < x0 + w:
            return base, (w - 1) - (X - x0)
    raise ValueError(X)


def ref_pack_plane(on):
    """on (180,160,3) bool → 11664B。偏移 = lane*1296 + row*24 + word*4, LE32。"""
    buf = bytearray(SLICE_DATA)
    for Y in range(H):
        h = 11 - Y // 15
        word, bit = h // 2, (h % 2) * 16 + Y % 15
        for X in range(W):
            base, row = ref_lane_row(X)
            for c in range(3):
                if on[Y, X, c]:
                    off = (base + c) * (ROWS * WORDS * 4) + row * (WORDS * 4) + word * 4
                    v = int.from_bytes(buf[off:off + 4], 'little') | (1 << bit)
                    buf[off:off + 4] = v.to_bytes(4, 'little')
    return bytes(buf)


def ref_pack_slice(img, bpp):
    if bpp == 1:
        return ref_pack_plane(np.asarray(img) >= 128 if img.dtype != np.bool_
                              else np.asarray(img))
    code = np.asarray(img, np.uint8)
    pad = b'\0' * (PLANE_STRIDE - SLICE_DATA)
    return b''.join(ref_pack_plane((code >> p) & 1 != 0) + pad for p in range(3))


# ================= 测试 =================

def _rs(seed=20260820):
    return np.random.RandomState(seed)


def test_stride_constants():
    assert pack_obs.slice_stride(1) == 0x3000
    assert pack_obs.slice_stride(3) == 0x9000 == pack_obs.SLICE_STRIDE_3BIT
    assert pack_obs.SLICE_DATA == 11664 and pack_obs.PLANE_STRIDE == 0x3000
    assert pack_obs.SLICE_STRIDE == 0x3000, '1-bit 兼容常量不许动'
    try:
        pack_obs.slice_stride(2)
    except ValueError:
        pass
    else:
        raise AssertionError('bpp=2 应当报错')
    print('[ok] ① 常量/片距: 1-bit 0x3000, 3-bit 0x9000')


def test_roundtrip_1bit():
    rs = _rs(1)
    for k in range(3):
        img = rs.rand(H, W, 3) > 0.5
        buf = pack_obs.pack_slice(img)
        assert len(buf) == SLICE_DATA
        assert len(pack_obs.pack_slice(img, pad=True)) == PLANE_STRIDE
        back = pack_obs.unpack_slice(buf)
        assert back.dtype == np.bool_ and back.shape == (H, W, 3)
        assert np.array_equal(back, img), f'1-bit 回环 seed {k} 不一致'
    print(f'[ok] ② 1-bit 随机图 pack→unpack 逐像素一致 (3 张 {H}x{W}x3)')


def test_roundtrip_3bit():
    rs = _rs(2)
    for k in range(3):
        img = rs.randint(0, 8, (H, W, 3)).astype(np.uint8)
        buf = pack_obs.pack_slice(img, bpp=3)
        assert len(buf) == 0x9000, len(buf)
        back = pack_obs.unpack_slice(buf, bpp=3)
        assert back.dtype == np.uint8 and back.shape == (H, W, 3)
        bad = int((back != img).sum())
        assert bad == 0, f'3-bit 回环 seed {k}: {bad} 个像素码值不符'
    # 边界: 全 0 / 全 7
    for v in (0, 7):
        img = np.full((H, W, 3), v, np.uint8)
        assert np.array_equal(pack_obs.unpack_slice(
            pack_obs.pack_slice(img, bpp=3), bpp=3), img)
    print(f'[ok] ② 3-bit 随机图 pack→unpack 逐像素码值一致 (3 张 + 全0/全7)')


def test_code_range_guard():
    img = np.zeros((H, W, 3), np.uint8)
    img[0, 0, 0] = 8
    try:
        pack_obs.pack_slice(img, bpp=3)
    except ValueError as e:
        assert '0..7' in str(e)
    else:
        raise AssertionError('码值 8 应当报错 (会静默丢最高位)')
    print('[ok] ② 3-bit 码值越界 (>7) 当场报错')


def test_vs_reference_impl():
    rs = _rs(3)
    b = rs.rand(H, W, 3) > 0.7
    assert pack_obs.pack_slice(b) == ref_pack_plane(b), '1-bit 与朴素参考实现不符'
    code = rs.randint(0, 8, (H, W, 3)).astype(np.uint8)
    assert pack_obs.pack_slice(code, bpp=3) == ref_pack_slice(code, 3), \
        '3-bit 与朴素参考实现不符'
    print('[ok] ② 与逐像素朴素参考实现对拍一致 (1-bit / 3-bit)')


def test_plane_layout():
    """3-bit 片 = 三个 plane 顺序排列, plane p 在 p*0x3000, 内部布局与 1-bit 同。"""
    rs = _rs(4)
    code = rs.randint(0, 8, (H, W, 3)).astype(np.uint8)
    buf = pack_obs.pack_slice(code, bpp=3)
    for p in range(3):
        off = p * PLANE_STRIDE
        want = pack_obs.pack_slice((code >> p) & 1 != 0)      # 走 1-bit 路径
        assert buf[off:off + SLICE_DATA] == want, f'plane {p} 数据不在 {off:#x}'
        assert buf[off + SLICE_DATA:off + PLANE_STRIDE] == b'\0' * 624, \
            f'plane {p} 尾部 624B 未补零'
    # 纯 1-bit 内容用 3-bit 送 (码值 0/7) ⇒ 三个 plane 完全相同, 且各自等于 1-bit 输出
    on = rs.rand(H, W, 3) > 0.5
    b3 = pack_obs.pack_slice((on * 7).astype(np.uint8), bpp=3)
    b1 = pack_obs.pack_slice(on)
    for p in range(3):
        assert b3[p * PLANE_STRIDE:p * PLANE_STRIDE + SLICE_DATA] == b1
    print('[ok] ③ 片内布局: plane p @ p*0x3000, plane 内逐字节 = 1-bit, 尾部补零')


def test_golden_1bit_regression():
    """1-bit 输出必须与 3-bit 改造之前**逐字节相同** (今天的空闲动画/板上默认内容)。"""
    rs = _rs()
    img = rs.randint(0, 256, (H, W, 3)).astype(np.uint8)
    got = hashlib.sha256(pack_obs.pack_slice(img)).hexdigest()
    assert got == GOLD_1BIT_U8, f'1-bit(uint8) 输出变了: {got}'
    b = rs.rand(H, W, 3) > 0.5
    got = hashlib.sha256(pack_obs.pack_slice(b)).hexdigest()
    assert got == GOLD_1BIT_BOOL, f'1-bit(bool) 输出变了: {got}'
    print('[ok] ④ 1-bit 黄金哈希与改造前一致 (uint8 阈值路径 + bool 路径)')


def test_to_3bit_quantizer():
    """8 级量化 + 残差抖动: 单调 / 无偏 / gamma 生效 / 相位随槽变。"""
    try:
        import gen_anime_slices as gas
    except ImportError as e:                 # PIL 缺失等
        print(f'[skip] ⑤ to_3bit 量化器测试 (import 失败: {e})')
        return
    g = gas.LED_GAMMA
    # 不抖动: 全图常量灰 v → 码值 = round(7*v^g), 且随 v 单调不降
    prev = -1
    for v in range(0, 256, 4):
        img = np.full((H, W, 3), float(v), np.float32)
        c = int(gas.to_3bit(img, 128, False, 0)[0, 0, 0])
        assert c == int(round(7 * (v / 255.0) ** g)), (v, c)
        assert c >= prev, f'码值随亮度非单调: v={v}'
        prev = c
    assert prev == 7, '满量程必须给到码值 7'
    # 抖动: 一片常量灰的平均码值 ≈ 7*v^g (无偏), 误差 ≤ 1/16 + 边界效应
    worst = 0.0
    for v in range(0, 256, 8):
        img = np.full((H, W, 3), float(v), np.float32)
        q = gas.to_3bit(img, 128, True, 0).astype(np.float64)
        worst = max(worst, abs(q.mean() - 7 * (v / 255.0) ** g))
    assert worst < 0.12, f'残差抖动有偏: 最大偏差 {worst:.3f} 码'
    # 相位随槽变 ⇒ 相邻槽的抖动图样不同 (时域平滑的前提)
    img = np.full((H, W, 3), 100.0, np.float32)
    a = gas.to_3bit(img, 128, True, 0)
    b = gas.to_3bit(img, 128, True, 1)
    assert not np.array_equal(a, b), '相位没起作用 (退化成固定阈值)'
    # gamma=1.0 一定比 gamma=2.2 亮 (中灰处)
    lin = gas.to_3bit(img, 128, False, 0, gamma=1.0)[0, 0, 0]
    assert lin > gas.to_3bit(img, 128, False, 0)[0, 0, 0]
    print(f'[ok] ⑤ to_3bit: 单调 + 无偏 (最大偏差 {worst:.3f} 码) + 相位随槽变')


def test_to_3bit_pack_roundtrip():
    """量化器 → pack → unpack 端到端: 出来的码值必须与量化器给的完全一致。"""
    try:
        import gen_anime_slices as gas
    except ImportError:
        print('[skip] ⑤ to_3bit → pack → unpack 端到端')
        return
    rs = _rs(5)
    img = rs.rand(H, W, 3).astype(np.float32) * 255.0
    q = gas.to_3bit(img, 128, True, 7)
    back = pack_obs.unpack_slice(pack_obs.pack_slice(q, bpp=3), bpp=3)
    assert np.array_equal(back, q)
    print('[ok] ⑤ to_3bit → pack_slice(bpp=3) → unpack_slice 端到端码值一致')


def main():
    tests = [v for k, v in globals().items() if k.startswith('test_')]  # 定义序
    for t in tests:
        t()
    print(f'\nALL PASS ({len(tests)} 组)')


if __name__ == '__main__':
    main()
