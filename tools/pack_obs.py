#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pack_obs.py — FS03 160x180 观察者视角 → lane fb 打包库 (gen_chess_obs.py 实测映射函数化).

映射 (2026-07-08 实测, 权威 = gen_chess_obs.py):
  条:   观察者 X 0..52 = lanes {6,7,8} (左条,53宽) / 53..105 = {3,4,5} (53宽)
        / 106..159 = {0,1,2} (54宽)
  行:   条内 row r = (条宽-1) - 条内偏移 p   (53宽条永远到不了 r=53)
  竖:   观察者 Y 0..179 直通 (上→下)
  色:   lane_base+0 = R / +1 = G / +2 = B
  bit:  h = 11 - Y//15, 32bit 字 = h//2, bit = (h%2)*16 + (Y%15)

slice 内布局 (DDR 镜像契约):
  偏移 = lane*1296 + row*24 + word*4, little-endian 32bit
  9 lane × 54 row × 6 word × 4B = 11664B, slice 步长 0x3000 (12KB), 余下补 0
"""
import numpy as np

W, H = 160, 180
STRIPS = [(0, 53, 6), (53, 53, 3), (106, 54, 0)]   # (X起点, 宽, lane基)
LANES, ROWS, WORDS = 9, 54, 6
SLICE_DATA = LANES * ROWS * WORDS * 4              # 11664
SLICE_STRIDE = 0x3000                              # 12288


def obs_to_lane_row(X):
    """观察者列 X (0..159) → (lane_base, row)。"""
    for x0, w, base in STRIPS:
        if x0 <= X < x0 + w:
            return base, (w - 1) - (X - x0)
    raise ValueError(f"X={X} out of range")


def lane_row_to_obs(lane_base, r):
    """(lane_base, row) → 观察者列 X, 无对应列 (53宽条 r=53) 返回 None。"""
    for x0, w, base in STRIPS:
        if base == lane_base:
            if r >= w:
                return None
            return x0 + (w - 1) - r
    raise ValueError(f"lane_base={lane_base} invalid")


# 预计算查表: X → (lane_base, row); Y → (word, bit)
_X_LANE = np.zeros(W, np.int32)
_X_ROW = np.zeros(W, np.int32)
for _X in range(W):
    _X_LANE[_X], _X_ROW[_X] = obs_to_lane_row(_X)
_Y_H = 11 - np.arange(H) // 15
_Y_WORD = _Y_H // 2
_Y_BIT = (_Y_H % 2) * 16 + np.arange(H) % 15


def pack_slice(img):
    """观察者视角图 (H=180, W=160, 3) uint8/bool → SLICE_DATA 字节 (lane-major).

    img[Y, X, c] 非零即点亮 (调用方自行完成 1-bit 阈值/抖动)。
    """
    a = np.asarray(img)
    assert a.shape == (H, W, 3), f"expect (180,160,3), got {a.shape}"
    on = a > 0 if a.dtype == np.bool_ else a >= 128
    words = np.zeros((LANES, ROWS, WORDS), np.uint32)
    ys, xs, cs = np.nonzero(on)
    lanes = _X_LANE[xs] + cs
    rows = _X_ROW[xs]
    wds = _Y_WORD[ys]
    bits = _Y_BIT[ys]
    np.bitwise_or.at(words, (lanes, rows, wds), (np.uint32(1) << bits.astype(np.uint32)))
    return words.astype('<u4').tobytes()


def unpack_slice(buf):
    """SLICE_DATA (或更长) 字节 → 观察者视角 bool 图 (180,160,3)。pack_slice 的逆。"""
    words = np.frombuffer(bytes(buf[:SLICE_DATA]), '<u4').reshape(LANES, ROWS, WORDS)
    img = np.zeros((H, W, 3), np.bool_)
    for lane in range(LANES):
        base, ci = (lane // 3) * 3, lane % 3
        for r in range(ROWS):
            X = lane_row_to_obs(base, r)
            if X is None:
                continue
            col = (words[lane, r, _Y_WORD] >> _Y_BIT) & 1
            img[:, X, ci] = col.astype(np.bool_)
    return img


def pack_image(im):
    """PIL RGB Image (160x180) → slice 字节。"""
    return pack_slice(np.array(im.convert('RGB')))
