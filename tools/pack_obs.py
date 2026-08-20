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

plane 内布局 (DDR 镜像契约, 1-bit 时代起就没变过):
  偏移 = lane*1296 + row*24 + word*4, little-endian 32bit
  9 lane × 54 row × 6 word × 4B = 11664B, plane 步长 0x3000 (12KB), 余下补 0

色深 (2026-08-20, feature/3bit-color):
  bpp=1 (默认): 一片 = 一个 plane = 0x3000, 与历史逐字节一致。
  bpp=3       : 每通道 3 bit (码值 0..7), 行内 BCM。
                一片 = 3 个 plane 顺序排列, **plane p 在 slice_base + p*0x3000**,
                plane0 = LSB(权重 1) / plane1(权重 2) / plane2 = MSB(权重 4)。
                每个 plane 内部沿用上面那套 lane-major 布局, 一个 bit 都没动。
                片步长 SLICE_STRIDE_3BIT = 0x9000 = 36864。
                硬件 OE 权重 27/54/108 沿 = 1:2:4 ⇒ **码值与发光时间成正比,
                是线性的**, 所有 gamma 编码都在 host 侧做完 (见 povstream --led-gamma)。
"""
import numpy as np

W, H = 160, 180
STRIPS = [(0, 53, 6), (53, 53, 3), (106, 54, 0)]   # (X起点, 宽, lane基)
LANES, ROWS, WORDS = 9, 54, 6
SLICE_DATA = LANES * ROWS * WORDS * 4              # 11664 = 一个 plane 的有效数据
PLANE_STRIDE = 0x3000                              # 12288 = 一个 plane 的步长
SLICE_STRIDE = PLANE_STRIDE                        # 1-bit 片步长 (兼容老常量)
SLICE_STRIDE_3BIT = 3 * PLANE_STRIDE               # 36864 = 0x9000
PLANE_PAD = b'\0' * (PLANE_STRIDE - SLICE_DATA)    # 624B
BPP_MODES = (1, 3)
PLANES = {1: 1, 3: 3}                              # bpp → 位平面数
MAX_CODE = {1: 1, 3: 7}                            # bpp → 最大码值


def slice_stride(bpp=1):
    """bpp → 片步长 (1 → 0x3000, 3 → 0x9000)。凡是过去写死 SLICE_STRIDE 的
    地方, 只要可能跑 3-bit 就改走这里 —— 板端也是同一条 stride 契约。"""
    if bpp not in BPP_MODES:
        raise ValueError(f'bpp={bpp} 不支持 (只有 {BPP_MODES})')
    return PLANES[bpp] * PLANE_STRIDE


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


def pack_plane(on):
    """一个位平面: 观察者视角 bool 图 (H,W,3) → SLICE_DATA 字节 (lane-major)。

    这是 1-bit 时代 pack_slice 的函数体, 一个字节都没动 —— 3-bit 只是把它
    在三个位平面上各调一次。"""
    words = np.zeros((LANES, ROWS, WORDS), np.uint32)
    ys, xs, cs = np.nonzero(on)
    lanes = _X_LANE[xs] + cs
    rows = _X_ROW[xs]
    wds = _Y_WORD[ys]
    bits = _Y_BIT[ys]
    np.bitwise_or.at(words, (lanes, rows, wds), (np.uint32(1) << bits.astype(np.uint32)))
    return words.astype('<u4').tobytes()


def unpack_plane(buf):
    """SLICE_DATA (或更长) 字节 → 观察者视角 bool 图 (H,W,3)。pack_plane 的逆。"""
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


def pack_slice(img, bpp=1, pad=False):
    """观察者视角图 (H=180, W=160, 3) → 一片的字节。

    bpp=1: img 非零/≥128 即点亮 (调用方自行完成 1-bit 阈值/抖动)。
           返回 SLICE_DATA=11664B; pad=True 时补到 PLANE_STRIDE=12288B。
           ⚠ 这条路径与 2026-08-20 之前**逐字节相同**, 是 3-bit 改造的回归判据。
    bpp=3: img 是 uint8 码值 0..7 (0=灭, 7=全亮, 与发光时间成正比)。
           按位切成 plane0(LSB)/plane1/plane2(MSB), 每个 plane 各走一遍
           pack_plane 并各自补到 0x3000 → 返回 SLICE_STRIDE_3BIT=36864B。
           plane 之间的 624B padding 是片内布局契约的一部分, 因此 3-bit
           **恒定带 padding**, pad 参数被忽略。
    """
    a = np.asarray(img)
    assert a.shape == (H, W, 3), f"expect (180,160,3), got {a.shape}"
    if bpp == 1:
        on = a > 0 if a.dtype == np.bool_ else a >= 128
        b = pack_plane(on)
        return b + PLANE_PAD if pad else b
    if bpp != 3:
        raise ValueError(f'bpp={bpp} 不支持 (只有 {BPP_MODES})')
    code = a.astype(np.uint8, copy=False)
    if code.size and int(code.max()) > 7:
        raise ValueError(f'3-bit 码值须在 0..7, 实得 max={int(code.max())}')
    return b''.join(pack_plane(((code >> p) & 1).astype(np.bool_)) + PLANE_PAD
                    for p in range(3))


def unpack_slice(buf, bpp=1):
    """一片的字节 → 观察者视角图。pack_slice 的逆 (含 padding 与否都能吃)。

    bpp=1 → bool 图 (180,160,3); bpp=3 → uint8 码值图 0..7。"""
    if bpp == 1:
        return unpack_plane(buf)
    if bpp != 3:
        raise ValueError(f'bpp={bpp} 不支持 (只有 {BPP_MODES})')
    b = bytes(buf)
    if len(b) < 2 * PLANE_STRIDE + SLICE_DATA:
        raise ValueError(f'3-bit 片需要 {SLICE_STRIDE_3BIT}B, 实得 {len(b)}B')
    img = np.zeros((H, W, 3), np.uint8)
    for p in range(3):
        off = p * PLANE_STRIDE
        img |= unpack_plane(b[off:off + SLICE_DATA]).astype(np.uint8) << p
    return img


def pack_image(im):
    """PIL RGB Image (160x180) → slice 字节 (1-bit)。"""
    return pack_slice(np.array(im.convert('RGB')))
