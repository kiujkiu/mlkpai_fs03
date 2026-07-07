#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_chess160.py — 生成 180(宽) x 160(高) 编号棋盘格测试图, 用于 ICND2049 屏上屏定位。

格子 20x20 → 9 列 x 8 行 = 72 格, 每格:
  - 底色: 7 种 1-bit 可表达纯色循环 (R/G/B/黄/品/青/白), 相邻格保证不同色
    (横向相邻: idx 差 1, 7 色循环必不同; 纵向相邻: idx 差 9, 9 mod 7 = 2 ≠ 0 也不同)
  - 白色 1px 边框 (白底格改黑边框, 否则看不见)
  - 格内左上画格子序号 (0..71), 3x5 点阵字体放大 2 倍;
    白/黄/青 底用黑字, 其余用白字 (保证 1-bit 阈值化后仍可辨认)

输出:
  tools/chess160.png          — 180x160 原始尺寸 (给 fb_pack.py 用)
  tools/chess160_preview.png  — 放大 4x 便于肉眼检查

纯软件工具, 不碰任何硬件。
"""
import os
from PIL import Image

# ---------- 几何 ----------
W, H = 180, 160          # 宽=列轴 180, 高=扫描轴 160
CELL = 20                # 格子边长
GX, GY = W // CELL, H // CELL   # 9 x 8 = 72 格

# ---------- 1-bit 可表达纯色调色板 (每通道只有 0/255) ----------
PAL = [
    (255, 0,   0),    # R
    (0,   255, 0),    # G
    (0,   0,   255),  # B
    (255, 255, 0),    # 黄
    (255, 0,   255),  # 品
    (0,   255, 255),  # 青
    (255, 255, 255),  # 白
]
# 这些底色上白字不清楚 → 用黑字; 白底还要黑边框
DARK_TEXT_BG = {(255, 255, 255), (255, 255, 0), (0, 255, 255)}

# ---------- 3x5 点阵数字字体 ----------
DIGITS = {
    '0': ['111', '101', '101', '101', '111'],
    '1': ['010', '110', '010', '010', '111'],
    '2': ['111', '001', '111', '100', '111'],
    '3': ['111', '001', '111', '001', '111'],
    '4': ['101', '101', '111', '001', '001'],
    '5': ['111', '100', '111', '001', '111'],
    '6': ['111', '100', '111', '101', '111'],
    '7': ['111', '001', '010', '100', '100'],
    '8': ['111', '101', '111', '101', '111'],
    '9': ['111', '101', '111', '001', '111'],
}
SCALE = 2                # 字体放大倍数 → 每个数字 6x10 px

px = [[(0, 0, 0)] * W for _ in range(H)]


def put_digit(ch, x0, y0, col):
    """在 (x0,y0) 画一个放大 SCALE 倍的点阵数字"""
    for dy, rowbits in enumerate(DIGITS[ch]):
        for dx, c in enumerate(rowbits):
            if c != '1':
                continue
            for sy in range(SCALE):
                for sx in range(SCALE):
                    X, Y = x0 + dx * SCALE + sx, y0 + dy * SCALE + sy
                    if 0 <= X < W and 0 <= Y < H:
                        px[Y][X] = col


for gy in range(GY):
    for gx in range(GX):
        idx = gy * GX + gx
        cc = PAL[idx % len(PAL)]
        x0, y0 = gx * CELL, gy * CELL
        # 底色填充
        for yy in range(CELL):
            for xx in range(CELL):
                px[y0 + yy][x0 + xx] = cc
        # 1px 边框: 白色; 白底格用黑色边框
        border = (0, 0, 0) if cc == (255, 255, 255) else (255, 255, 255)
        for xx in range(CELL):
            px[y0][x0 + xx] = border
            px[y0 + CELL - 1][x0 + xx] = border
        for yy in range(CELL):
            px[y0 + yy][x0] = border
            px[y0 + yy][x0 + CELL - 1] = border
        # 序号数字
        tcol = (0, 0, 0) if cc in DARK_TEXT_BG else (255, 255, 255)
        s = str(idx)
        wx = x0 + 3                       # 左上角留 3px (含边框)
        wy = y0 + 3
        for ch in s:
            put_digit(ch, wx, wy, tcol)
            wx += 3 * SCALE + 2           # 数字宽 + 2px 间距

# ---------- 输出 PNG ----------
here = os.path.dirname(os.path.abspath(__file__))
im = Image.new('RGB', (W, H))
for y in range(H):
    for x in range(W):
        im.putpixel((x, y), px[y][x])
out = os.path.join(here, 'chess160.png')
im.save(out)
im.resize((W * 4, H * 4), Image.NEAREST).save(os.path.join(here, 'chess160_preview.png'))
print(f"saved {out} ({W}x{H}, {GX}x{GY}={GX*GY} cells, cell={CELL})")
print(f"saved chess160_preview.png (4x)")
