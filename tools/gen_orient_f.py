#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_orient_f.py — 生成屏幕取向探针图 tools/orient_f.png (观察者视角 160宽 x 180高)。

内容:
  - 黑底
  - 大号白色字母 "F": 占中间约 100x140, 笔画宽 14px (>=12),
    标准写法 — 竖笔在左, 开口朝右 (上横长, 中横略短)
  - 左上角 15x15 纯红块 / 右上角 15x15 纯蓝块 / 左下角 15x15 纯绿块

上屏后判读:
  - F 正立不镜像 + 红左上/蓝右上/绿左下 → 取向和镜像全对
  - F 镜像 → 需加 --flip-x 或 --flip-y (视旋转而定)
  - F 躺倒 → --rotate 选错方向 (90 vs 270)
  角块颜色同时兼做 lane 颜色核对。

输出: tools/orient_f.png + orient_f_preview.png (4x)
纯软件工具, 不碰硬件。
"""
import os
from PIL import Image

W, H = 160, 180              # 观察者视角: 160 宽 x 180 高
BLACK, WHITE = (0, 0, 0), (255, 255, 255)
RED, GREEN, BLUE = (255, 0, 0), (0, 255, 0), (0, 0, 255)

im = Image.new('RGB', (W, H), BLACK)
px = im.load()


def rect(x0, y0, w, h, col):
    """填充矩形 [x0,x0+w) x [y0,y0+h), 自动裁剪到画布"""
    for y in range(max(0, y0), min(H, y0 + h)):
        for x in range(max(0, x0), min(W, x0 + w)):
            px[x, y] = col


# ---------- 字母 F: 100x140 居中, 笔画宽 14 ----------
FW, FH, S = 100, 140, 14
fx, fy = (W - FW) // 2, (H - FH) // 2       # (30, 20)
rect(fx, fy, S, FH, WHITE)                   # 竖笔 (左)
rect(fx, fy, FW, S, WHITE)                   # 上横 (全宽 → 开口朝右)
rect(fx, fy + (FH - S) // 2, int(FW * 0.75), S, WHITE)   # 中横 (略短)

# ---------- 角标: 红左上 / 蓝右上 / 绿左下 ----------
M = 15
rect(0, 0, M, M, RED)
rect(W - M, 0, M, M, BLUE)
rect(0, H - M, M, M, GREEN)

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, 'orient_f.png')
im.save(out)
im.resize((W * 4, H * 4), Image.NEAREST).save(os.path.join(here, 'orient_f_preview.png'))
print(f"saved {out} ({W}x{H}), F at ({fx},{fy}) {FW}x{FH} stroke={S}")
