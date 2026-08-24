#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_wedge.py — 8 级灰度楔测试图 (3-bit 色深上板目视用, 05_3bit_bcm.md §7.3).

判据 (上板肉眼看):
  · 8 阶必须**单调**: 从码 0 到码 7 一路变亮, 不许中间有一档反而变暗
    (反了 = plane 权重接错/位序反了, 27/54/108 三个 OE 权重与 plane0/1/2 对不上)
  · 不许**跳阶**: 相邻两档亮度差应当逐档变小 (线性光 1/7 步进 + 人眼 ~2.2 次方
    ⇒ 0→1 那一步天生最大, 6→7 最小); 若某一档"跳"得离谱, 多半是那个 plane
    的 OE 窗口不对 (权重不是 1:2:4) 或 LWAIT 插进来了
  · 三色分开看: R/G/B 各自的 8 阶都要单调 —— 单色不单调 = 那条 lane 的位平面串了

输出 (默认 = 一个可以直接推流的预渲染目录):
  <out-dir>/frame_0000.bin  n_slices 片, 每片同一张图 (旋转起来 = 绕轴的回转体)
  <out-dir>/meta.json       bpp / n_slices / geom_flags, povstream stream --dir 认这个
  可选 --png                预览图 (需要 PIL), 按 sRGB 反 gamma 显示, 近似肉眼所见

用法:
  python3 tools/gen_wedge.py --bpp 3 --n-slices 50 \\
      --out-dir stream/pc/frames_wedge3 --png /tmp/wedge.png
  python3 stream/pc/povstream.py stream --dir stream/pc/frames_wedge3 \\
      --n-slices 50 --host <board> --fps 5 --loop
  # 1-bit 对照组 (同样的目标灰度, 走 Bayer 抖动) —— A/B 看 3-bit 到底赢在哪:
  python3 tools/gen_wedge.py --bpp 1 --n-slices 360 --out-dir stream/pc/frames_wedge1

⚠ 图案与观察者视角的关系 (选 --pattern 时要想清楚):
  · flat  (默认): 8 档沿 **Y (屏高)**, 4 条色带沿 X。屏**不转**、直接看面板时
                  这就是所见即所得的 8×4 方格。
  · checker: 数字灰度棋盘 —— X 方向 8 列 = 码值 0..7, Y 方向逐行正/反相,
             每格里用 3x5 点阵写着自己的码值。**屏不转直接看面板**用这个:
             一眼同时看灰度单调性 / 相邻级可分辨性 / 格边界有没有糊开。
  · grid64: 8x8=64 格编号方阵 + 三色渐变。行 = 通道组合 (白/R/G/B/黄/品红/青/
             白反向), 列 = 8 级亮度, 每格写着自己的格号 1..64。
             看三色独立性 / 通道串扰 / 两端压不压得住, 出问题能指认格号。
  · rings: 同样 8 档沿 Y, 但 X 方向**关于中心列左右对称**。屏转起来时观察者列 X
           与 160-1-X 落在同一半径上, 不对称的图案会互相叠加成一团;
           对称排布转出来就是干净的同心色环 —— **转起来看用这个**。
  · ramp:  连续渐变 (线性光) 走 to_3bit 的残差抖动, 用来看抖动够不够顺、
           有没有 Bayer 网纹。需要 gen_anime_slices (PIL)。
"""
import os
import sys
import json
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pack_obs
from pack_obs import W, H

LEVELS = 8                      # 码值 0..7
DEFAULT_GAMMA = 2.2             # = gen_anime_slices.LED_GAMMA (只影响 1-bit 对照组与预览)
BANDS = [('W', (1, 1, 1)), ('R', (1, 0, 0)), ('G', (0, 1, 0)), ('B', (0, 0, 1))]


def _band_of_y(Y):
    """屏高 Y → 档位 0..7 (**上亮下暗**: Y=0 是码 7)。"""
    return (LEVELS - 1) - min(Y * LEVELS // H, LEVELS - 1)


# 3x5 点阵数字 0-7 (用来把码值本身写进格子里, 免得靠数位置认级)
DIGITS = {
    0: ('111', '101', '101', '101', '111'),
    1: ('010', '110', '010', '010', '111'),
    2: ('111', '001', '111', '100', '111'),
    3: ('111', '001', '111', '001', '111'),
    4: ('101', '101', '111', '001', '001'),
    5: ('111', '100', '111', '001', '111'),
    6: ('111', '100', '111', '101', '111'),
    7: ('111', '001', '001', '001', '001'),
    8: ('111', '101', '111', '101', '111'),
    9: ('111', '101', '111', '001', '111'),
}
CELL = 20                       # 格边长 (px): W=160 -> 8 列 = 码值 0..7; H=180 -> 9 行
DIG_SCALE = 4                   # 3x5 -> 12x20, 正好塞进一格


def _stamp_digit(code, y0, x0, d, val):
    """把数字 d 以码值 val 画进格子 (y0,x0) 左上角的 CELL x CELL 区域, 居中。"""
    gh, gw = 5 * DIG_SCALE, 3 * DIG_SCALE
    oy, ox = y0 + (CELL - gh) // 2, x0 + (CELL - gw) // 2
    for r, row in enumerate(DIGITS[d]):
        for c, ch in enumerate(row):
            if ch == '1':
                ys, xs = oy + r * DIG_SCALE, ox + c * DIG_SCALE
                code[ys:ys + DIG_SCALE, xs:xs + DIG_SCALE, :] = val


def build_checker():
    """数字灰度棋盘: X 方向 8 列 = 码值 0..7; Y 方向逐行反相 (背景/数字对调)。

    一列里同时看到该码值的"正"(亮格黑字)和"反"(黑格亮字)两种呈现:
      · 横看 8 列 = 8 级灰度是否单调、相邻级分不分得开
      · 格边界 = 空间对齐 / 串扰 (plane 边界若锁存错位, 会在这里糊开)
      · 数字本身 = 不用数位置就知道这格该是几
    """
    code = np.zeros((H, W, 3), np.uint8)
    for cx in range(W // CELL):                     # 8 列
        val = min(cx, LEVELS - 1)                   # 码值 0..7
        for cy in range(H // CELL):                 # 9 行
            y0, x0 = cy * CELL, cx * CELL
            if cy % 2 == 0:                         # 亮格黑字
                code[y0:y0 + CELL, x0:x0 + CELL, :] = val
                _stamp_digit(code, y0, x0, val, 0)
            else:                                   # 黑格亮字
                _stamp_digit(code, y0, x0, val, val)
    return code



# ---- grid64: 8x8 编号方阵 + 三色渐变 -------------------------------------
GRID_CW, GRID_CH = W // 8, 22       # 20 x 22, 8 行占 176 行, 顶部留 4 行黑
GRID_ROWS = [                       # (名字, 通道掩码, 是否反向递减)
    ('W',  (1, 1, 1), False),
    ('R',  (1, 0, 0), False),
    ('G',  (0, 1, 0), False),
    ('B',  (0, 0, 1), False),
    ('RG', (1, 1, 0), False),
    ('RB', (1, 0, 1), False),
    ('GB', (0, 1, 1), False),
    ('W',  (1, 1, 1), True),
]


def _stamp_number(code, y0, x0, n, val, mask):
    """把十进制数 n 画进 (y0,x0) 起的 GRID_CW x GRID_CH 格子, 居中。"""
    sc = 2
    ds = str(n)
    gw = (len(ds) * 3 + (len(ds) - 1)) * sc      # 位宽 3 + 位间距 1
    gh = 5 * sc
    oy, ox = y0 + (GRID_CH - gh) // 2, x0 + (GRID_CW - gw) // 2
    for i, ch in enumerate(ds):
        bx = ox + i * 4 * sc
        for r, row in enumerate(DIGITS[int(ch)]):
            for c, bit in enumerate(row):
                if bit == '1':
                    ys, xs = oy + r * sc, bx + c * sc
                    for k in range(3):
                        if mask[k]:
                            code[ys:ys + sc, xs:xs + sc, k] = val


def build_grid64():
    """8x8 = 64 格, 每格写编号 1..64; 每行一种通道组合, 沿 X 走 8 级渐变。

    一张图同时给出:
      · 第 2/3/4 行 = R/G/B 三条各自的 8 级渐变 (三色是不是独立、级数对不对)
      · 第 5/6/7 行 = 黄/品红/青 三个混色 (通道之间有没有串)
      · 第 1/8 行 = 正反两条白阶对照 (两端有没有被压掉)
      · 格号 1..64 = 出问题时能直接指认是哪一格
    """
    code = np.zeros((H, W, 3), np.uint8)
    for ry, (_, mask, rev) in enumerate(GRID_ROWS):
        y0 = 4 + ry * GRID_CH
        if y0 + GRID_CH > H:
            break
        for cx in range(8):
            x0 = cx * GRID_CW
            val = (LEVELS - 1 - cx) if rev else cx
            for k in range(3):
                if mask[k]:
                    code[y0:y0 + GRID_CH, x0:x0 + GRID_CW, k] = val
            # 数字取对比色: 底亮就画暗字, 底暗就画亮字
            _stamp_number(code, y0, x0, ry * 8 + cx + 1,
                          0 if val >= 4 else LEVELS - 1, mask)
    return code



def build_bw(cell=40):
    """纯黑白粗棋盘 (只用码值 0 和 7), 但走 3-bit 路径。

    诊断用: 把"灰度"和"3-bit 时序"分开 ——
      · 这张图还糊  => 问题在 3-bit 的时序/锁存, 与灰度级无关
      · 这张图很锐  => 3-bit 通路是好的, 糊来自中间灰度级本身
                       (或笔画太细 / 抖动 / 面板本身的点扩散)
    格子取 40x45 (4x4), 远大于笔画宽度, 排除"太细看不清"的干扰。
    """
    code = np.zeros((H, W, 3), np.uint8)
    ch = H // 4
    for cy in range(4):
        for cx in range(W // cell):
            if (cx + cy) % 2 == 0:
                code[cy * ch:(cy + 1) * ch, cx * cell:(cx + 1) * cell, :] = LEVELS - 1
    return code



def build_levels():
    """最干净的灰阶判据: 8 条纯白色块横排, 码值 0..7, 无数字/无棋盘/无彩色。

    专门用来回答"到底有没有 8 级"这一个问题 —— grid64 里的数字和棋盘
    会严重干扰相邻块的亮度比较, 高亮端尤其容易糊成一片。
    """
    code = np.zeros((H, W, 3), np.uint8)
    for cx in range(8):
        code[:, cx * (W // 8):(cx + 1) * (W // 8), :] = cx
    return code


def build_full():
    """全屏码值 7 —— 用来看物理点亮区域到底覆盖到哪, 确认有没有额外黑边。"""
    return np.full((H, W, 3), LEVELS - 1, np.uint8)



def build_rgb3():
    """一张图同时显示 R/G/B 三条渐变: 屏高分 3 条大色带, 每条沿 X 走 8 级。

    比 grid64 直观 —— 不用在 8 行里找哪行是纯色, 三条色带一眼平行对比:
      · 每条各自单调吗 (三个通道独立成阶)
      · 三条的级数看起来一致吗 (通道间增益是否一致)
      · 同一列上三条的亮度关系 (白平衡)
    """
    code = np.zeros((H, W, 3), np.uint8)
    band = H // 3
    for bi, ch in enumerate((0, 1, 2)):            # 上=R 中=G 下=B
        y0 = bi * band
        y1 = H if bi == 2 else (bi + 1) * band
        for cx in range(8):
            code[y0:y1, cx * (W // 8):(cx + 1) * (W // 8), ch] = cx
    return code



def build_ab():
    """受控对比: 上半屏白色 8 级 / 下半屏红色 8 级, 同一张图同一组码值。

    用来回答一个具体问题: "单通道(只有一个颜色亮)时还有没有 8 级?"
      · 上下都是 8 级 => 单通道没问题, 之前判读受了干扰
      · 上半 8 级 / 下半只有亮和不亮 => 单通道特有问题, 往 lane/plane 映射查
    白色三通道同时亮, 低码值端的绝对亮度是单通道的 3 倍, 所以低端更容易看见 ——
    这本身也是一个可能的解释, 对比图能把它和"真的只有二值"分开。
    """
    code = np.zeros((H, W, 3), np.uint8)
    half = H // 2
    for cx in range(8):
        x0, x1 = cx * (W // 8), (cx + 1) * (W // 8)
        code[:half, x0:x1, :] = cx          # 上: 白 (R=G=B)
        code[half:, x0:x1, 0] = cx          # 下: 纯红
    return code



def build_halftest():
    """半屏扫描 (half_scan) 专用: 内容只放在 **Y 90..179**, 上半留黑。

    为什么是下半: pack_obs 的 Y 映射是 `_Y_H = 11 - Y//15`, 即 Y=179 落在芯片 0
    (移位链的数据入口端)。half_scan 每行只发 96 bit, 更新的正是靠入口那 6 颗
    = 芯片 0..5 = **Y 90..179**; 远端 6 颗保持旧值 (须先整链清零)。

    图案: 下半屏放 8 列灰阶 (码值 0..7) + 一条彩色带, 既验色深也验哪一半亮。
    上半屏全黑 —— 如果开了 half_scan 之后上半屏出现残影, 说明链没清干净。
    """
    code = np.zeros((H, W, 3), np.uint8)
    y0 = H // 2                       # 90
    band = (H - y0) // 2              # 45
    for cx in range(8):
        x0, x1 = cx * (W // 8), (cx + 1) * (W // 8)
        code[y0:y0 + band, x0:x1, :] = cx            # 上半段: 白灰阶
        code[y0 + band:, x0:x1, cx % 3] = cx         # 下半段: R/G/B 轮转
    return code


def build_codes(pattern):
    """→ (H,W,3) uint8 码值图 0..7。"""
    code = np.zeros((H, W, 3), np.uint8)
    lvl = np.array([_band_of_y(y) for y in range(H)], np.uint8)      # (H,)
    if pattern == 'flat':
        for i, (_, mask) in enumerate(BANDS):
            x0, x1 = i * W // len(BANDS), (i + 1) * W // len(BANDS)
            for c in range(3):
                if mask[c]:
                    code[:, x0:x1, c] = lvl[:, None]
    elif pattern == 'halftest':
        return build_halftest()
    elif pattern == 'ab':
        return build_ab()
    elif pattern == 'rgb3':
        return build_rgb3()
    elif pattern == 'levels':
        return build_levels()
    elif pattern == 'full':
        return build_full()
    elif pattern == 'bw':
        return build_bw()
    elif pattern == 'grid64':
        return build_grid64()
    elif pattern == 'checker':
        return build_checker()
    elif pattern == 'rings':
        # 左右对称: 半宽 80 分 4 段, 由中心向外 W/R/G/B, 另一半镜像
        half = W // 2
        for i, (_, mask) in enumerate(BANDS):
            r0, r1 = i * half // len(BANDS), (i + 1) * half // len(BANDS)
            for c in range(3):
                if mask[c]:
                    code[:, half + r0:half + r1, c] = lvl[:, None]      # 右半
                    code[:, half - r1:half - r0, c] = lvl[:, None]      # 左半 (镜像)
    else:
        raise ValueError(pattern)
    return code


def build_ramp_codes(gamma, phase):
    """连续线性渐变 (码值域 0..7 连续) → to_3bit 残差抖动后的码值图。"""
    import gen_anime_slices as gas
    # 目标线性光沿 Y 从 0 到 1 连续; 反 gamma 回输入域再交给 to_3bit
    lin = np.linspace(1.0, 0.0, H, dtype=np.float32)                 # 上亮下暗
    v = 255.0 * np.power(lin, 1.0 / gamma)
    img = np.repeat(np.repeat(v[:, None, None], W, 1), 3, 2)
    for i, (_, mask) in enumerate(BANDS):                            # 分色带
        x0, x1 = i * W // len(BANDS), (i + 1) * W // len(BANDS)
        for c in range(3):
            if not mask[c]:
                img[:, x0:x1, c] = 0.0
    return gas.to_3bit(img, 128, True, phase, gamma=gamma)


def codes_to_input(code, gamma):
    """码值 0..7 → 输入域 (sRGB) 0..255: v = 255·(code/7)^(1/gamma)。
    1-bit 对照组和 PNG 预览都用它 —— 保证两条路的**目标线性光完全一样**。"""
    return 255.0 * np.power(code.astype(np.float32) / (LEVELS - 1), 1.0 / gamma)


def pack_frame(code, bpp, n_slices, gamma, phase0=0):
    """码值图 → 一帧 (n_slices 片, 每片同一张图)。

    bpp=3: 直接打包码值。
    bpp=1: 先把码值折回输入域, 再走与 povstream 完全同一套 Bayer 抖动
           (相位逐槽变) —— 这样 1-bit 对照组和 3-bit 是同一个目标亮度。"""
    parts = []
    if bpp == 3:
        one = pack_obs.pack_slice(code, bpp=3)
        return one * n_slices
    import gen_anime_slices as gas
    img = codes_to_input(code, gamma)
    for s in range(n_slices):
        on = gas.to_1bit(img, 128, True, phase0 + s)
        parts.append(pack_obs.pack_slice(on, bpp=1, pad=True))
    return b''.join(parts)


def save_png(code, path, gamma):
    from PIL import Image
    px = np.clip(codes_to_input(code, gamma), 0, 255).astype(np.uint8)
    Image.fromarray(px).resize((W * 3, H * 3), Image.NEAREST).save(path)


def main():
    ap = argparse.ArgumentParser(description='8 级灰度楔测试图 (3-bit 上板目视)')
    ap.add_argument('--bpp', type=int, choices=sorted(pack_obs.BPP_MODES), default=3)
    ap.add_argument('--pattern', choices=['flat', 'checker', 'grid64', 'rgb3', 'ab', 'halftest', 'levels', 'full', 'bw', 'rings', 'ramp'], default='flat')
    ap.add_argument("--n-slices", type=int, default=50,
                    help='帧里的片数 (默认 60 = 3-bit 方案推荐的每面槽数)')
    ap.add_argument('--gamma', type=float, default=DEFAULT_GAMMA,
                    help=f'反 gamma (默认 {DEFAULT_GAMMA}), 只影响 1-bit 对照组/预览/ramp; '
                         'flat/rings 的 3-bit 码值是**直接写死的 0..7**, 不过 gamma')
    ap.add_argument('--out-dir', default=None,
                    help='预渲染目录 (默认 stream/pc/frames_wedge<bpp>)')
    ap.add_argument('--png', default=None, metavar='PATH', help='另存预览 PNG (需 PIL)')
    a = ap.parse_args()

    stride = pack_obs.slice_stride(a.bpp)
    if not 1 <= a.n_slices <= 8847360 // stride:
        sys.exit(f'--n-slices 须在 1..{8847360 // stride} (bpp={a.bpp} 片距 0x{stride:X})')

    code = (build_ramp_codes(a.gamma, 0) if a.pattern == 'ramp'
            else build_codes(a.pattern))
    raw = pack_frame(code, a.bpp, a.n_slices, a.gamma)
    assert len(raw) == a.n_slices * stride, (len(raw), a.n_slices * stride)

    out = a.out_dir or os.path.join(HERE, '..', 'stream', 'pc', f'frames_wedge{a.bpp}')
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'frame_0000.bin'), 'wb') as f:
        f.write(raw)
    # meta.json: povstream stream --dir 靠 bpp 推片距, 靠 geom_flags 推面拆分
    meta = {'anim': f'wedge-{a.pattern}', 'frames': 1, 'render_slices': a.n_slices,
            'frame_raw': len(raw), 'freeze_phase': True,
            'bpp': a.bpp, 'led_gamma': a.gamma if a.bpp == 3 else None,
            'n_slices': a.n_slices, 'geom_flags': 0,
            'faces': [{'name': 'A', 'axis_off_px': 0.0, 'n_slices': a.n_slices}],
            'note': 'gen_wedge.py 生成的 8 级灰度楔 (单面, 每片同图)'}
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    if a.png:
        save_png(code, a.png, a.gamma)

    used = sorted(set(int(v) for v in np.unique(code)))
    print(f'[wedge] {a.pattern} bpp={a.bpp} 码值 {used} → {out}')
    print(f'[wedge] {a.n_slices} 片 × 0x{stride:X} = {len(raw)}B'
          + (f', 预览 {a.png}' if a.png else ''))
    print(f'[wedge] 推流: python3 stream/pc/povstream.py stream --dir {out} '
          f'--n-slices {a.n_slices}' + (f' --bpp {a.bpp}' if a.bpp != 1 else '')
          + ' --host <board> --fps 5 --loop')


if __name__ == '__main__':
    main()
