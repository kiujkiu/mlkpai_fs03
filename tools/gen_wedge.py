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
  · colors8: **8 色循环屏测** (唯一的多帧图案, 见下)。

--------------------------------------------------------------------------
colors8 —— 8 色循环屏幕测试 (装机自检 / 坏点·串色排查)
--------------------------------------------------------------------------
唯一一个**出多帧**的 pattern: 每帧全屏一种纯色, 逐帧循环
    黑 → 红 → 绿 → 蓝 → 黄 → 青 → 品红 → 白
每色再按 --levels 走若干档 (默认 `7,3` = 满量程 + 中亮), **同色的两档紧挨着**,
方便直接比亮度差。黑色两档都是 0 ⇒ 黑帧天然双倍时长 = 循环起点的同步标记。

为什么默认 `7,3` 而不是只有 7:
    码 7 = 0b111 (三个位平面全开), 码 3 = 0b011 (只开 plane1+plane2)。
    两者**恰好差一个 plane0** (MSB, 权重 184/322)。于是 7↔3 这一步同时验了
    "MSB 位平面到底有没有到位" —— 只发满量程是看不出某个 plane 死掉的
    (plane0 死了, 全白只是整体暗一点, 没有参照物就发现不了)。

⚠ 旋转会把**列方向**的缺陷洗掉: 屏转起来后观察者的每个像素是 142 个角度的
   叠加, 一条坏列(坏 lane)只在 1/142 的时间里是暗的 ⇒ 几乎看不见。
   而 Y (屏高) 在旋转下不变 ⇒ **坏行 / 坏芯片 / 通道串色转着看很清楚**。
   所以完整验收要两步: ① 停转直视面板 (查坏点/坏列/坏 lane)
                      ② 转起来看 (查坏行/坏芯片/串色/白平衡/暗区)
   本图案两种看法用的是同一份数据 (每片同图), 不用重新生成。

--marks rulers (默认) 会在纯色上**挖黑**画标尺 (挖黑 = 不发光 ⇒ 完全不影响
"红帧只许红亮"的串色判据):
  · Y = 15,30,…,165 共 11 条 1px 黑线 = 12 颗行驱芯片的分界 (pack_obs 的
    h = 11 - Y//15)。转起来是 11 个干净的水平暗环, 哪个芯片死了一眼看出是第几段。
  · X = 53 / 106 两条 1px 黑线 = 三个 lane 条的分界 (pack_obs.STRIPS)。
    只在**停转直视**时有用 (转起来被叠加洗掉), 用来指认坏 lane 属于哪条。
  · 左上角一个黑色数字 = 颜色序号 0..7, 录像回放时好指认是第几帧。
--marks none 则是完全的纯色 (查坏点最干净)。

用法 (现役 142 槽双面, 每色停 0.5 s):
  python3 tools/gen_wedge.py --pattern colors8 --bpp 3 --n-slices 142 --faces 2 \\
      --out-dir stream/pc/frames_colors8 --png /tmp/colors8.png
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


# ---- colors8: 8 色循环屏测 (多帧) ----------------------------------------
# (中文名, 短名, 通道掩码) —— 顺序 = 播放顺序。
# 先黑 (查常亮坏点), 再 R/G/B 三原色 (查通道接反/串色), 再黄/青/品红三个两两
# 混色 (查通道之间有没有互相拉), 最后白 (查三通道亮度均衡 = 白平衡)。
COLORS8 = [
    ('黑', 'black',   (0, 0, 0)),
    ('红', 'red',     (1, 0, 0)),
    ('绿', 'green',   (0, 1, 0)),
    ('蓝', 'blue',    (0, 0, 1)),
    ('黄', 'yellow',  (1, 1, 0)),
    ('青', 'cyan',    (0, 1, 1)),
    ('品红', 'magenta', (1, 0, 1)),
    ('白', 'white',   (1, 1, 1)),
]
CHIP_ROWS = 15                   # pack_obs: h = 11 - Y//15 ⇒ 12 颗行驱, 每颗 15 行
N_CHIPS = H // CHIP_ROWS         # 12
STRIP_EDGES = [x0 for x0, _w, _b in pack_obs.STRIPS if x0 > 0]      # [53, 106]


def apply_marks(code, idx):
    """在纯色场上**挖黑**画标尺 (原地改), 返回 code。

    只挖黑、绝不加别的通道 —— 黑 = 不发光, 所以"红帧里只许红亮"这条串色判据
    一个字都不用改; 同时给出行驱芯片 / lane 条的刻度, 出问题能指认是第几段。
    """
    for k in range(1, N_CHIPS):                 # Y=15..165, 11 条
        code[k * CHIP_ROWS, :, :] = 0
    for x in STRIP_EDGES:                       # X=53 / 106
        code[:, x, :] = 0
    _stamp_digit(code, 2, 2, idx, 0)            # 左上角黑色序号
    return code


def build_colors8_seq(levels, marks):
    """→ [(标签, 码值图)] 播放序列: 每色按 levels 逐档, **同色相邻**。

    黑色的每一档都是全 0 ⇒ 黑帧长度自动 = len(levels) 倍, 正好当循环起点标记。
    """
    seq = []
    for idx, (cn, en, mask) in enumerate(COLORS8):
        for lv in levels:
            code = np.zeros((H, W, 3), np.uint8)
            for c in range(3):
                if mask[c]:
                    code[:, :, c] = lv
            if marks == 'rulers':
                apply_marks(code, idx)
            seq.append((f'{idx}:{en}@{lv}', code))
    return seq


def codes_via_to_3bit(code, gamma):
    """码值图 → (反解到输入域 → gas.to_3bit 就近取整) → 码值图。

    ⚠ 这不是多此一举: 项目约定**所有量化必须走 gas.to_3bit** (它管 gamma 解码
    + 残差 Bayer 抖动)。测试图要的是"就近取整"⇒ dither=False —— 抖动会把纯色
    打散成点阵, 看不出真实色阶。走一遍 to_3bit 还顺带成了量化器的**恒等自检**:
    v = (c/7)^(1/γ) 再 x = 7·v^γ = c, 取整必须原样回来, 回不来就是量化器坏了。
    """
    import gen_anime_slices as gas
    out = gas.to_3bit(codes_to_input(code, gamma), 128, False, 0, gamma=gamma)
    if not np.array_equal(out, code):
        bad = int((out != code).sum())
        raise AssertionError(f'to_3bit 恒等自检失败: {bad} 个像素码值对不上 '
                             f'(gamma={gamma})')
    return out


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


def _font(sz):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=sz)
    except TypeError:                      # 老 Pillow: load_default 不吃 size
        return ImageFont.load_default()


def save_png_seq(files, path, gamma, dwell, rpm, labels, bpp, scale=2, off=0):
    """colors8 序列预览: **从落盘的 .bin 反解**回观察者视角 + 一条切换节奏时间轴。

    🔴 刻意从磁盘上的字节读回来 (而不是内存里的 code 数组): 要验的是
    "真正会发给板子的那串字节", 打包/位平面/padding 全在里面。

    off: 从第几片开始取 (--blank-face a 时面A 整片是黑的, 要看面B 那一半)。
    """
    from PIL import Image, ImageDraw
    stride = pack_obs.slice_stride(bpp)
    tiles, mean_rgb = [], []
    for f in files:
        raw = open(f, 'rb').read()
        code = pack_obs.unpack_slice(raw[off * stride:(off + 1) * stride], bpp=bpp)
        if bpp == 1:
            code = code.astype(np.uint8) * (LEVELS - 1)
        px = np.clip(codes_to_input(code, gamma), 0, 255).astype(np.uint8)
        tiles.append(Image.fromarray(px).resize((W * scale, H * scale), Image.NEAREST))
        # 时间轴用的色: 取整幅的最大值 (标尺黑线不该把色条拉暗)
        mean_rgb.append(tuple(int(v) for v in px.reshape(-1, 3).max(axis=0)))

    n = len(tiles)
    cols = len(COLORS8)                       # 8 列 = 8 种颜色
    rows = (n + cols - 1) // cols             # 行 = 档位数
    tw, th = W * scale, H * scale
    padx, lab_h, top_h, tl_h = 6, 20, 26, 76
    cw, ch = tw + padx, th + lab_h + padx
    cvw = cols * cw + padx
    cvh = top_h + rows * ch + tl_h + padx
    canvas = Image.new('RGB', (cvw, cvh), (18, 18, 18))
    d = ImageDraw.Draw(canvas)
    f_lab, f_ttl = _font(15), _font(17)

    d.text((padx, 6), f'colors8  {n} frames  |  dwell {dwell * 1e3:.0f} ms/frame '
                      f'= {dwell * rpm / 60.0:.1f} rev @ {rpm:.0f} RPM  |  '
                      f'cycle {n * dwell:.1f} s  |  --fps {1.0 / dwell:g}',
           fill=(230, 230, 230), font=f_ttl)
    # ⚠ 序列是"按颜色分组、档位相邻"排的 (i = color*rows + level), 拼图要按
    #    列=颜色 / 行=档位 放, 所以这里不是简单的行优先。
    for i, (t, lb) in enumerate(zip(tiles, labels)):
        c, r = divmod(i, rows)
        x0, y0 = padx + c * cw, top_h + r * ch
        canvas.paste(t, (x0, y0))
        d.rectangle([x0 - 1, y0 - 1, x0 + tw, y0 + th], outline=(70, 70, 70))
        d.text((x0 + 2, y0 + th + 3), lb, fill=(200, 200, 200), font=f_lab)

    # ---- 时间轴: 按**播放顺序**排的色条, 看得出切换节奏和黑帧双倍长 ----
    ty = top_h + rows * ch + 8
    bx0, bx1 = padx, cvw - padx
    seg = (bx1 - bx0) / float(n)
    d.text((bx0, ty - 2), 'play order (time ->)', fill=(160, 160, 160), font=f_lab)
    for i, col in enumerate(mean_rgb):
        a, b = bx0 + i * seg, bx0 + (i + 1) * seg
        d.rectangle([a, ty + 16, b - 1, ty + 46], fill=col)
        d.rectangle([a, ty + 16, b - 1, ty + 46], outline=(90, 90, 90))
        if i % max(1, n // 8) == 0:
            d.text((a + 2, ty + 49), f'{i * dwell:.1f}s',
                   fill=(160, 160, 160), font=f_lab)
    canvas.save(path)
    return canvas.size


def main():
    ap = argparse.ArgumentParser(description='8 级灰度楔 / 8 色循环屏测 (3-bit 上板目视)')
    ap.add_argument('--bpp', type=int, choices=sorted(pack_obs.BPP_MODES), default=3)
    ap.add_argument('--pattern', choices=['flat', 'checker', 'grid64', 'rgb3', 'ab', 'halftest', 'levels', 'full', 'bw', 'rings', 'ramp', 'colors8'], default='flat')
    ap.add_argument("--n-slices", type=int, default=50,
                    help='**每面**片数 (现役配置 142; 帧总片数 = 本值 × --faces)')
    ap.add_argument('--gamma', type=float, default=DEFAULT_GAMMA,
                    help=f'反 gamma (默认 {DEFAULT_GAMMA}), 只影响 1-bit 对照组/预览/ramp; '
                         'flat/rings 的 3-bit 码值是**直接写死的 0..7**, 不过 gamma')
    ap.add_argument('--out-dir', default=None,
                    help='预渲染目录 (默认 stream/pc/frames_wedge<bpp>, colors8 → frames_colors8)')
    ap.add_argument('--png', default=None, metavar='PATH', help='另存预览 PNG (需 PIL)')
    # ---- colors8 专用 ----
    ap.add_argument('--levels', default='7,3', metavar='C[,C...]',
                    help='colors8: 每色走哪几个码值档 (默认 7,3 = 满量程 + 只开低两个 '
                         'plane; 两者恰好差一个 plane0/MSB ⇒ 顺带验 MSB 位平面)')
    ap.add_argument('--marks', choices=['none', 'rulers'], default='rulers',
                    help='colors8: rulers = 在纯色上挖黑画芯片/lane 刻度 + 序号 '
                         '(挖黑不发光, 不影响串色判据); none = 完全纯色 (查坏点最干净)')
    ap.add_argument('--faces', type=int, choices=(1, 2), default=2,
                    help='1 = 只驱动穿心面 A (geom_flags=0); '
                         '2 = 双面 DUAL_FACE (两块物理屏一起测, 默认)')
    ap.add_argument('--blank-face', choices=['none', 'a', 'b'], default='none',
                    help='--faces 2 时把其中一面整片压黑 —— 几何/时序与双面完全一致, '
                         '只有一块屏亮 ⇒ 用来判定坏点长在 A 屏还是 B 屏')
    ap.add_argument('--dwell', type=float, default=0.5, metavar='S',
                    help='colors8: 每帧停留秒数 (默认 0.5 s ≈ 8 转 @969RPM); '
                         '推流 fps = 1/dwell, 只用于算推流命令与预览标注')
    ap.add_argument('--rpm', type=float, default=969.0, help='转速, 只用于预览标注')
    a = ap.parse_args()

    stride = pack_obs.slice_stride(a.bpp)
    faces = a.faces if a.pattern == 'colors8' else 1
    if a.blank_face != 'none' and faces != 2:
        sys.exit('--blank-face 需要 --faces 2')
    total_sl = a.n_slices * faces
    # 上限走 pack_obs.FRAME_RAW_MAX (host 侧唯一定义处), 别再抄字面量
    ns_max = pack_obs.FRAME_RAW_MAX // (stride * faces)
    if not 1 <= a.n_slices <= ns_max:
        sys.exit(f'--n-slices 须在 1..{ns_max} (bpp={a.bpp} 片距 0x{stride:X}, '
                 f'{faces} 面)')

    out = a.out_dir or os.path.join(HERE, '..', 'stream', 'pc',
                                    'frames_colors8' if a.pattern == 'colors8'
                                    else f'frames_wedge{a.bpp}')
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)

    # ---------------- 多帧路径: colors8 ----------------
    if a.pattern == 'colors8':
        try:
            levels = [int(x) for x in a.levels.split(',') if x.strip() != '']
        except ValueError:
            sys.exit(f'--levels {a.levels!r} 解析失败')
        if not levels or any(not 0 <= v <= LEVELS - 1 for v in levels):
            sys.exit(f'--levels 每档须在 0..{LEVELS - 1}')
        seq = build_colors8_seq(levels, a.marks)
        # 🔴 量化统一走 gas.to_3bit(dither=False) —— 见 codes_via_to_3bit 的说明
        if a.bpp == 3:
            seq = [(lb, codes_via_to_3bit(c, a.gamma)) for lb, c in seq]
        blank = np.zeros((H, W, 3), np.uint8)
        files, labels = [], []
        for i, (lb, code) in enumerate(seq):
            fa = blank if a.blank_face == 'a' else code
            raw = pack_frame(fa, a.bpp, a.n_slices, a.gamma)
            if faces == 2:
                fb = blank if a.blank_face == 'b' else code
                raw += pack_frame(fb, a.bpp, a.n_slices, a.gamma)
            assert len(raw) == total_sl * stride, (len(raw), total_sl * stride)
            # 每片必须逐字节相同 (内容与转角无关) —— 打包链路的廉价自检。
            # bpp=1 不成立: 那条路每片换一次 Bayer 相位, 本来就该不一样。
            if a.bpp == 3:
                assert raw[:stride] == raw[(a.n_slices - 1) * stride:
                                           a.n_slices * stride]
            fp = os.path.join(out, f'frame_{i:04d}.bin')
            with open(fp, 'wb') as f:
                f.write(raw)
            files.append(fp)
            labels.append(lb)

        geom = 0x0008 if faces == 2 else 0                      # FLAG_DUAL_FACE
        face_meta = [{'name': 'A', 'axis_off_px': 0.0, 'n_slices': a.n_slices}]
        if faces == 2:
            face_meta.append({'name': 'B', 'axis_off_px': 0.0,
                              'n_slices': a.n_slices})
        meta = {'anim': 'colors8', 'frames': len(seq), 'render_slices': a.n_slices,
                'frame_raw': total_sl * stride, 'freeze_phase': True,
                'bpp': a.bpp, 'led_gamma': a.gamma if a.bpp == 3 else None,
                'dark_floor': False if a.bpp == 3 else None,
                'n_slices': total_sl, 'geom_flags': geom, 'faces': face_meta,
                'colors8': {'order': [c[1] for c in COLORS8], 'levels': levels,
                            'marks': a.marks, 'blank_face': a.blank_face,
                            'dwell_s': a.dwell, 'sequence': labels},
                'note': 'gen_wedge.py --pattern colors8 屏幕测试图 (每片同图, '
                        '与转角无关; 停转直视 / 转起来看都用这一份)'}
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1, ensure_ascii=False)

        print(f'[colors8] {len(seq)} 帧 = {len(COLORS8)} 色 × 档位 {levels}'
              f'{"" if a.marks == "none" else " + rulers 刻度"}'
              f'{"" if a.blank_face == "none" else f" (面{a.blank_face.upper()} 压黑)"}')
        print(f'[colors8] 每帧 {total_sl} 片 ({faces} 面 × {a.n_slices}) '
              f'× 0x{stride:X} = {total_sl * stride}B → {out}')
        print(f'[colors8] 节奏: dwell {a.dwell * 1e3:.0f} ms/帧 = '
              f'{a.dwell * a.rpm / 60.0:.1f} 转 @{a.rpm:.0f} RPM, '
              f'整轮 {len(seq) * a.dwell:.1f} s')
        if a.png:
            sz = save_png_seq(files, a.png, a.gamma, a.dwell, a.rpm, labels, a.bpp,
                              off=a.n_slices if a.blank_face == 'a' else 0)
            print(f'[colors8] 预览 {a.png} ({sz[0]}x{sz[1]})')
        print(f'[colors8] 推流: python3 stream/pc/povstream.py stream --dir {out} '
              f'--host <board> --bpp {a.bpp} --n-slices {a.n_slices} '
              f'--fps {1.0 / a.dwell:g} --loop --codec lz4 '
              f'--stream-split even --stream-workers 3')
        return

    # ---------------- 单帧路径 (原有图案, 行为不变) ----------------
    code = (build_ramp_codes(a.gamma, 0) if a.pattern == 'ramp'
            else build_codes(a.pattern))
    raw = pack_frame(code, a.bpp, a.n_slices, a.gamma)
    assert len(raw) == a.n_slices * stride, (len(raw), a.n_slices * stride)

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
