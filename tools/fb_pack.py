#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fb_pack.py — 把任意 PNG 打包成 ICND2049 屏 (160x180, 3 区 x 9 lane) 的
FPGA framebuffer BRAM 灌写脚本。

几何 (全部可用命令行参数覆盖, 方便上屏试错迭代):
  - 屏: 扫描轴 160 行 x 列轴 180。
  - 扫描轴分 3 区 (--sections 54,53,53), 区起始 y = 0 / 54 / 107;
    扫描行位 p (0..max_section-1) 同时点亮 3 个区各自的第 p 行。
  - 列轴 180 = 12 颗 ICND2049 级联 x 每颗 15 个有效通道 (16 通道浪费 1 个)。
  - 9 条 lane, 每 lane = (一个区, 一种颜色)。默认 lane = color*3 + section
    (color: 0=R 1=G 2=B), 用 --lane-order 改 (9 个 token, 第 i 个描述 lane i
    的内容, 如 r0 = 红色/区0)。
  - 每行每 lane 灌 12 个 16-bit word, 发送顺序编号 w=0..11。
    先发的 word 落在级联最远端 → 默认 w=0 对应 chip 11 (x 最大端),
    --first-word-chip {far,near} 切换 (near: w=0 对应 chip 0, x 最小端)。
  - word 内 bit15 先发, 且 bit15 = OUT15 (手册: 先传输的数据为寄存器高位),
    即 word 的 bit b 对应芯片 OUTb。
  - 每颗芯片 16 通道只用 15 个, dummy 通道 --dummy-ch (默认 15,
    即有效通道 = OUT0..14)。
  - 芯片内有效通道按 x 升序编号 k=0..14, k → OUT 映射默认升序,
    --ch-reverse 反向 (k=0 → 最大的有效 OUT)。
  - x → chip: chip = x // 15, 通道 k = x % 15。
  - --rotate {0,90,180,270}: 打包前对输入图顺时针旋转; 90/270 时输入为
    观察者视角 160宽x180高, 旋转后成 180x160 再走原打包流程。
  - --flip-x / --flip-y 整屏镜像 (旋转之后、映射之前作用于图像坐标)。

framebuffer 地址布局 (AXI 32-bit 写):
  addr = FB_BASE + lane*0x800 + row*0x20 + pair*4
       (FB_BASE=0x40018000, lane 0..8, row 0..53, pair 0..5)
  data32 = word(2*pair) | word(2*pair+1) << 16   (word 编号即发送顺序 w)

输出:
  <name>_fb.tcl — xsct 片段, 只含 mwr 批量行 (每行 6 个 32-bit word,
                  远小于 64 上限), 不含 connect/exit。
  <name>_fb.bin — 从 FB_BASE 起连续 9*0x800 字节的小端 32-bit dump
                  (行间 0x18/0x1C 空洞与行 53 之后区域补 0), 备用。

纯软件工具, 只生成文件, 不执行任何 xsct/JTAG 操作。
"""
import argparse
import os
import struct
import sys

from PIL import Image

# ---------- 固定几何常量 (布局变了才需要改) ----------
IMG_W, IMG_H = 180, 160        # 列轴 x 扫描轴
CH_PER_CHIP = 15               # 每颗有效通道数
N_CHIPS = IMG_W // CH_PER_CHIP  # 12
N_LANES = 9
FB_BASE = 0x40018000
LANE_STRIDE = 0x800
ROW_STRIDE = 0x20
FB_LO, FB_HI = 0x40018000, 0x4001FFFF   # 合法地址窗口 (自检用)

COLOR_IDX = {'r': 0, 'g': 1, 'b': 2}
COLOR_NAME = 'RGB'


def parse_args():
    ap = argparse.ArgumentParser(
        description='PNG -> ICND2049 framebuffer mwr tcl/bin 打包器',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('png', help='输入 PNG (自动缩放到 180x160)')
    ap.add_argument('-o', '--out-prefix', default=None,
                    help='输出前缀 (默认 = 输入文件名去扩展名)')
    ap.add_argument('--lane-order',
                    default='r0,r1,r2,g0,g1,g2,b0,b1,b2',
                    help='9 个 token, 第 i 个 = lane i 的 (颜色+区), '
                         '如 r0,g0,b0,r1,... 表示 lane0=R区0 lane1=G区0 ...')
    ap.add_argument('--sections', default='54,53,53',
                    help='3 个区的行数 (逗号分隔, 和必须 = 160)')
    ap.add_argument('--dummy-ch', type=int, default=15,
                    help='每颗芯片不用的 OUT 通道号 (0..15)')
    ap.add_argument('--ch-reverse', action='store_true',
                    help='芯片内通道 k (x 升序) -> OUT 映射反向')
    ap.add_argument('--first-word-chip', choices=['far', 'near'], default='far',
                    help='w=0 落在哪端: far = chip 11 (x 最大端), near = chip 0')
    ap.add_argument('--rotate', type=int, choices=[0, 90, 180, 270], default=0,
                    help='打包前对输入图做顺时针旋转的度数。90/270 时输入应为 '
                         '160宽x180高 (观察者视角), 旋转后成 180x160 再走原打包流程; '
                         '0/180 输入为 180x160')
    ap.add_argument('--flip-x', action='store_true', help='整屏水平镜像')
    ap.add_argument('--flip-y', action='store_true', help='整屏垂直镜像')
    ap.add_argument('--threshold', type=int, default=128,
                    help='1-bit 阈值 (通道值 >= 阈值算 1)')
    ap.add_argument('--fb-base', type=lambda s: int(s, 0), default=FB_BASE,
                    help='framebuffer 基址')
    ap.add_argument('--samples', type=int, default=6,
                    help='打印多少个抽样映射自检')
    return ap.parse_args()


def build_lane_map(spec):
    """解析 --lane-order → dict[(color,section)] = lane"""
    toks = [t.strip().lower() for t in spec.split(',')]
    if len(toks) != N_LANES:
        sys.exit(f"--lane-order 必须 9 个 token, 收到 {len(toks)}")
    m = {}
    for lane, tok in enumerate(toks):
        if len(tok) != 2 or tok[0] not in COLOR_IDX or tok[1] not in '012':
            sys.exit(f"--lane-order token 非法: {tok!r} (格式如 r0/g1/b2)")
        key = (COLOR_IDX[tok[0]], int(tok[1]))
        if key in m:
            sys.exit(f"--lane-order 重复 token: {tok}")
        m[key] = lane
    return m


def build_sections(spec):
    """解析 --sections → [(start,size),...] 并校验"""
    sizes = [int(t) for t in spec.split(',')]
    if len(sizes) != 3 or sum(sizes) != IMG_H:
        sys.exit(f"--sections 必须 3 段且和为 {IMG_H}, 收到 {sizes}")
    starts, acc = [], 0
    for sz in sizes:
        starts.append(acc)
        acc += sz
    if max(sizes) * ROW_STRIDE > LANE_STRIDE:
        sys.exit(f"区行数 {max(sizes)} 超过 lane 容量 {LANE_STRIDE//ROW_STRIDE}")
    return list(zip(starts, sizes))


def build_valid_outs(dummy_ch, reverse):
    """有效通道 k (x 升序 0..14) → OUT 编号表"""
    if not 0 <= dummy_ch <= 15:
        sys.exit("--dummy-ch 必须 0..15")
    outs = [o for o in range(16) if o != dummy_ch]   # 升序 15 个
    return outs[::-1] if reverse else outs


class Mapper:
    """像素 (x, y, color) → (lane, row, w, addr, bit) 的几何映射"""

    def __init__(self, args):
        self.lane_map = build_lane_map(args.lane_order)
        self.sections = build_sections(args.sections)
        self.valid_outs = build_valid_outs(args.dummy_ch, args.ch_reverse)
        self.first_far = (args.first_word_chip == 'far')
        self.fb_base = args.fb_base
        self.max_rows = max(sz for _, sz in self.sections)   # 54

    def section_of(self, y):
        for s, (start, sz) in enumerate(self.sections):
            if start <= y < start + sz:
                return s, y - start
        raise ValueError(f"y={y} 不在任何区内")

    def map(self, x, y, color):
        s, row = self.section_of(y)
        lane = self.lane_map[(color, s)]
        chip = x // CH_PER_CHIP                       # chip 0 在 x 最小端
        w = (N_CHIPS - 1 - chip) if self.first_far else chip   # 发送顺序编号
        k = x % CH_PER_CHIP                           # 芯片内有效通道 (x 升序)
        out = self.valid_outs[k]                      # OUT 编号 = 16-bit word 内 bit 位
        pair = w // 2
        addr = self.fb_base + lane * LANE_STRIDE + row * ROW_STRIDE + pair * 4
        bit32 = out + (16 if (w & 1) else 0)          # 32-bit 数据里的 bit 位
        return lane, row, w, out, addr, bit32


def main():
    args = parse_args()
    mp = Mapper(args)

    # ---------- 读图 + 缩放 + 旋转 + 翻转 + 阈值化 ----------
    im = Image.open(args.png).convert('RGB')
    # 旋转 90/270 时输入是观察者视角 160x180, 先缩放到该尺寸再顺时针转成 180x160
    in_size = (IMG_H, IMG_W) if args.rotate in (90, 270) else (IMG_W, IMG_H)
    if im.size != in_size:
        im = im.resize(in_size, Image.LANCZOS)
        print(f"resize -> {in_size[0]}x{in_size[1]}")
    if args.rotate:
        # PIL transpose ROTATE_* 是逆时针; 顺时针 r 度 = 逆时针 (360-r) 度
        cw = {90: Image.ROTATE_270, 180: Image.ROTATE_180, 270: Image.ROTATE_90}
        im = im.transpose(cw[args.rotate])
        print(f"rotate {args.rotate} CW -> {im.size[0]}x{im.size[1]}")
    assert im.size == (IMG_W, IMG_H)
    if args.flip_x:
        im = im.transpose(Image.FLIP_LEFT_RIGHT)
    if args.flip_y:
        im = im.transpose(Image.FLIP_TOP_BOTTOM)
    pix = im.load()

    # ---------- 打包: words[lane][row][w] = 16-bit ----------
    words = [[[0] * N_CHIPS for _ in range(mp.max_rows)] for _ in range(N_LANES)]
    lit = 0
    for y in range(IMG_H):
        for x in range(IMG_W):
            rgb = pix[x, y]
            for c in range(3):
                if rgb[c] < args.threshold:
                    continue
                lane, row, w, out, _, _ = mp.map(x, y, c)
                words[lane][row][w] |= (1 << out)
                lit += 1

    # ---------- 输出文件 ----------
    prefix = args.out_prefix or os.path.splitext(args.png)[0]
    tcl_path = prefix + '_fb.tcl'
    bin_path = prefix + '_fb.bin'

    # tcl: 每 (lane,row) 一行 mwr, 6 个 32-bit word (<= 64 上限)
    lines = []
    addr_min, addr_max = 0xFFFFFFFF, 0
    for lane in range(N_LANES):
        for row in range(mp.max_rows):
            vals = []
            for pair in range(N_CHIPS // 2):          # pair 0..5
                v = words[lane][row][2 * pair] | (words[lane][row][2 * pair + 1] << 16)
                vals.append(v)
            addr = args.fb_base + lane * LANE_STRIDE + row * ROW_STRIDE
            addr_min = min(addr_min, addr)
            addr_max = max(addr_max, addr + (len(vals) - 1) * 4)
            lines.append('mwr -force -size w 0x%08X {%s} %d'
                         % (addr, ' '.join('0x%08X' % v for v in vals), len(vals)))
    with open(tcl_path, 'w') as f:
        f.write('# generated by fb_pack.py — 只含 mwr, 需外部 connect/targets\n')
        f.write('# args: %s\n' % ' '.join(sys.argv[1:]))
        f.write('\n'.join(lines) + '\n')

    # bin: FB_BASE 起连续 9*0x800 字节, 小端 32-bit, 空洞补 0
    buf = bytearray(N_LANES * LANE_STRIDE)
    for lane in range(N_LANES):
        for row in range(mp.max_rows):
            for pair in range(N_CHIPS // 2):
                v = words[lane][row][2 * pair] | (words[lane][row][2 * pair + 1] << 16)
                off = lane * LANE_STRIDE + row * ROW_STRIDE + pair * 4
                buf[off:off + 4] = struct.pack('<I', v)
    with open(bin_path, 'wb') as f:
        f.write(buf)

    # ---------- 自检 ----------
    ok = FB_LO <= addr_min and addr_max <= FB_HI
    print(f"tcl: {tcl_path}  ({len(lines)} mwr lines, 6 words/line)")
    print(f"bin: {bin_path}  ({len(buf)} bytes)")
    print(f"lit bits: {lit}")
    print(f"addr range: 0x{addr_min:08X} .. 0x{addr_max:08X}  "
          f"(window 0x{FB_LO:08X}..0x{FB_HI:08X}) -> {'OK' if ok else 'OUT OF RANGE!'}")
    if not ok:
        sys.exit(1)

    # 抽样映射打印: 四角 + 中心附近
    samples = [(0, 0, 0), (179, 0, 0), (0, 159, 2), (179, 159, 2),
               (90, 80, 1), (14, 53, 0), (15, 54, 1), (165, 107, 2)]
    print("\nsample (x,y,color) -> lane/row/w/OUT/addr/bit32:")
    for (x, y, c) in samples[:max(args.samples, 1)]:
        lane, row, w, out, addr, bit32 = mp.map(x, y, c)
        print(f"  ({x:3d},{y:3d},{COLOR_NAME[c]}) -> lane={lane} row={row:2d} "
              f"w={w:2d} OUT{out:<2d} addr=0x{addr:08X} bit={bit32:2d}")


if __name__ == '__main__':
    main()
