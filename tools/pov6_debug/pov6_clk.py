#!/usr/bin/env python3
# FCLK0 运行时切换 (SLCR 0xF8000170) — 免 rebuild 的 aclk 超频/回退
# usage: pov6_clk.py [50|62|59]   50=1000/20 基线 / 62=62.5M (DCLK 31.25M) / 59=59.26M (DDR PLL/18)
# 顺序: 停引擎 → 切时钟 → 需重跑 pov6_chess.py 重配点亮
import mmap, os, sys, struct

sel = sys.argv[1] if len(sys.argv) > 1 else '50'
VAL = {'50': 0x00400500,   # IO PLL 1000M / (5*4) = 50.0M   (出厂值)
       '62': 0x00400400,   # IO PLL 1000M / (4*4) = 62.5M   (DCLK 31.25M)
       '59': 0x00101230}   # DDR PLL 1066.7M / 18 = 59.26M  (DCLK 29.63M)
v = VAL[sel]

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
# 先停 panel 引擎 (0x40010000 + 0x0C auto 停)
pp = mmap.mmap(f, 4096, offset=0x40010000)
pp[0x0C:0x10] = (0xC1000000).to_bytes(4, 'little')
# SLCR unlock + 写 FPGA0_CLK_CTRL
ps = mmap.mmap(f, 4096, offset=0xF8000000)
ps[0x008:0x00C] = (0x0000DF0D).to_bytes(4, 'little')   # SLCR_UNLOCK
old = struct.unpack('<I', ps[0x170:0x174])[0]
ps[0x170:0x174] = v.to_bytes(4, 'little')
new = struct.unpack('<I', ps[0x170:0x174])[0]
print('FPGA0_CLK_CTRL 0x%08X -> 0x%08X (%s)' % (old, new, sel), flush=True)
print('引擎已停, 重跑 pov6_chess.py 重配点亮', flush=True)
