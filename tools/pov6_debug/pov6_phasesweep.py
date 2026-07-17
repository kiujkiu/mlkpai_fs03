#!/usr/bin/env python3
# 输出相位扫描: 0x24 高位旋钮 [20]dclk反相 [22:21]数据平移拍 [24:23]LE平移拍
# 治首/尾 bit 装载错位 (CLK 振铃假沿案, 2026-07-16)。
# 先灌好棋盘格再跑 — 本脚本只动 0x24 高位, 不碰 fb/引擎, 盯棋盘边界列!
# usage: pov6_phasesweep.py [hold_s] [full] [row_cfg_low_hex]
#   默认 8 组合 (LE 跟随数据拍); full = 32 组合 (LE 独立扫)
import mmap, os, sys, time

hold = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
full = len(sys.argv) > 2 and sys.argv[2] == 'full'
low  = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0   # 行驱 cfg 低位保持
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')

combos = []
for inv in range(2):
    for d in range(4):
        if full:
            for le in range(4):
                combos.append((inv, d, le))
        else:
            combos.append((inv, d, d))    # LE 跟随数据拍

print('相位扫描 %d 组合 × %.0fs — 盯棋盘边界列/数字!' % (len(combos), hold), flush=True)
for i, (inv, d, le) in enumerate(combos):
    cfg = low | (inv << 20) | (d << 21) | (le << 23)
    pw(0x24, cfg)
    print('combo %2d: 0x24=0x%08X  dclk_inv=%d data_dly=%d le_dly=%d'
          % (i, cfg, inv, d, le), flush=True)
    time.sleep(hold)
pw(0x24, low)
print('SWEEP_DONE (0x24 回 0x%08X). 哪个 combo 边界干净?' % low, flush=True)
