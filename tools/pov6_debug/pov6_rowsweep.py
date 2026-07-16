#!/usr/bin/env python3
# 1028 行驱极性扫描: 16 组合 × hold 秒, 全屏红 auto 扫描下逐组切
# 用户盯屏: 任何组合亮了立刻记下编号!
# usage: pov6_rowsweep.py [hold_s]
import mmap, os, sys, time

hold = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')

# 全屏红灌 A/B 两 fb
def fb_fill_red(sel_b):
    pw(0x10, ((360 << 16) | 0x4) | (0x8 if sel_b else 0))
    for lane in range(9):
        w = 0xFFFFFFFF if lane in (0, 3, 6) else 0
        base = 0x8000 | (lane << 11)
        for i in range(512):
            pw(base | (i << 2), w)

pw(0x10, (360 << 16) | 0x4)     # dual_en, pov 关
pw(0x0C, 0x000001FF)
pw(0x0C, 0x98363001)            # 54行/oe48/双沿
fb_fill_red(0); fb_fill_red(1)
pw(0x10, (360 << 16) | 0x4)
pw(0x0C, 0xC1000003)            # auto 扫描开

print('行驱极性扫描: 16 组合 × %.0fs — 盯屏!' % hold, flush=True)
print('combo = [19]sdi反相 [18]lck反 [17]dclk反 [16]bk电平', flush=True)
for combo in range(16):
    cfg = ((combo & 8) << 16) | ((combo & 4) << 16) | ((combo & 2) << 16) | ((combo & 1) << 16)
    pw(0x24, cfg)
    print('combo %2d: 0x24=0x%08X  sdi_inv=%d lck_inv=%d dclk_inv=%d bk=%d'
          % (combo, cfg, (combo >> 3) & 1, (combo >> 2) & 1, (combo >> 1) & 1, combo & 1), flush=True)
    time.sleep(hold)
pw(0x24, 0)
print('SWEEP_DONE (0x24 回默认). 哪个 combo 亮了?', flush=True)
