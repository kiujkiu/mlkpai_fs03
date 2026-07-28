#!/usr/bin/env python3
# 相位扫描: 0x24 [20]=dclk_inv [22:21]=data_dly [24:23]=le_dly
# usage: pov6_phase.py sweep [hold_s]   — 12 常用组合轮播
#        pov6_phase.py set <inv> <ddly> <ldly>  — 锁定一组
import mmap, os, sys, time

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')

def setph(inv, ddly, ldly):
    v = (inv << 20) | (ddly << 21) | (ldly << 23)
    pw(0x24, v)
    return v

if sys.argv[1] == 'set':
    inv, ddly, ldly = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    v = setph(inv, ddly, ldly)
    print('0x24=0x%08X inv=%d data_dly=%d le_dly=%d' % (v, inv, ddly, ldly))
else:
    hold = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    COMBOS = [(0,0,0),(1,0,0),(0,1,0),(1,1,0),(0,1,1),(1,1,1),
              (0,2,1),(1,2,1),(0,2,2),(1,2,2),(0,0,1),(1,0,1)]
    for k,(inv,dd,ld) in enumerate(COMBOS):
        v = setph(inv, dd, ld)
        print('组合 %2d: inv=%d data_dly=%d le_dly=%d (0x24=0x%08X) — 看边界!'
              % (k, inv, dd, ld, v), flush=True)
        time.sleep(hold)
    setph(0,0,0)
    print('SWEEP_DONE 回默认. 哪组编号最干净?', flush=True)
