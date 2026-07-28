#!/usr/bin/env python3
# v6 七色循环 — oe_window 可调版 (板上 root)
# usage: pov6_colors2.py [rounds] [hold_s] [oe沿数] [slow]
#   oe 默认 48 沿 (25% 占空)。⚠ 全屏实心时电流最坏:
#   共阳屏红线 ≤8 沿; 双屏全白 25% 占空已顶接口板 TPS54560 5A 上限 —— 从小往大试。
import mmap, os, sys, time

rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 2
hold   = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
oe     = int(sys.argv[3]) if len(sys.argv) > 3 else 48
slow   = 'slow' in sys.argv[4:]

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(o, v): p[o:o+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')
def pr(o):    return int.from_bytes(p[o:o+4], 'little')

CTRL = (360 << 16) | 0x4          # POV 关, dual_en 开 (A/B 两个引擎都跑)
def fb_fill(lanes, sel_b):
    pw(0x10, CTRL | (0x8 if sel_b else 0))
    for lane in range(9):
        w = 0xFFFFFFFF if lane in lanes else 0
        base = 0x8000 | (lane << 11)
        for i in range(512):
            pw(base | (i << 2), w)

cfg = 0x98360001 | ((oe & 0xFF) << 8) | (0x20000000 if slow else 0)
pw(0x10, CTRL)
pw(0x0C, 0x000001FF)              # sdi_mask 全开
pw(0x0C, cfg)
COLORS = [('RED',{0,3,6}), ('GREEN',{1,4,7}), ('BLUE',{2,5,8}),
          ('YELLOW',{0,3,6,1,4,7}), ('MAGENTA',{0,3,6,2,5,8}),
          ('CYAN',{1,4,7,2,5,8}), ('WHITE',set(range(9)))]
print('cfg=0x%08X oe=%d沿 %s' % (cfg, oe, 'SLOW-25M' if slow else 'FAST-50M'), flush=True)
try:
    for r in range(rounds):
        for name, lanes in COLORS:
            fb_fill(lanes, 0); fb_fill(lanes, 1)
            pw(0x10, CTRL)
            pw(0x0C, 0xC1000003)
            print('[%d] %-8s status0=0x%08X' % (r, name, pr(0x00)), flush=True)
            time.sleep(hold)
finally:
    # 退出保留最后一帧 (不写 0xC1000000), 熄屏用 pov6_hold.py OFF
    print('COLORS_DONE status0=0x%08X' % pr(0x00), flush=True)
