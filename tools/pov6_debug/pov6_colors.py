#!/usr/bin/env python3
# v6 三色循环 (板上 root): POV 关, fb 直灌纯色, auto 引擎扫描
# usage: pov6_colors.py [rounds] [hold_s] [slow]   (slow = ddr_slow 25Mbps 降级档)
import mmap, os, sys, time

rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
hold   = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
slow   = 'slow' in sys.argv[3:]

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')
def pr(off):    return int.from_bytes(p[off:off+4], 'little')

def fb_fill(lanes_on, sel_b):
    # fb 窗: awaddr[15]=1, [14:11]=lane, [10:2]={row,pair}; fb_sel_b 决定 A/B
    ctrl = (360<<16)|0x4
    if sel_b: ctrl |= 0x8
    pw(0x10, ctrl)
    for lane in range(9):
        word = 0xFFFFFFFF if lane in lanes_on else 0
        base = 0x8000 | (lane << 11)
        for i in range(512):
            pw(base | (i << 2), word)

cfg = 0x98360801   # oe=8沿: 共阳屏全屏实心电流红线 | (0x20000000 if slow else 0)
pw(0x10,(360<<16)|0x4)   # dual_en 常开 (屏可能在 P3=B 引擎)
pw(0x0C, 0x000001FF)      # sdi_mask
pw(0x0C, cfg)             # 54 行 / oe 48 沿 / 双沿或slow
COLORS = [('RED', {0,3,6}), ('GREEN', {1,4,7}), ('BLUE', {2,5,8})]
print('cfg=0x%08X %s' % (cfg, 'SLOW-25M' if slow else 'FAST-50M'), flush=True)

for r in range(rounds):
    for name, lanes in COLORS:
        fb_fill(lanes, 0)                     # 屏 A
        fb_fill(lanes, 1)                     # 屏 B
        pw(0x10,(360<<16)|0x4)             # fb_sel 回 A
        pw(0x0C, 0xC1000003)                  # auto_en + use_fb
        print('[%d] %s  status0=0x%08X' % (r, name, pr(0x00)), flush=True)
        time.sleep(hold)
pw(0x0C, 0xC1000000)      # auto 停
print('COLORS_DONE status0=0x%08X' % pr(0x00), flush=True)
