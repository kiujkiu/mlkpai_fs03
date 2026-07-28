#!/usr/bin/env python3
# 设定单色并保持 (退出不熄屏) — 用于逐色核对 lane 映射
# usage: pov6_hold.py <RED|GREEN|BLUE|YELLOW|MAGENTA|CYAN|WHITE|L0..L8|OFF> [oe沿数]
import mmap, os, sys

name = (sys.argv[1] if len(sys.argv) > 1 else 'RED').upper()
oe   = int(sys.argv[2]) if len(sys.argv) > 2 else 48

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(o, v): p[o:o+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')
def pr(o):    return int.from_bytes(p[o:o+4], 'little')

NAMED = {'RED':{0,3,6}, 'GREEN':{1,4,7}, 'BLUE':{2,5,8},
         'YELLOW':{0,3,6,1,4,7}, 'MAGENTA':{0,3,6,2,5,8},
         'CYAN':{1,4,7,2,5,8}, 'WHITE':set(range(9)), 'OFF':set()}
if name.startswith('L') and name[1:].isdigit():
    lanes = {int(name[1:])}            # 单 lane 点名, 用于定位第几根数据线
else:
    lanes = NAMED[name]

CTRL = (360 << 16) | 0x4               # POV 关, dual_en 开
def fb_fill(sel_b):
    pw(0x10, CTRL | (0x8 if sel_b else 0))
    for lane in range(9):
        w = 0xFFFFFFFF if lane in lanes else 0
        base = 0x8000 | (lane << 11)
        for i in range(512):
            pw(base | (i << 2), w)

cfg = 0x98360001 | ((oe & 0xFF) << 8)
pw(0x10, CTRL)
pw(0x0C, 0x000001FF)                   # sdi_mask 全开
pw(0x0C, cfg)                          # rows/oe/双沿
fb_fill(0); fb_fill(1)                 # A/B 两个 fb 都灌
pw(0x10, CTRL)                         # fb_sel 回 A
pw(0x0C, 0xC1000003)                   # auto_en + use_fb —— 退出后保持
print('HOLD %s lanes=%s oe=%d status0=0x%08X' %
      (name, sorted(lanes), oe, pr(0x00)), flush=True)
