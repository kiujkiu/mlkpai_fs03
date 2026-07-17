#!/usr/bin/env python3
# 棋盘格终验加载器: 11664B fb bin (lane*1296 + row*24 + word*4) 灌双 fb, auto+use_fb 点亮
# usage: pov6_chess.py [bin_path] [oe沿数] [slow]   默认 chess_fb.bin / oe=8 / 双沿50M
#   slow = ddr_slow 25Mbps 单沿降级 (SI 裕量实验档)
import mmap, os, sys

path = sys.argv[1] if len(sys.argv) > 1 else '/home/uisrc/chess_fb.bin'
oe   = int(sys.argv[2]) if len(sys.argv) > 2 else 8
slow = 'slow' in sys.argv[3:]
data = open(path, 'rb').read()
assert len(data) == 11664, 'bin 必须 11664B'

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')
def pr(off):    return int.from_bytes(p[off:off+4], 'little')

def fb_load(sel_b):
    ctrl = (360 << 16) | 0x4
    if sel_b: ctrl |= 0x8
    pw(0x10, ctrl)
    for lane in range(9):
        for r in range(54):
            for wd in range(6):
                o = lane*1296 + r*24 + wd*4
                v = int.from_bytes(data[o:o+4], 'little')
                pw(0x8000 | (lane << 11) | (r * 0x20) | (wd * 4), v)

cfg = 0x98360001 | ((oe & 0xFF) << 8) | (0x20000000 if slow else 0)
pw(0x10, (360 << 16) | 0x4)      # POV 关, dual_en
pw(0x0C, 0x000001FF)             # sdi_mask
pw(0x0C, cfg)                    # 54行 / oe / 双沿
fb_load(0); fb_load(1)           # 屏在 P3=B 引擎, 双 fb 都灌
pw(0x10, (360 << 16) | 0x4)      # fb_sel 回 A
pw(0x0C, 0xC1000003)             # auto_en + use_fb
print('CHESS_ON %s oe=%d cfg=0x%08X status0=0x%08X' % (path, oe, cfg, pr(0x00)), flush=True)
