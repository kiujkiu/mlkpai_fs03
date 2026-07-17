#!/usr/bin/env python3
# SSO 定罪实验: 运行时改 sdi_mask (哪些数据 lane 参与翻转) + 速度档
# usage: pov6_mask.py <mask_hex> [slow]   例: pov6_mask.py 1 = fast+仅lane0 / 1ff = 全开
import mmap, os, sys

mask = int(sys.argv[1], 16) if len(sys.argv) > 1 else 0x1FF
slow = 'slow' in sys.argv[2:]
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')

pw(0x0C, 0xC1000000)                             # auto 停
pw(0x0C, 0x00000000 | (mask & 0x1FF))            # sdi_mask
pw(0x0C, 0x98360801 | (0x20000000 if slow else 0))
pw(0x0C, 0xC1000003)                             # auto+use_fb 重开
print('MASK=0x%03X %s' % (mask, 'SLOW-25M' if slow else 'FAST-50M'), flush=True)
