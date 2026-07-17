#!/usr/bin/env python3
# 行边界静默区 + 相位旋钮 (0x24 组合写, 运行时生效)
# usage: pov6_qgap.py <gap拍数 0-63> [inv] [ddly] [ldly]
#   gap: LE尾→OE落 与 OE落→下行突发 各插 gap 拍死区 (每行代价 2×gap 拍)
import mmap, os, sys

gap  = int(sys.argv[1]) if len(sys.argv) > 1 else 16
inv  = int(sys.argv[2]) if len(sys.argv) > 2 else 0
ddly = int(sys.argv[3]) if len(sys.argv) > 3 else 0
ldly = int(sys.argv[4]) if len(sys.argv) > 4 else 0
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000)
v = ((gap & 0x3F) << 25) | ((ldly & 3) << 23) | ((ddly & 3) << 21) | ((inv & 1) << 20)
p[0x24:0x28] = v.to_bytes(4, 'little')
print('0x24=0x%08X  qgap=%d inv=%d ddly=%d ldly=%d' % (v, gap, inv, ddly, ldly), flush=True)
