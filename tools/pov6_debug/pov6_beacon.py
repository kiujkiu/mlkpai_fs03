#!/usr/bin/env python3
# OE 信标: auto 停, OE 2Hz 方波 (P1.10 + P3.10 同时), Ctrl-C 停
import mmap, os, time
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000)
def pw(off, v): p[off:off+4] = v.to_bytes(4, 'little')
pw(0x0C, 0xC1000000)          # auto 停 (其他线全静)
pw(0x0C, 0x000001FF)
print('BEACON: OE 2Hz, 其余全静', flush=True)
while True:
    pw(0x0C, 0x80000000); time.sleep(0.25)   # OE=0
    pw(0x0C, 0x80000001); time.sleep(0.25)   # OE=1
