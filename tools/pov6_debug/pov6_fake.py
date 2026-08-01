#!/usr/bin/env python3
# v6 双屏 2047 fake 首点脚本 (板上 root 跑): usage pov6_fake.py [single|dual|off] [rps]
import mmap, os, sys, time

mode = sys.argv[1] if len(sys.argv) > 1 else 'single'
rps  = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000)
def pw(off, v): p[off:off+4] = v.to_bytes(4, 'little')
def pr(off):    return int.from_bytes(p[off:off+4], 'little')

if mode == 'off':
    pw(0x10, 0)
    print('POV off'); sys.exit(0)

# 引擎配置 (v6 语义: bit29=ddr_slow=0 双沿快速, bit28=overlap(核忽略), bit27=cfg_we,
#            rows=54, oe_window=48 沿 = 25% 占空 — 供电裁决默认, 别乱调大)
pw(0x0C, 0x000001FF)      # sdi_mask 全 lane
pw(0x0C, 0x98363001)      # cfg: 54 行 / oe 48 沿 / 双沿
pw(0x0C, 0xC1000003)      # auto_en + use_fb
pw(0x28, 0)               # slice_base_b = 0 → 屏B 回落用 0x18 (v3.1 前的共享基址模式)
                          #  ⚠ 必须先清, 否则上一次双面运行残留的面B 基址会让屏B 读错数据
pw(0x18, 0x10000000)      # slice_base = bank A (pov_boot 预载 bonsai)
fake_period = int(50e6 / (rps * 360))
pw(0x14, fake_period)
ctrl = (360 << 16) | 0x3                 # pov_en + fake_en
if mode == 'dual': ctrl |= 0x4           # dual_en
pw(0x10, ctrl)
print('CTRL=0x%08X fake_period=%d (%.2f rps)' % (ctrl, fake_period, rps))

# 观测: slice_idx 应在走; dual 时 0x1C 回读 idx_B 相位差 180
for k in range(5):
    time.sleep(0.5)
    s10, s1c, s00 = pr(0x10), pr(0x1C), pr(0x00)
    print('t%+.1fs 0x10=0x%08X idxA=%d | 0x1C=0x%08X | status0=0x%08X'
          % (k*0.5, s10, s10 & 0x1FF, s1c, s00), flush=True)
print('DONE')
