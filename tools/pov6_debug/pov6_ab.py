#!/usr/bin/env python3
# A/B 引擎 ↔ 物理口 (P1/P3) 判别 (板上 root)
#
# 背景: README 铁律①记「屏在 P3 = B 引擎」但只是当时的推测
# (pov6_colors.py:27 原注释是「屏*可能*在 P3」)。接第二块屏前必须钉死归属,
# 否则 PHASE_B / 灌图目标 / 故障定位全是糊的。
#
# 手法: 用**内容**区分而非 enable 区分 —— eng_b_en = dual_en | fb_sel_b,
# A 引擎没有 disable 位, 寄存器静默不了, 只能一边灌色一边灌黑。
#
# usage: pov6_ab.py [hold_s]     默认每相 4s, Ctrl-C 停
#
# ⚠ 全程单色 (3 lane) + oe=8 沿 — 共阳屏全屏实心的电流红线, 勿加亮度。
import mmap, os, sys, time

hold = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x10000, offset=0x40010000)
def pw(off, v): p[off:off+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')
def pr(off):    return int.from_bytes(p[off:off+4], 'little')

CTRL_BASE = (360 << 16) | 0x4          # dual_en 常开 (两个引擎都要跑)
CFG       = 0x98360801                 # 54 行 / oe 8 沿 / fast 50M

def fb_fill(lanes_on, sel_b):
    # fb 窗: awaddr[15]=1, [14:11]=lane, [10:2]={row,pair}; fb_sel_b 选 A/B
    pw(0x10, CTRL_BASE | (0x8 if sel_b else 0))
    for lane in range(9):
        word = 0xFFFFFFFF if lane in lanes_on else 0
        base = 0x8000 | (lane << 11)
        for i in range(512):
            pw(base | (i << 2), word)

RED = {0, 3, 6}
NONE = set()

pw(0x10, CTRL_BASE)
pw(0x0C, 0x000001FF)      # sdi_mask 全开
pw(0x0C, CFG)

print('oe=8 沿单色红 — 看哪个物理口亮, 记下来', flush=True)
try:
    while True:
        for tag, la, lb in (('A 亮 / B 黑', RED, NONE),
                            ('A 黑 / B 亮', NONE, RED)):
            fb_fill(la, 0)
            fb_fill(lb, 1)
            pw(0x10, CTRL_BASE)               # fb_sel 回 A
            pw(0x0C, 0xC1000003)              # auto_en + use_fb
            print('%-12s status0=0x%08X  (bit0=engA_busy bit12=engB_busy)'
                  % (tag, pr(0x00)), flush=True)
            time.sleep(hold)
except KeyboardInterrupt:
    pass
finally:
    pw(0x0C, 0xC1000000)  # auto 停
    print('AB_DONE status0=0x%08X' % pr(0x00), flush=True)
