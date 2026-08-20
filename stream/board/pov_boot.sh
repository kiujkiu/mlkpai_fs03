#!/bin/sh
# FS03 POV 开机 bring-up v5 fastboot — 显示最先 (panel+默认动画秒出), USB/WiFi 殿后
exec > /home/uisrc/pov_boot.log 2>&1
echo "=== pov_boot v5 uptime=$(cut -d' ' -f1 /proc/uptime)"
dmesg -n 1
# ①显示通路: panel 引擎 + sensor 模式 + bank A 默认动画 (最快出画面)
python3 - <<'PY'
import mmap, os
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000)
def pw(off, v): p[off:off+4] = v.to_bytes(4,'little')
# 帧区地址表 v3.1 (权威表见 stream/board/pov_rxd.c 文件头): bank 间距 16 MB,
# 每 bank 实际用 0x870000 (720 片双面帧的最大尺寸)。老的 5 MB 间距/0x438000
# 已作废——按老值清零会清到 bank A 自己身上, 而 bank B 留着上电垃圾。
BANK_A, BANK_B, BANK_BYTES = 0x10000000, 0x11000000, 0x870000

# ==== v3.4 3-bit 色深 (docs/design_icnd2047/05_3bit_bcm.md) ==================
# 🔴 **冷启动默认仍然是 1-bit** —— BPP3=0 时下面每一个写出去的值都与改这段之前
# 逐位相同 (自己对一遍: N_SLICES=360, STRIDE=0x3000, PHASE_B=180,
# 0x28=BANK_A+360*0x3000, CFG_MISC=0x98366F01)。要冷启动就进 3-bit, **只改
# BPP3 这一个数**, 其余四行会自己跟着走。
#
# 切 3-bit 需要同时对的四件事 (少一件就是花屏/错色, 不是"效果差一点"):
#   1) 默认内容换成 3-bit 打包的那份 (每片 3 个位平面, 片距 0x9000)
#   2) 引擎每圈片数 0x10[31:16] 换成 3-bit 的槽数 (方案定的是每面 60)
#   3) PHASE_B(0x1C) 必须 < 每圈片数 —— RTL 只做一次条件减不取模, 写 180 而
#      引擎只有 60 片 = 屏B 索引越界 = 读野地址。所以这里一律取"半圈"
#   4) oe_w0 = 0x0C sub10 的 oe_window: 1-bit 用 111 (免费亮度上限), 3-bit 用 184
#      (plane0 = MSB; 只有 plane2 受 111 限制, 见 05_3bit_bcm.md §5), 必须
#      降到 27, 否则 BCM 三个平面的权重变成 111:54:108, 1:2:4 的比例全乱
BPP3 = 0                                  # ← 唯一的开关: 1 = 冷启动进 3-bit
N_SLICES = 50 if BPP3 else 360            # 引擎每圈片数 (每面); 3-bit 实测每圈 53 个角度
STRIDE   = 0x9000 if BPP3 else 0x3000     # 片距 = 3 个位平面 / 1 个位平面
DEFAULT_BIN = ('/home/uisrc/anime_dual3b120.bin' if BPP3      # 60+60 片 3-bit
               else '/home/uisrc/anime_dual720.bin')          # 360+360 片 1-bit
OE_W0 = 184 if BPP3 else 111              # plane0=MSB(权重4) / 1-bit 亮度 (0x0C sub10)
OE_W1, OE_W2 = 54, 108                    # 中/高位平面 (0x0C sub01), 27:54:108 = 1:2:4
# ===========================================================================

ba = mmap.mmap(f, BANK_BYTES, offset=BANK_A)
n = 0
try:
    # v3.1 偏心双面默认帧 (720 片 = 面A 360 穿心 + 面B 360 偏移 14.29px)。
    # 旧的 anime_slices.bin 是 v3 对称几何的单面 360 片, 在偏心机器上
    # 「屏B@θ ≡ 屏A@(θ+180)」前提已作废 -> 两屏会显示两个不同的 3D 体。
    # BPP3=1 时换成 3-bit 那份 (120 片 * 0x9000 = 4.42 MB, 仍在一个 bank 里)。
    d = open(DEFAULT_BIN,'rb').read(BANK_BYTES)
    ba[:len(d)] = d; n = len(d)
    print('bank A <- default', DEFAULT_BIN, n)
except Exception as e:
    print('bank A: no default anime:', e)
ba[n:] = b'\0'*(BANK_BYTES - n)          # 尾部清零 (含 360 片以后的面B 区)
ba.close()
bb = mmap.mmap(f, BANK_BYTES, offset=BANK_B); bb[:] = b'\0'*BANK_BYTES; bb.close()
pw(0x1C, N_SLICES // 2)                  # PHASE_B = 半圈: 补偿渲染侧面B 的符号/手性
                                         #  (见 pov_rxd.c 的 PHASE_B_DUAL 注释)
                                         #  🔴 必须 < 每圈片数, 见上面第 3 条
pw(0x28, BANK_A + N_SLICES*STRIDE)       # 面B 基址 = 载荷后半段 (双面独立数据)
pw(0x18, BANK_A)
pw(0x10, (N_SLICES << 16) | 0x5)     # sensor 模式 + dual_en(bit2) <- 少了它屏B 不亮
# oe_window 192(箝位187) -> 111: LWAIT = max(0, oe-111), 所以 111 是
# **免费亮度上限** —— 行周期 271->195 拍, 顺带 LED 电流约 -29% 给 5V 让余量。
#
# bit29 (ddr_slow) 0xB8->0x98 = 翻回 fast, 2026-08-20 上板实测:
#   OE 沿数 258649 -> 536277 (2.07x) / 行周期 15.52 -> 7.81 us
#   整屏 838 -> 421 us / 2D 刷新 1193 -> 2373 Hz / 占空 0.574 -> 0.569 (守恒)
# 🔴 回滚: 把 CFG_MISC 改回 0xB8366F01 (SI 逃生门, 屏花/丢行时用)
# oe_window 是 CFG_MISC 的 [15:8]: 1-bit 111 / 3-bit 27 (= BCM 的 oe_w0)。
# BPP3=0 时下面这行算出来正好还是 0x98366F01, 一位都不差。
CFG_MISC = (0x98366F01 & ~0xFF00) | ((OE_W0 & 0xFF) << 8)
pw(0x0C, 0x000001FF); pw(0x0C, CFG_MISC); pw(0x0C, 0xC1000003)
# v3.4 0x0C **subcmd=01**: [7:0]=oe_w1 [15:8]=oe_w2 [16]=bpp_mode。
# 这是 3-bit 的固化点 —— 不写的话重启就回 1-bit (pov_rxd 会在收到第一帧时按帧
# 重写它, 但那要等到有人推流, 冷启动的默认内容等不到)。
# ⚠ 老比特流里 subcmd=01 是个未实现的空槽, 写它是无害的空操作, 所以这一行
#   在 RTL 落地之前也可以照写。
pw(0x0C, (1 << 30) | (OE_W1 & 0xFF) | ((OE_W2 & 0xFF) << 8) | ((1 if BPP3 else 0) << 16))
print('bpp_mode', 1 if BPP3 else 0, 'oe_w', OE_W0, OE_W1, OE_W2, 'n_slices', N_SLICES)
print('DISPLAY UP uptime', open('/proc/uptime').read().split()[0])
PY
# WC 映射窗 = bank A 起 .. bank B 末 (0x1870000) + 余量; 显式传参, 不靠模块默认
/sbin/insmod /home/uisrc/povmem.ko base=0x10000000 size=0x2900000 2>/dev/null
# pov_rxd 由 povrxd.service 托管 (Restart=always + 开机自启), 本脚本不再自己起 ——
# 两边都起会抢 9500 端口, 表现为 service 无限重启 (NRestarts 狂涨) + bind 失败。
# ②USB PHY 复位 (短脉冲) + WiFi
modprobe mt7921u 2>&1
python3 - <<'PY'
import mmap, os, time
f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
slcr = mmap.mmap(f, 4096, offset=0xF8000000)
def srd(o): return int.from_bytes(slcr[o:o+4],'little')
def swr(o, v): slcr[o:o+4] = v.to_bytes(4,'little')
swr(0x008, 0xDF0D)
swr(0x71C, 0x600)
swr(0x12C, srd(0x12C) | (1 << 22))
g = mmap.mmap(f, 4096, offset=0xE000A000)
def rd(o): return int.from_bytes(g[o:o+4],'little')
def wr(o, v): g[o:o+4] = v.to_bytes(4,'little')
wr(0x204, rd(0x204) | 0x80); wr(0x208, rd(0x208) | 0x80)
wr(0x40, rd(0x40) & ~0x80); time.sleep(0.3)
wr(0x40, rd(0x40) | 0x80);  time.sleep(0.8)
print('phy pulse done')
PY
echo ci_hdrc.0 > /sys/bus/platform/drivers/ci_hdrc/unbind 2>/dev/null; sleep 1
echo ci_hdrc.0 > /sys/bus/platform/drivers/ci_hdrc/bind 2>/dev/null
IF=''
for t in $(seq 1 30); do
    IF=$(ls /sys/class/net 2>/dev/null | grep '^wl' | head -1)
    [ -n "$IF" ] && break
    sleep 1
done
sleep 2; IF2=$(ls /sys/class/net 2>/dev/null | grep '^wl' | head -1); [ -n "$IF2" ] && IF=$IF2
echo "IF=[$IF]"
[ -n "$IF" ] || { echo NO_WLAN_STATIC_ONLY; exit 0; }
pkill wpa_supplicant 2>/dev/null; sleep 1
wpa_supplicant -B -i $IF -c /etc/wpa_supplicant/wpa.conf
for t in $(seq 1 20); do
    iw dev $IF link 2>/dev/null | grep -q Connected && break
    sleep 1
done
iw dev $IF link | head -2
dhclient $IF
ip -br addr show $IF
echo BRINGUP_DONE
