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
# 0x28=BANK_A+360*0x3000, CFG_MISC=0x98366F01)。唯一变了的是 0x0C sub01:
# 0x40006C36 → 0x40002E5C (oe_w1/oe_w2 从 54/108 修成 92/46) —— 但 bpp_mode=0
# 时这两个权重根本不参与 1-bit 通路, 且 pov_rxd 的 bcm_apply 本来就用 92/46,
# 所以 1-bit 显示逐位不变。要冷启动就进 3-bit, **只改 BPP3 这一个数**,
# 下面那四行 (N_SLICES/STRIDE/DEFAULT_BIN/OE_W0) 会自己跟着走。
#
# 🔴🔴 下面这五样是**一个整体, 只能一起改**, 不存在"先改一个看看" ————————
#      ① plane 位序 (host 侧 pack_obs.pack_slice bpp=3 的 plane0 装哪一位)
#      ② oe_w0 / oe_w1 / oe_w2 三个权重
#      ③ N_SLICES (引擎每圈槽数) —— ⚠ **这个数只有本脚本写**: pov_rxd 从头到尾
#         只 reg_rd(0x24) 读回它, 从不按帧改 0x10[31:16]。所以 BPP3=0 时哪怕
#         推一份 3-bit 的流进来, 引擎照样按 360 槽 × 0x9000 去扫一份只有 100
#         片的载荷 = 260 槽读野地址。**推 3-bit 之前必须先让本脚本进 3-bit**。
#      ④ 默认内容 .bin 的面拆分排布 (N_SLICES 片 + N_SLICES 片)
#      ⑤ PHASE_B (= N_SLICES//2)
#      本仓库已经为"只改了一半"付过两次学费:
#        · CFG_MISC bit29 的语义反转了, 而引导脚本没跟着改 → 屏**静默**降速跑了
#          一个月, 日志里一个字都没有;
#        · 2026-08-20 把位序从 LSB-first 改成 MSB-first, 却漏改了这里的
#          oe_w1/oe_w2 → 权重成了 184/54/108, 配 MSB-first 位序算出来是
#            码值1(bit0→plane2)=108 沿, 码值2(bit1→plane1)=54 沿
#          **码值 1 比码值 2 还亮 = 灰阶非单调**, 上板肉眼直接看得见。
#      判据: 权重表必须满足 plane0:plane1:plane2 = 4:2:1 **且** plane0 装 MSB。
#      改任何一个之前, 先跑 `python3 tools/test_idle_anim_3bit.py` (离线自检,
#      会把这五样重新算一遍并与本文件里的值对账)。
#
# 切 3-bit 需要同时对的四件事 (少一件就是花屏/错色, 不是"效果差一点"):
#   1) 默认内容换成 3-bit 打包的那份 (每片 3 个位平面, 片距 0x9000,
#      面拆分 = N_SLICES + N_SLICES; 由 tools/gen_default_3bit.py 生成)
#   2) 引擎每圈片数 0x10[31:16] 换成 3-bit 的槽数: 3-bit 一片要画三个位平面,
#      frame_period 从 10530 涨到 31590 拍 ⇒ 同样转速每圈只画得出 **53** 个
#      角度, 所以取 50 (留余量, 且是偶数 → PHASE_B 半圈整除)
#   3) PHASE_B(0x1C) 必须 < 每圈片数 —— RTL 只做一次条件减不取模, 写 180 而
#      引擎只有 50 片 = 屏B 索引越界 = 读野地址。所以这里一律取"半圈"
#      (pov_rxd.c 的 flip 线程同样已经从写死 180 改成 n_eng/2, 两边同一条规则)
#   4) oe_w0 = 0x0C sub10 的 oe_window: 1-bit 用 111 (免费亮度上限), 3-bit 用
#      **184**。为什么 3-bit 反而能超过 111: 硬件 LWAIT = max(0, oe-111) 里的
#      111 来自"OE 结束后还要等行驱推进 80 拍", 而 **plane 边界不推进行驱**
#      ⇒ 三个 plane 里只有最后一个 (plane2) 受 111 限制, plane0/plane1 的上限
#      是移位窗的 192。所以把**最大权重放在不受限的 plane0** 上 = plane0 装
#      MSB, 权重 184/92/46 (精确 4:2:1)。实测占空比 0.550 vs 1-bit 的 0.569
#      = 96.7%, 亮度几乎无损, 且 frame_period 一拍不涨。
#      (反过来 LSB-first 的话 4W 落在受限的 plane2 上, W<=27, 占空比只剩 0.32)
BPP3 = 0                                  # ← 唯一的开关: 1 = 冷启动进 3-bit
N_SLICES = 50 if BPP3 else 360            # 引擎每圈片数 (每面); 见上面第 2 条
STRIDE   = 0x9000 if BPP3 else 0x3000     # 片距 = 3 个位平面 / 1 个位平面
DEFAULT_BIN = ('/home/uisrc/anime_dual3b100.bin' if BPP3      # 50+50 片 3-bit
               else '/home/uisrc/anime_dual720.bin')          # 360+360 片 1-bit
# 🔴 三个权重与 plane 位序是一对, 见上面那段"五样一起改"。plane0=MSB(权重4)。
OE_W0 = 184 if BPP3 else 111              # plane0 = MSB, 走 0x0C **sub10** 的 oe_window
OE_W1, OE_W2 = 92, 46                     # plane1(权重2) / plane2 = LSB(权重1), 走 sub01
assert not BPP3 or (OE_W0, OE_W1, OE_W2) == (4*46, 2*46, 46), '权重必须是 4:2:1'
assert not BPP3 or N_SLICES % 2 == 0, 'PHASE_B = N_SLICES//2 要整除'
assert 2*N_SLICES*STRIDE <= BANK_BYTES, '双面帧装不进一个 bank'
# ===========================================================================

ba = mmap.mmap(f, BANK_BYTES, offset=BANK_A)
n = 0
try:
    # v3.1 偏心双面默认帧 (720 片 = 面A 360 穿心 + 面B 360 偏移 14.29px)。
    # 旧的 anime_slices.bin 是 v3 对称几何的单面 360 片, 在偏心机器上
    # 「屏B@θ ≡ 屏A@(θ+180)」前提已作废 -> 两屏会显示两个不同的 3D 体。
    # BPP3=1 时换成 3-bit 那份: 100 片 (面A 50 + 面B 50) * 0x9000 = 3,686,400 B
    # = 3.52 MB = bank 的 42%, 三缓冲/povmem 映射窗一个字节都不用改。
    # 🔴 这份 .bin 是**解压后的载荷原样**, 不是 PVSA 容器 —— 本脚本直接 memcpy
    #   进 bank, 不解压不走协议。生成: tools/gen_default_3bit.py --slices 50
    #   (面拆分点 50*0x9000=0x1C2000 必须与下面 pw(0x28,...) 算出来的一致)
    d = open(DEFAULT_BIN,'rb').read(BANK_BYTES)
    ba[:len(d)] = d; n = len(d)
    print('bank A <- default', DEFAULT_BIN, n)
except Exception as e:
    print('bank A: no default anime:', e)
ba[n:] = b'\0'*(BANK_BYTES - n)          # 尾部清零 (含载荷末尾以后的整段)
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
# oe_window 是 CFG_MISC 的 [15:8] = **BCM 的 oe_w0**: 1-bit 111 / 3-bit 184。
# 🔴 184 > 111 不是笔误: plane 边界不推进行驱 ⇒ 只有 plane2 吃 LWAIT 的 111
#    上限, plane0 的上限是移位窗 192。见上面第 4 条。
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
# WC 映射窗: pov_rxd v3.2 是**三缓冲** (bank A/B/C @ 16MB 一格), 所需最小值
# FRAME_MAP_LEN = 2*0x1000000 + 0x870000 = 0x2870000 (40.4 MiB)。这里给
# 0x2900000 = 41.0 MiB 留一点余量。⚠ 给小了 mmap 直接 -EINVAL, pov_rxd 静默
# 回落到 /dev/mem 的 Strongly-Ordered 映射 (8.85MB 要 74-148ms) = 帧率崩。
# 3-bit 100 片 (3.52MB/帧) 比 1-bit 720 片还小, 这个窗**不用跟着改**。
# 显式传参, 不靠模块默认 (.ko 默认只有 0x1900000, 对三缓冲不够)
/sbin/insmod /home/uisrc/povmem.ko base=0x10000000 size=0x2A00000 2>/dev/null
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
