#!/bin/sh
# FS03 POV 开机 bring-up v5 fastboot — 显示最先 (panel+默认动画秒出), USB/WiFi 殿后
exec > /home/uisrc/pov_boot.log 2>&1
echo "=== pov_boot v6 uptime=$(cut -d' ' -f1 /proc/uptime)"
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
ba = mmap.mmap(f, BANK_BYTES, offset=BANK_A)
n = 0
try:
    d = open('/home/uisrc/anime_slices.bin','rb').read(BANK_BYTES)
    ba[:len(d)] = d; n = len(d)
    print('bank A <- default', n)
except Exception as e:
    print('bank A: no default anime:', e)
ba[n:] = b'\0'*(BANK_BYTES - n)          # 尾部清零 (含 360 片以后的面B 区)
ba.close()
bb = mmap.mmap(f, BANK_BYTES, offset=BANK_B); bb[:] = b'\0'*BANK_BYTES; bb.close()
pw(0x28, 0)                              # 面B 基址清 0 = PL 两面都用 0x18 (单面)
pw(0x18, BANK_A)
pw(0x0C, 0x000001FF); pw(0x0C, 0x9836C001); pw(0x0C, 0xC1000003)  # fast 25M 双沿/54行/oe192满亮(POV稀疏安全,全屏实心禁)
pw(0x10, (360 << 16) | 0x5)          # sensor+dual 光电角度 (2026-07-17 固化; fake调试: pov6_fake.py dual 0.5)
print('DISPLAY UP uptime', open('/proc/uptime').read().split()[0])
PY
# WC 映射窗 = bank A 起 .. bank B 末 (0x1870000) + 余量; 显式传参, 不靠模块默认
/sbin/insmod /home/uisrc/povmem.ko base=0x10000000 size=0x2900000 2>/dev/null; pkill pov_rxd 2>/dev/null
/home/uisrc/pov_rxd > /home/uisrc/pov_rxd.log 2>&1 &
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
sleep 2; IF2=$(ls /sys/class/net 2>/dev/null | grep ^wl | head -1); [ -n "$IF2" ] && IF=$IF2; echo "IF=[$IF]"
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
