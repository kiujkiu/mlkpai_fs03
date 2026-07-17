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
ba = mmap.mmap(f, 0x438000, offset=0x10000000)
try:
    d = open('/home/uisrc/anime_slices.bin','rb').read(0x438000)
    ba[:len(d)] = d
    print('bank A <- default', len(d))
except Exception as e:
    ba[:] = b'\0'*0x438000; print('bank A zeroed:', e)
ba.close()
bb = mmap.mmap(f, 0x438000, offset=0x10500000); bb[:] = b'\0'*0x438000; bb.close()
pw(0x18, 0x10000000)
pw(0x0C, 0x000001FF); pw(0x0C, 0x9836C001); pw(0x0C, 0xC1000003)  # fast 25M 双沿/54行/oe192满亮(POV稀疏安全,全屏实心禁)
pw(0x10, (360 << 16) | 0x5)          # sensor+dual 光电角度 (2026-07-17 固化; fake调试: pov6_fake.py dual 0.5)
print('DISPLAY UP uptime', open('/proc/uptime').read().split()[0])
PY
/sbin/insmod /home/uisrc/povmem.ko 2>/dev/null; pkill pov_rxd 2>/dev/null
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
