#!/bin/sh
# povwifi_watchdog.sh — mt7921u 掉线自愈 (2026-08-01)
#
# 背景: MT7921U dongle (0e8d:7961) 不稳, 掉线后 wpa_supplicant **不会自己重连**,
# 原来只能断电重启。实测特征:
#   · 信号很强 (-41 dBm) 却照样掉 → 不是覆盖问题
#   · 网卡统计 errors=0 dropped=0 → 不是链路误码
#   · **每次开机首次 init 必失败** ("Failed to get patch semaphore" ×8 →
#     "hardware init failed"), 靠 boot 脚本的 USB 复位脉冲重新枚举才成功
#   ⇒ 恢复手段对症: **重新枚举 USB** 比重连 wpa 更根治
#
# 🔴 保守设计 (最重要的一条): 推流满载时 ICMP 可能被降级/丢弃, 误判会把**好链路
# 拆掉**, 演示中出这事比掉线更糟。所以分两档:
#   · carrier 掉 或 没 IP  → 无歧义, 立即动手
#   · 有 carrier 有 IP 只是网关 ping 不通 → 要**连续 3 次**(45s) 才动手
#
# 装法见文件尾。停用: systemctl disable --now povwifi.timer

IF=wlx90de80351941
GW=10.10.20.1
USBDEV=1-1
LOG=/var/log/povwifi.log
STATE=/run/povwifi.fails
MAXFAIL=3

log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

have_carrier() { [ "$(cat /sys/class/net/$IF/carrier 2>/dev/null)" = "1" ]; }
have_ip()      { ip -4 addr show "$IF" 2>/dev/null | grep -q 'inet 10\.'; }
gw_ok()        { ping -c 3 -W 2 -I "$IF" "$GW" >/dev/null 2>&1; }

bring_up() {
    pkill wpa_supplicant 2>/dev/null; sleep 1
    /sbin/wpa_supplicant -B -i "$IF" -c /etc/wpa_supplicant/wpa.conf >/dev/null 2>&1
    sleep 5
    /sbin/dhclient -1 "$IF" >/dev/null 2>&1
    sleep 3
}

# ---------- 健康判定 ----------
if have_carrier && have_ip; then
    if gw_ok; then
        [ -f "$STATE" ] && rm -f "$STATE"      # 恢复正常, 清计数
        exit 0
    fi
    # 有 carrier 有 IP, 只是网关不通 → 可能是负载导致的 ICMP 丢失, 先累计
    n=$(cat "$STATE" 2>/dev/null || echo 0)
    n=$((n + 1))
    echo "$n" > "$STATE"
    if [ "$n" -lt "$MAXFAIL" ]; then
        log "gw unreachable ($n/$MAXFAIL) — 有 IP 有 carrier, 先观察不动手"
        exit 0
    fi
    log "gw unreachable ×$n — 判定掉线"
else
    log "DOWN: carrier=$(cat /sys/class/net/$IF/carrier 2>/dev/null) ip=$(have_ip && echo yes || echo no)"
fi
rm -f "$STATE"

# ---------- 🔴 动手前先取证 ----------
# 掉线后重启会清掉 dmesg, 之前每次都只能靠猜。恢复动作会改变现场,
# 所以**必须先存证再动手**。下一次掉线就有据可查了。
{
    echo "=========== $(date '+%F %T') 掉线现场 ==========="
    echo "--- dmesg 尾 30 行:"
    dmesg 2>/dev/null | tail -30
    echo "--- USB 设备还在不在 (不在=掉到 USB 层了):"
    ls -d /sys/bus/usb/devices/$USBDEV 2>/dev/null || echo "  $USBDEV 已消失!"
    cat /sys/bus/usb/devices/$USBDEV/power/runtime_status 2>/dev/null
    echo "--- 网卡与统计:"
    ip -s link show "$IF" 2>/dev/null | tail -5
    cat /proc/net/wireless 2>/dev/null | tail -2
    echo "--- 负载与内存:"
    uptime; free -m 2>/dev/null | head -2
    echo "--- 温度 (若有):"
    for z in /sys/class/thermal/thermal_zone*/temp; do [ -f "$z" ] && echo "  $z=$(cat $z)"; done
    echo "--- POV 引擎是否还活着 (区分'只是 WiFi 挂'与'整板挂'):"
    busybox devmem 0x40010000 2>/dev/null || echo "  寄存器读失败"
    echo "--- pov_rxd:"; pgrep -a pov_rxd | head -1
    echo "=============================================="
} >> "$LOG" 2>&1

# ---------- 第一级: 重连 ----------
log "L1: 重跑 wpa_supplicant + dhclient"
bring_up
if have_ip && gw_ok; then log "RECOVERED (L1 重连)"; exit 0; fi

# ---------- 第二级: 重新枚举 USB (复现开机时那次成功的 reset) ----------
log "L2: USB 重新枚举 $USBDEV"
echo "$USBDEV" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null
sleep 2
echo "$USBDEV" > /sys/bus/usb/drivers/usb/bind   2>/dev/null
sleep 8                                   # 等驱动 probe + 固件加载
bring_up
if have_ip && gw_ok; then log "RECOVERED (L2 USB 重新枚举)"; exit 0; fi

log "STILL DOWN — 需人工断电重启 (L1/L2 都没救回来)"
exit 1

# ================= 安装 =================
# cp povwifi_watchdog.sh /home/uisrc/ && chmod +x /home/uisrc/povwifi_watchdog.sh
#
# cat > /etc/systemd/system/povwifi.service <<'EOF'
# [Unit]
# Description=POV WiFi watchdog (mt7921u 掉线自愈)
# [Service]
# Type=oneshot
# ExecStart=/home/uisrc/povwifi_watchdog.sh
# EOF
#
# cat > /etc/systemd/system/povwifi.timer <<'EOF'
# [Unit]
# Description=Run POV WiFi watchdog every 15s
# [Timer]
# OnBootSec=60
# OnUnitActiveSec=15
# AccuracySec=1s
# [Install]
# WantedBy=timers.target
# EOF
#
# systemctl daemon-reload && systemctl enable --now povwifi.timer
