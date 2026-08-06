#!/bin/sh
# wifi_band.sh — 板端 WiFi 频段切换 (2026-08-06), **带自动回退**
#
# 为什么: 板子关联在 2437 MHz (ch6), 而 ch6 上同时能扫到 9 个以上 BSS,
# 其中 6 个强于 -52 dBm —— 是一条被占满的信道。同 SSID 的 5 GHz BSS
# (5745 MHz, ch149) 信号 -29 dBm, 比 2.4 GHz 的 -41 还强。
#
# 为什么带自动回退: 用 bssid 把 wpa_supplicant 钉死在一个 BSS 上, 万一关联
# 不上它**只会一直重试这一个**, 板子就彻底失联了 (没有串口兜底)。所以先
# 用 systemd-run --on-active 排一个"到点无条件解钉"的定时器, 切换成功后
# 再把定时器撤掉。
#
# 用法 (板上, root):
#   wifi_band.sh 5      切到 5 GHz (fa:38:8d:9f:53:dd @5745)
#   wifi_band.sh 24     解钉, 回默认选择 (通常落回 2.4 GHz)
#   wifi_band.sh show   只看当前状态
set -u
IF=$(ls /sys/class/net | grep '^wl' | head -1)
BSSID_5G=${BSSID_5G:-fa:38:8d:9f:53:dd}   # undef @5745 ch149 80MHz
BSSID_24=${BSSID_24:-fa:38:8d:1f:53:dd}   # undef @2437 ch6  20MHz
REVERT_S=${REVERT_S:-150}
wc() { wpa_cli -i "$IF" "$@"; }

show() {
    echo "IF=$IF"
    wc status | grep -E '^(bssid|freq|wpa_state|ip_address)='
    wc signal_poll | grep -E 'RSSI|LINKSPEED|FREQUENCY|WIDTH'
}

arm_revert() {
    systemctl stop wifirevert.timer wifirevert.service 2>/dev/null
    systemctl reset-failed wifirevert.timer wifirevert.service 2>/dev/null
    systemd-run --unit=wifirevert --on-active="$REVERT_S" \
        /bin/sh -c "wpa_cli -i $IF set_network 0 bssid any; wpa_cli -i $IF reassociate" \
        >/dev/null 2>&1
}
disarm_revert() {
    systemctl stop wifirevert.timer wifirevert.service 2>/dev/null
    systemctl reset-failed wifirevert.timer wifirevert.service 2>/dev/null
}

case "${1:-show}" in
5)
    arm_revert
    wc set_network 0 bssid "$BSSID_5G" >/dev/null
    wc reassociate >/dev/null
    for i in $(seq 1 25); do
        f=$(wc status | sed -n 's/^freq=//p')
        st=$(wc status | sed -n 's/^wpa_state=//p')
        [ "$st" = COMPLETED ] && [ "$f" = 5745 ] && break
        sleep 1
    done
    show
    f=$(wc status | sed -n 's/^freq=//p')
    if [ "$f" = 5745 ]; then disarm_revert; echo "OK 5GHz (回退定时器已撤)";
    else echo "FAIL: 还在 $f, ${REVERT_S}s 后自动解钉"; fi
    ;;
24)
    # 🔴 必须**钉 2.4 GHz 的 BSSID**, 不能只是 `bssid any`。踩过: 解钉之后
    #   wpa_supplicant 按信号强弱自己选, 而 5 GHz 那个 BSS (-29 dBm) 比
    #   2.4 GHz 的 (-41 dBm) 还强 —— 于是"切回 2.4 GHz"实际上原地不动,
    #   一整轮 A/B 三组配置全跑在 5 GHz 上, 却按 2.4/5 GHz 汇报。
    #   判据只看 freq=, 不看命令有没有返回 OK。
    arm_revert
    wc set_network 0 bssid "$BSSID_24" >/dev/null
    wc reassociate >/dev/null
    for i in $(seq 1 25); do
        f=$(wc status | sed -n 's/^freq=//p')
        st=$(wc status | sed -n 's/^wpa_state=//p')
        [ "$st" = COMPLETED ] && [ "$f" = 2437 ] && break
        sleep 1
    done
    show
    f=$(wc status | sed -n 's/^freq=//p')
    if [ "$f" = 2437 ]; then disarm_revert; echo "OK 2.4GHz";
    else echo "FAIL: 还在 $f"; fi
    ;;
any)
    wc set_network 0 bssid any >/dev/null
    wc reassociate >/dev/null
    sleep 6
    disarm_revert
    show
    ;;
*) show ;;
esac
