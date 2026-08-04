#!/bin/bash
# ⚠ 必须 bash: 用了 /dev/tcp, dash 不支持
# board_ip.sh — 从 WSL 解析板子当前 IP (2026-08-04)
#
# 为什么需要它:
#   板子的地址一直是 DHCP 来的 (`pov_boot.sh` 直接跑 dhclient), 历史上在
#   10.10.20.234 / 10.10.20.239 / 10.10.21.3 / 10.10.21.250 之间跳过。
#   ⚠ 而"IP 变了"的现象与"WiFi 真掉线"**完全相同** (ping 不通/ssh 超时),
#     曾经把两个不相关的现象串成一条错误因果链 —— 排查前必须先解析地址。
#
# 为什么不硬设静态 IP:
#   DHCP 服务器就是网关 10.10.20.1, 历史租约横跨 .20/.21 两个半段,
#   **池子很可能覆盖任何我们想占的地址**。在办公网上占用池内地址迟早撞车,
#   而撞车现象同样伪装成"掉线"。⇒ 用 mDNS, 不跟 DHCP 抢。
#
# 为什么要绕 Windows:
#   板子跑 avahi (主机名 pov.local)。但 **WSL2 在 NAT 后面, mDNS 组播出不去**,
#   WSL 里装 libnss-mdns + avahi 也没用 (实测)。Windows 直连局域网, 能解析。
#   ⇒ 借 `cmd.exe ping -4` 拿 IPv4。⚠ 必须带 `-4`, 否则 Windows 会回 IPv6
#     链路本地地址 (fe80::...), WSL 用不了。
#
# 用法:
#   IP=$(tools/board_ip.sh) && ssh uisrc@$IP ...
#   tools/board_ip.sh --check      # 顺便验证 22 端口通不通

set -u
HOSTNAME_MDNS=${POV_MDNS:-pov.local}
MAC=${POV_MAC:-90-de-80-35-19-41}          # ARP 兜底用; Windows arp -a 是这个横杠格式
FALLBACKS=${POV_FALLBACK_IPS:-"10.10.20.239 10.10.21.250 10.10.21.3 10.10.20.234"}

alive() { timeout "${2:-2}" bash -c "cat </dev/null >/dev/tcp/$1/22" 2>/dev/null; }

# 收集候选: ① mDNS ② ARP 按 MAC ③ 已知地址。
# 🔴 每个候选都要**先验证 22 端口**再采信 —— 否则会照单全收陈旧缓存或错误条目
#   (实测: ARP 里 ff-ff-ff-ff-ff-ff 对应广播地址 10.10.21.255, 会被当成板子)。
cands=""
if command -v cmd.exe >/dev/null 2>&1; then
    cands="$cands $(cmd.exe /c "ping -4 -n 1 $HOSTNAME_MDNS" 2>/dev/null | tr -d '\r' \
           | grep -aoE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1)"
    cands="$cands $(cmd.exe /c "arp -a" 2>/dev/null | tr -d '\r' \
           | grep -ai "$MAC" | grep -aoE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1)"
fi
cands="$cands $FALLBACKS"

ip=""
for cand in $cands; do
    case "$cand" in ''|*.255|*.0) continue;; esac      # 跳空串与广播/网络地址
    if alive "$cand"; then ip=$cand; break; fi
done

if [ -z "$ip" ]; then
    echo "找不到板子: 候选 [$(echo $cands)] 都不通 22" >&2
    exit 1
fi
echo "$ip"

[ "${1:-}" = "--check" ] && echo "  22 通 (返回前已验证)" >&2
exit 0
