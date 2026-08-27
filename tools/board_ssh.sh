#!/bin/bash
# board_ssh.sh — 在 FS03 板子上跑命令 / 传文件 (封装 plink+pscp)
#
# 为什么要有它: WSL 没装 sshpass, 每次都得手拼 plink 的 -hostkey/-pw/-batch,
# 命令行长且容易漏 -batch (漏了会在没 TTY 的地方吊死)。
#
# 用法:
#   tools/board_ssh.sh "uptime; free -m"          在板上执行
#   tools/board_ssh.sh --sudo "busybox devmem 0x40010024 32"   先缓存 sudo 凭据再执行
#   tools/board_ssh.sh --put <本地> <板上>         上传 (pscp)
#   tools/board_ssh.sh --get <板上> <本地>         下载 (pscp)
#   tools/board_ssh.sh --ip                        只打印当前板子 IP
#
# IP: 默认 mDNS 解析 pov.local (DHCP 会变, 见 reference_fs03_board_access)。
#     可用 BOARD_IP=x.x.x.x 覆盖。
set -u

PLINK='/mnt/c/Program Files/PuTTY/plink.exe'
PSCP='/mnt/c/Program Files/PuTTY/pscp.exe'
# 每台板子一个 SSH 主机指纹。BOARD_HOSTKEY= 可覆盖; 否则按 BOARD_IP 选。
#   pov  10.10.20.239  第一台 (ICND2047 双面偏心)
#   pov2 10.10.21.226  第二台 (2026-08-26 装, ICND2049 单面)
HOSTKEY_pov='SHA256:u14U8c0RuKnVinQuaGH5ey6OKScaPOlRF3vMNqSnEGI'
HOSTKEY_pov2='SHA256:RfuPXPxnrnRkhscA6GQsFZBXzNOuUvn8qGvrEIGNPKo'
case "${BOARD_IP:-}" in
    10.10.21.226) HOSTKEY="${BOARD_HOSTKEY:-$HOSTKEY_pov2}" ;;
    *)            HOSTKEY="${BOARD_HOSTKEY:-$HOSTKEY_pov}"  ;;
esac
BUSER="${BOARD_USER:-uisrc}"
BPW="${BOARD_PW:-root}"

resolve_ip() {
    [ -n "${BOARD_IP:-}" ] && { echo "$BOARD_IP"; return 0; }
    # mDNS: WSL2 在 NAT 后组播出不去, 借 Windows 的 ping; -4 必须带, 否则回 IPv6 链路本地
    local ip
    ip=$(cmd.exe /c "ping -4 -n 1 pov.local" 2>/dev/null \
         | grep -ao '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
    [ -n "$ip" ] && { echo "$ip"; return 0; }
    echo "board_ssh: 解析不到 pov.local, 用 BOARD_IP= 指定" >&2
    return 1
}

IP=$(resolve_ip) || exit 1

case "${1:---help}" in
  --ip)   echo "$IP" ;;
  --put)  "$PSCP" -pw "$BPW" -hostkey "$HOSTKEY" "$2" "$BUSER@$IP:$3" ;;
  --get)  "$PSCP" -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP:$2" "$3" ;;
  --sudo) shift
          # sudo -S -v 单独缓存凭据; 不能对着 heredoc 用 -S (密码和脚本抢 stdin)
          "$PLINK" -ssh -batch -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP" \
            "echo $BPW | sudo -S -v 2>/dev/null; $*" ;;
  --help) sed -n '2,20p' "$0" ;;
  *)      "$PLINK" -ssh -batch -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP" "$*" ;;
esac
