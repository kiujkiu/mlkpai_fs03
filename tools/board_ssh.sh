#!/bin/bash
# board_ssh.sh — 在 FS03 板子上跑命令 / 传文件 (封装 plink+pscp)
#
# 为什么要有它: WSL 没装 sshpass, 每次都得手拼 plink 的 -hostkey/-pw/-batch,
# 命令行长且容易漏 -batch (漏了会在没 TTY 的地方吊死)。
#
# 用法:
#   BOARD=pov2 tools/board_ssh.sh "uptime"        指定板子 (默认 pov)
#   tools/board_ssh.sh --sudo "busybox devmem 0x40010024 32"   先缓存 sudo 凭据
#   tools/board_ssh.sh --put <本地> <板上>         上传 (pscp)
#   tools/board_ssh.sh --get <板上> <本地>         下载 (pscp)
#   tools/board_ssh.sh --ip                        只打印解析到的 IP
#   tools/board_ssh.sh --who                       连上去问 hostname, 核对是不是想要的那台
#
# 🔴 2026-08-28 改为**按板名**而不是按 IP:
#   当天两块板的 DHCP 地址换了三轮 (pov2: .226 -> .241 -> 20.239; pov1 接手了 .241),
#   旧版按 BOARD_IP 猜指纹的 case 分支当场全错, 表现是"连不上"。
#   (plink 的 -hostkey 本身能防误连 —— 指纹不符会直接拒, 所以不会连错板还不自知。)
#   ⇒ 现在: 板名 -> mDNS <名>.local -> 该板自己的指纹。IP 再怎么换都不用改脚本。
#   应急仍可 BOARD_IP= / BOARD_HOSTKEY= 覆盖。
set -u

PLINK='/mnt/c/Program Files/PuTTY/plink.exe'
PSCP='/mnt/c/Program Files/PuTTY/pscp.exe'

BOARD="${BOARD:-pov}"                       # pov = 第一台 / pov2 = 第二台
HOSTKEY_pov='SHA256:u14U8c0RuKnVinQuaGH5ey6OKScaPOlRF3vMNqSnEGI'
HOSTKEY_pov2='SHA256:RfuPXPxnrnRkhscA6GQsFZBXzNOuUvn8qGvrEIGNPKo'
case "$BOARD" in
    pov)  DEFKEY="$HOSTKEY_pov"  ;;
    pov2) DEFKEY="$HOSTKEY_pov2" ;;
    *)    echo "board_ssh: 未知板名 '$BOARD' (只认 pov / pov2)" >&2; exit 1 ;;
esac
HOSTKEY="${BOARD_HOSTKEY:-$DEFKEY}"
BUSER="${BOARD_USER:-uisrc}"
BPW="${BOARD_PW:-root}"

resolve_ip() {
    [ -n "${BOARD_IP:-}" ] && { echo "$BOARD_IP"; return 0; }
    # mDNS: WSL2 在 NAT 后组播出不去, 借 Windows 的 ping; -4 必须带, 否则回 IPv6 链路本地
    local ip
    ip=$(cmd.exe /c "ping -4 -n 1 $BOARD.local" 2>/dev/null \
         | grep -ao '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
    [ -n "$ip" ] && { echo "$ip"; return 0; }
    echo "board_ssh: 解析不到 $BOARD.local, 用 BOARD_IP= 指定" >&2
    return 1
}

IP=$(resolve_ip) || exit 1

case "${1:---help}" in
  --ip)   echo "$IP" ;;
  --who)  # 连上去问 hostname, 核对是不是想要的那台 (今天 IP 换了三轮, 值得随手核)
          got=$("$PLINK" -ssh -batch -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP" hostname 2>/dev/null | tr -d '\r\n')
          if [ "$got" = "$BOARD" ]; then
              echo "OK  BOARD=$BOARD  ip=$IP  hostname=$got"
          else
              echo "⚠ 不符  BOARD=$BOARD  ip=$IP  hostname=${got:-<连不上>}" >&2; exit 1
          fi ;;
  --put)  "$PSCP" -pw "$BPW" -hostkey "$HOSTKEY" "$2" "$BUSER@$IP:$3" ;;
  --get)  "$PSCP" -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP:$2" "$3" ;;
  --sudo) shift
          # sudo -S -v 单独缓存凭据; 不能对着 heredoc 用 -S (密码和脚本抢 stdin)
          "$PLINK" -ssh -batch -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP" \
            "echo $BPW | sudo -S -v 2>/dev/null; $*" ;;
  --help) sed -n '2,25p' "$0" ;;
  *)      "$PLINK" -ssh -batch -pw "$BPW" -hostkey "$HOSTKEY" "$BUSER@$IP" "$*" ;;
esac
