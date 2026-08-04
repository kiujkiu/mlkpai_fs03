#!/bin/bash
# flip_bench.sh — 端到端帧率 / 丢帧测量台 (2026-08-04)
#
# 一次测量 = 「板端重启 pov_rxd(带参数) + 设定引擎转速 + PC 全速推流 N 秒 +
#             收板端 DIAG 行」, 输出一行可对比的结果。
#
# 🔴 前置事实 (别再踩): POV 引擎默认是 **sensor 模式**, slice_idx 由电机的
#    index 脉冲推进。**电机不转时 slice_idx 恒为 0**, flip 线程永远等不到
#    「离开翻页窗再进入」, 每帧顶满 FLIP_TIMEOUT_MS=2000ms 才 FORCED 翻一次
#    -> flip≈0.95/s, 其余全进 drop -> 看起来「丢帧 83%」。这与网络无关。
#    所以本脚本每次测量前都用 diag_fakespin.py 把引擎推起来并**回读确认**。
#
# 用法:
#   tools/flip_bench.sh 单次 [rps] [pov_rxd 额外参数...]
#   tools/flip_bench.sh 矩阵                 # 跑完整对比矩阵
# 环境变量: BOARD (默认走 tools/board_ip.sh), PVS (anim.pvs 路径), SECS (默认 20)

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
BOARD=${BOARD:-$("$HERE/board_ip.sh")} || exit 1
PVS=${PVS:-/mnt/d/claude_workspace/pov3d/_flipdiag/anim.pvs}
SECS=${SECS:-20}
PLINK=${PLINK:-/mnt/c/Program Files/PuTTY/plink.exe}
HK=${HK:-SHA256:u14U8c0RuKnVinQuaGH5ey6OKScaPOlRF3vMNqSnEGI}
RXD=${RXD:-/tmp/pov_rxd_diag}          # 带 DIAG 的诊断构建
UNIT=povdiag

sh_board() { timeout 90 "$PLINK" -ssh -batch -hostkey "$HK" -pw root "uisrc@$BOARD" "$1"; }
sudo_board() { sh_board "echo root | sudo -S sh -c '$1'" 2>/dev/null; }

# 一次测量。$1=rps, $2=loadgen 额外参数, $3.. = pov_rxd 参数
run_one() {
    local rps=$1 genargs=$2; shift 2
    local rxdargs="$*"
    sudo_board "systemctl stop $UNIT 2>/dev/null; sleep 1; systemd-run --unit=$UNIT $RXD $rxdargs" >/dev/null
    sleep 2
    local spin
    spin=$(sudo_board "python3 /tmp/diag_fakespin.py $rps" | grep -a '实测' | sed 's/.*: //')
    echo "--- rxd[$rxdargs] gen[$genargs] rps=$rps  引擎: $spin"
    python3 "$HERE/pvs_loadgen.py" --host "$BOARD" --file "$PVS" --seconds "$SECS" \
            --quiet --tag "" $genargs
    # 取推流稳定后的 DIAG 行 (掐头去尾各 2 行), 求平均
    sudo_board "journalctl -u $UNIT --no-pager | grep -a DIAG | tail -n $((SECS - 3))" \
      | sed 's/.*DIAG //' | awk '
        { for (i = 1; i <= NF; i++) {
            if ($i ~ /^eng=/) { sub("eng=", "", $i); sub("rev/s", "", $i); en += $i }
            if ($i ~ /^rx=/)  { sub("rx=", "", $i); sub("/s", "", $i); rx += $i }
            if ($i ~ /^flip=/) { sub("flip=", "", $i); sub("/s", "", $i); fl += $i }
            if ($i ~ /^drop=/) { sub("drop=", "", $i); dr = $i }
            if ($i == "dec")  { split($(i+1), a, "/"); dec += a[1]; decmax += a[2] + 0 }
            if ($i == "(A")   { fa += $(i+1); fb += $(i+3) }
            if ($i == "cpy")  { split($(i+1), b, "/"); cpy += b[1]; cpymax += b[2] + 0 }
            if ($i == "wait") { split($(i+1), c, "/"); wt += c[1]; wtmax += c[2] + 0 } }
          n++ }
        END { if (!n) { print "  (无 DIAG 行)"; exit }
              printf "  板端均值(%d 秒窗): eng %.1frev/s  rx %.2f/s  flip %.2f/s  drop累计 %s | dec %.1fms (A %.1f B %.1f, 峰 %.1f) | cpy %.1fms (峰 %.1f) | wait %.1fms (峰 %.1f)\n",
                     n, en/n, rx/n, fl/n, dr, dec/n, fa/n, fb/n, decmax/n, cpy/n, cpymax/n, wt/n, wtmax/n }'
}

case "${1:-矩阵}" in
单次) rps=${2:-16.1}; shift 2 2>/dev/null || shift $#; run_one "$rps" "--window 2 --fps 0" "$@" ;;
矩阵)
    echo "=== flip_bench 矩阵  board=$BOARD  每档 ${SECS}s ==="
    run_one 16.1 "--window 2 --fps 0"                              # 基准
    run_one 16.1 "--window 1 --fps 0"                              # stop-and-wait
    run_one 16.1 "--window 3 --fps 0"
    run_one 16.1 "--window 4 --fps 0"
    run_one 16.1 "--window 2 --fps 0" --decode serial              # 单核解码
    run_one 16.1 "--window 2 --fps 0" --flip-window dual           # 半圈双窗
    run_one 8    "--window 2 --fps 0"                              # 转速一半
    run_one 4    "--window 2 --fps 0"                              # 转速 1/4 -> flip 成瓶颈
    run_one 16.1 "--window 2 --fps 4"                              # 发送端限流
    ;;
*) echo "用法: $0 [单次 rps [rxd 参数...] | 矩阵]"; exit 2 ;;
esac
