#!/bin/sh
# oe_sweep.sh — 扫 oe_window, 测 OE 低宽/行周期的**斜率**。
#
# 为什么这么测:
#   ddr_slow 的作用是把 OE 窗从 oe 拍变成 2*oe 拍 (icnd2047_panel_core.v:97
#   win_aclk = ddr_slow ? oe*2 : oe)。
#   ⇒ d(OE低宽)/d(oe) = 2  表示 slow(25Mbps) 生效; = 1 表示 fast。
#   **这样就不用翻 bit29 也能测出 bit29 的效果** —— 改 oe 只动亮度, 不动链路速率,
#   风险远低于把 50Mbps 打到屏上。
#   截距还能顺带解出模型里那个未建模的固定开销 (实测行周期比模型长 1.79x)。
#
# 🔴 会写 0x0C (只改 [15:8]=oe, 其余位原样保留), 结束时无条件恢复。
#    oe 太低会暗到看不见(见 feedback_oe_window_too_low_invisible), 但这是
#    运行时寄存器, 恢复一条写就回来, 且重启即回默认。
#
# 用法 (板上): sudo sh oe_sweep.sh
# 🔴 dash 陷阱: $(( ... oe << 8 ... )) 里的 `<<` 会被当成 here-document ⇒ 语法错。
#    改用 oe*256, 地址也写死, 全程不做十六进制算术。
set -u
W0C=0x4001000C          # AXI-Lite 写口 0x0C
ORIG=0xB8366F01         # pov_boot.sh:38 的原值 (oe=0x6F=111)
BASEW=3090350081        # 0xB8360001 十进制 ([15:8]=0, 加 oe*256 即可)

restore() {
    busybox devmem $W0C 32 $ORIG
    echo "-- restored 0x0C = $ORIG (oe=111)"
}
trap restore EXIT INT TERM

printf '%-6s %-12s %-12s %-8s %s\n' oe "OE_low_clk" "row_clk" "duty" "2D_Hz"
# 可测区间由自检门槛(每段>=10样本, dt~0.47us)反推: 低段 ~3*oe 拍 ⇒ oe>=78;
# 高段 = 行(~730拍) - 低段 ⇒ oe<=164。区间外必然 INCONCLUSIVE, 不必浪费。
for oe in 80 95 110 125 140 155; do
    W=$(printf '0x%08X' $(( BASEW + oe * 256 )))
    busybox devmem $W0C 32 $W
    sleep 1                     # 让当前帧走完, 别测到过渡态
    /tmp/oeprobe 2.0 2>/dev/null | awk -v oe=$oe '
        /=> row/ { row=$3; low=$6 }
        /OE-low duty/ { duty=$NF }
        END {
            if (row == "") { printf "%-6s (INCONCLUSIVE)\n", oe; exit }
            printf "%-6s %-10.1f %-10.1f %-8.3f %.0f\n",
                   oe, low*50, row*50, duty, 1e6/(row*54)
        }'
done
