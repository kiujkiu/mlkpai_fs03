# FS03 首点亮: 烧 bit + ps7_init + ICND2049 静态稀疏图形 (单色单行, 限流保守)
# ⚠ 双 cable 纪律: FS03=MLK.JTAG1U1 2515BCEF4DEA; 板1 SMT2 210251A08870 绝不碰
# ⚠ 若 fpga 挂死: 拔 SD + 冷循环再跑 (SOP feedback_zynq_jtag_flash_cold_board_sop)
# 用法: xsct tools/_panel_light.tcl  (工作目录 = 仓根)

set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
set PS7I ps7/ps7_init.tcl
set BASE 0x40010000

proc log {m} { puts "\[panel\] $m" }

connect

# --- 停 ARM (FS03 上 rst -processor 会挂死, 只 stop 不 rst) ---
targets -set -filter {name =~ "*Cortex-A9*#0*"}
catch { stop }
catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
log "ARM stopped"

# --- 烧 bit (按 SN 选 FS03 的 xc7z020) ---
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
fpga -file $BIT
log "bitstream loaded"

# --- ps7_init (新 XSA 的, 含 GP0 + FCLK0=50M) ---
# 选 APU 目标 = DAP 物理地址访问, 绕开 halted Linux 的 MMU 映射
targets -set -filter {name =~ "APU*"}
source $PS7I
ps7_init
ps7_post_config
log "ps7_init + post_config done"
# APU 目标默认挡 PL AXI 地址, 显式加映射
memmap -addr 0x40010000 -size 0x10000 -flags 3

# --- panel 寄存器 ---
proc preg {off} { expr {0x40010000 + $off} }
proc cmd_w {data le mode} { mwr [preg 0] [expr {($data & 0xFFFF) | (($le & 0x7F)<<16) | (($mode & 0x3)<<24)}] }
proc burst {n} { mwr [preg 4] $n }
proc row_adv {sdi} { mwr [preg 8] $sdi; after 1 }
proc misc {v} { mwr [preg 0xC] $v }
proc pstat {} { log "STATUS=[format 0x%08x [lindex [mrd -value [preg 0]] 0]]" }

pstat
# 1) 消隐 (复位默认已是 1, 再显式设一次)
misc 0x80000001
# 2) 只开 R 三路: sdi_mask = 0b001001001
misc 0x00000049
# 3) 灌 12 words/lane: 前 11 个无 LE (burst=10 → 11 发), 最后 1 个 LE=1 锁存
#    0x8000 = 每颗芯片只亮 OUT0 → 12 点/lane × 3 R-lane = 36 LED ≈ 0.5A, 限流保守
#    ⚠ 2049 LE 长度编码: 3=普通锁存 4=换行(行+1) 5=首行锁存, LE<3 不锁存!
burst 10
cmd_w 0x8000 0 0
after 20
cmd_w 0x8000 5 0
after 20
pstat
# 4) 3019: 选通 2 行 (推 2 个 '1' 进链, ~1A 安全)
row_adv 1
row_adv 1
# 5) 开显示 (OE 下降沿把 latch1 转移进 reg2 并点亮)
misc 0x80000000
pstat
log "预期: 一行稀疏红点 (每芯片 1 点). 换绿: misc 0x92 后重灌; 换蓝: 0x124"
log "OE 关: misc 0x80000001 / 行前进: row_adv 0 (走灯) "
exit
