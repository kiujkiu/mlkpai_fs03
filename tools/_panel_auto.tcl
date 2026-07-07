# v2: 烧 bit + ps7_init + 开 PL 自主扫描 (之后无需 JTAG, 板子自己刷屏)
# 参数: [color] = R|G|B|W (默认 W=全开), [pattern] 16bit hex (默认 0x8000)
set color   [expr {$argc > 0 ? [lindex $argv 0] : "W"}]
set pattern [expr {$argc > 1 ? [lindex $argv 1] : 0x8000}]
set rows    [expr {$argc > 2 ? [lindex $argv 2] : 0}]
set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
proc log {m} { puts "\[auto\] $m" }

connect
targets -set -filter {name =~ "*Cortex-A9*#0*"}
catch { stop }
catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
fpga -file $BIT
log "bitstream loaded"
targets -set -filter {name =~ "APU*"}
source ps7/ps7_init.tcl
ps7_init
ps7_post_config
memmap -addr 0x40010000 -size 0x10000 -flags 3
log "ps7 ready"

array set MASKS {R 0x49 G 0x92 B 0x124 W 0x1FF}
mwr 0x4001000C [expr {$MASKS($color)}]                        ;# 颜色 (sdi_mask)
if {$rows > 0} { mwr 0x4001000C [expr {0x80000000 | (($rows & 0x1FF) << 16) | 1}] }  ;# 行数
mwr 0x4001000C [expr {0xC0000000 | (($pattern & 0xFFFF) << 8) | 1}]  ;# auto on
after 200
log "STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] (bit4=auto)"
log "自主扫描运行中: color=$color pattern=[format 0x%04X $pattern]"
exit
