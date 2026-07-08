# v4 bit 上板: 停 auto → 烧 → ps7_init → 灌棋盘 fb → v3 兼容模式点亮
# (本 IP 无 DDR master, 热烧前停 auto 只为干净收尾)
proc log {m} { puts "\[v4\] $m"; flush stdout }
if {[catch { connect -url tcp:127.0.0.1:3122 }]} { connect }
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
catch { mwr 0x4001000C 0xC0000000 }        ;# auto 停
after 100
catch { targets -set -filter {name =~ "*Cortex-A9*#0*"}; stop }
catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
fpga -file build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
targets -set -filter {name =~ "APU*"}
source ps7/ps7_init.tcl
ps7_init
ps7_post_config
memmap -addr 0x40010000 -size 0x10000 -flags 3
log "v4 bit + ps7 OK"
mwr 0x4001000C 0x000001FF                  ;# 9 路全开
mwr 0x4001000C [expr {0x80000000 | (54 << 16) | 1}]  ;# rows=54
source tools/chess160_fb.tcl
mwr 0x4001000C 0xC1000003                  ;# v3 兼容: 12.5M 非 overlap disp=1024
after 200
log "compat chess STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] (期望 0x30)"
exit
