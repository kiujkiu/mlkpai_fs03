# 通用灌图: xsct _panel_fbload.tcl <fb.tcl 路径> (前提: bit 已烧+ps7 已初始化)
set FB [lindex $argv 0]
if {[catch { connect -url tcp:127.0.0.1:3122 }]} { connect }
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
mwr 0x4001000C [expr {0x80000000 | (54 << 16) | 1}]   ;# rows=54
mwr 0x4001000C 0x000001FF                              ;# 9 路全开
source $FB
mwr 0x4001000C 0xC0000003                              ;# auto + use_fb
after 200
puts "\[fbload\] $FB STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]]"
exit
