# v4 overlap 模式: xsct _panel_overlap.tcl [oe_window DCLK 默认48] [fast 默认1] [overlap 默认1]
# 前提: v4 bit 已烧 + ps7 已初始化 + fb 已灌 (图案沿用 BRAM 现有内容)
# 亮度 = oe_window/192; 48=1/4 (白场 3.8V 轨 ~2.4A 安全)
set oew  [expr {$argc > 0 ? [lindex $argv 0] : 48}]
set fast [expr {$argc > 1 ? [lindex $argv 1] : 1}]
set ovl  [expr {$argc > 2 ? [lindex $argv 2] : 1}]
if {[catch { connect -url tcp:127.0.0.1:3122 }]} { connect }
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
mwr 0x4001000C 0xC0000000                  ;# ① auto 停 (dclk_fast 不能带载切)
after 50
mwr 0x4001000C [expr {0x80000000 | (1<<27) | ($fast<<29) | ($ovl<<28) \
                      | (($oew & 0xFF)<<8) | (54<<16) | 1}]   ;# ② cfg_we + rows=54
mwr 0x4001000C 0xC1000003                  ;# ③ auto_en + use_fb
after 200
set st [lindex [mrd -value 0x40010000] 0]
puts "\[ovl\] oe_window=$oew fast=$fast overlap=$ovl STATUS=[format 0x%08x $st] (bit7=fast bit6=ovl bit5=fb bit4=auto)"
exit
