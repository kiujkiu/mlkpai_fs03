# 灌棋盘格 + 开 fb 显示 (前提: v3 bit 已烧 + ps7 已初始化)
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
mwr 0x4001000C 0x000001FF                  ;# 9 路全开
mwr 0x4001000C [expr {0x80000000 | (54 << 16) | 1}]  ;# rows=54
source tools/chess160_fb.tcl               ;# 灌帧 (486 行 bulk mwr)
mwr 0x4001000C 0xC0000003                  ;# auto_en + use_fb
after 200
puts "\[chess\] STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] (bit5=use_fb bit4=auto)"
exit
