# 运行时换色/换图形 (auto 模式下, 不重烧): xsct _panel_color.tcl R|G|B|W [pattern16]
set color   [expr {$argc > 0 ? [lindex $argv 0] : "W"}]
set pattern [expr {$argc > 1 ? [lindex $argv 1] : 0x8000}]
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
array set MASKS {R 0x49 G 0x92 B 0x124 W 0x1FF}
mwr 0x4001000C [expr {$MASKS($color)}]
mwr 0x4001000C [expr {0xC0000000 | (($pattern & 0xFFFF) << 8) | 1}]
puts "\[color\] $color pattern=[format 0x%04X $pattern] STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]]"
exit
