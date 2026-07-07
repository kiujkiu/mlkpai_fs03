# 运行时改扫描行数: xsct _panel_rows.tcl <n>
set rows [lindex $argv 0]
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
mwr 0x4001000C [expr {0x80000000 | (($rows & 0x1FF) << 16) | 1}]
puts "\[rows\] rows=$rows STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]]"
exit
