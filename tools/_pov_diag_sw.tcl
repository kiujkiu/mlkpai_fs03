# 64-bit 对交换验证: 灌 swapped diag → fetch 模式
proc log {m} { puts "\[sw [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }
catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
dow -data tools/diag_slice_sw.bin 0x11000000
mwr 0x40010010 0x0
mwr 0x40010018 0x11000000
mwr 0x40010010 [expr {(1 << 16) | 0x3}]
after 300
log "swapped diag 已上, STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] — 期望: 正确 15/45 交替阶梯"
exit
