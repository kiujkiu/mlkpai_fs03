# 诊断切片上板: 灌 diag_slice.bin @0x11000000, POV 指到它, n_slices=1 fake
# 纯寄存器+dow -data, 不烧 bit。用法: xsct _pov_diag.tcl
proc log {m} { puts "\[diag [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3

dow -data tools/diag_slice.bin 0x11000000
log "diag slice @0x11000000, word\[0\]=[format 0x%08x [lindex [mrd -value 0x11000000] 0]] (期望 0x0000ffff)"

mwr 0x40010010 0x0                                 ;# 先停 pov
mwr 0x40010018 0x11000000                          ;# slice_base
mwr 0x40010014 5000000                             ;# fake_period 慢 (无所谓, 只有1片)
mwr 0x40010010 [expr {(1 << 16) | 0x3}]            ;# n_slices=1 | fake | pov
after 500
log "STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] CTRL=[format 0x%08x [lindex [mrd -value 0x40010010] 0]]"
log "期望屏面(上→下): 6 条 15px 亮带阶梯: Y0-14@X区3 / 45-59@X区2偏右 / 60-74@X区2 / 105-119@X区1偏右 / 120-134@X区1 / 165-179@X区0"
exit
