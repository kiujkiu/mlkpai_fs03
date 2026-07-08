# 对照实验: pov_en=0, AXI fb 窗直写 diag 阶梯图案 (等价棋盘已验证路径)
# 期望: 屏上 15/45 交替间距阶梯 (与 fetch 路径显示的均匀 30px 阶梯对比)
proc log {m} { puts "\[diagA [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3

mwr 0x40010010 0x0          ;# pov_en=0 → fb 写口还给 AXI
log "pov off, AXI 直写 fb (2916 词)..."

for {set lane 0} {$lane < 9} {incr lane} {
    for {set row 0} {$row < 54} {incr row} {
        set wlit [expr {$row / 9}]
        set base [expr {0x40018000 + $lane*0x800 + $row*0x20}]
        set vals {}
        for {set w 0} {$w < 6} {incr w} {
            if {$w == $wlit} {
                lappend vals [expr {($w % 2) ? 0xFFFF0000 : 0x0000FFFF}]
            } else {
                lappend vals 0
            }
        }
        mwr $base $vals 6      ;# 一行 6 词一次写
    }
    log "lane $lane done"
}
log "fb 写完, STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]] — 拍屏对比"
exit
