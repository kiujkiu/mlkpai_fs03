# v5 纯寄存器切 sensor 模式 + rev_period 轮询 (不烧 bit 不灌 DDR, PL 必须已活)
# 用法: xsct _pov_sensor_sw.tcl [轮询秒数=40]
set SECS [expr {$argc > 0 ? [lindex $argv 0] : 40}]
proc log {m} { puts "\[pov [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3

# 切 sensor: n_slices=360 | pov_en, 清 fake_en
mwr 0x40010010 [expr {(360 << 16) | 0x1}]
log "已切 sensor 模式, 等 W6 光电脉冲 — 现在手拨转盘!"

for {set i 0} {$i < $SECS} {incr i} {
    set st  [lindex [mrd -value 0x40010010] 0]
    set rp  [lindex [mrd -value 0x40010014] 0]
    log "t=${i}s slice_idx=[expr {$st & 0xFFFF}] rev_period=[format 0x%08x $rp]"
    after 1000
}
log "轮询结束"
exit
