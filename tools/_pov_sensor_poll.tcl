# sensor 加强轮询: 0x10 slice_idx / 0x14 rev_period / 0x18 locked_ever+slice_max
proc log {m} { puts "\[sp [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }
set SECS [expr {$argc > 0 ? [lindex $argv 0] : 90}]
catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
mwr 0x40010010 [expr {(360 << 16) | 0x1}]   ;# sensor 模式 (清 fake)
log "sensor 模式, 拨盘!"
for {set i 0} {$i < $SECS} {incr i} {
    set st [lindex [mrd -value 0x40010010] 0]
    set rp [lindex [mrd -value 0x40010014] 0]
    set pk [lindex [mrd -value 0x40010018] 0]
    log "t=$i idx=[expr {$st & 0xFFFF}] rev=[format %x $rp] peak=[format %08x $pk] lock=[expr {$st >> 31}]"
    after 1000
}
exit
