# lane 轮播探针: fb 全 1, 单 lane 每 6s 轮流亮, 定 lane↔(区,色) 映射
# 用法: xsct _panel_lanewalk.tcl  (前提: v4 bit 已烧+ps7 初始化, overlap cfg 沿用)
proc log {m} { puts "\[lw [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }
if {[catch { connect -url tcp:127.0.0.1:3122 }]} { connect }
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
set ones {}
for {set i 0} {$i < 64} {incr i} { lappend ones 0xFFFFFFFF }
for {set lane 0} {$lane < 9} {incr lane} {
    for {set blk 0} {$blk < 8} {incr blk} {
        mwr -size w [expr {0x40018000 + $lane*0x800 + $blk*0x100}] $ones 64
    }
}
mwr 0x4001000C [expr {0x80000000 | (54 << 16) | 1}]   ;# rows=54
log "fb 全1, 开始 lane 轮播 (每 lane 6s, 0→8 循环)"
while {1} {
    for {set lane 0} {$lane < 9} {incr lane} {
        mwr 0x4001000C [expr {1 << $lane}]             ;# 单 lane mask
        mwr 0x4001000C 0xC1000003                      ;# auto+use_fb
        log "===== lane $lane ====="
        after 6000
    }
}
