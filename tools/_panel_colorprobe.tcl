# 色序探针: 数字 fb 已灌前提下, 轮播 3 组 mask, 每条只亮 1 个单色数字
# 组1 {0,4,8} / 组2 {1,5,6} / 组3 {2,3,7}, 每组 12s, 循环
proc log {m} { puts "\[cp [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }
if {[catch { connect -url tcp:127.0.0.1:3122 }]} { connect }
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
set groups {
    {A 0x111 "lane 0/4/8"}
    {B 0x062 "lane 1/5/6"}
    {C 0x08C "lane 2/3/7"}
}
while {1} {
    foreach g $groups {
        lassign $g name mask desc
        mwr 0x4001000C $mask
        mwr 0x4001000C 0xC1000003
        log "===== 组$name $desc ====="
        after 12000
    }
}
