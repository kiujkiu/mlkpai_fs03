# 色序验证: R→G→B 每 ~30s 轮换, 单行走灯, 看屏报实际颜色顺序
proc log {m} { puts "\[colors\] $m" }
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
proc preg {off} { expr {0x40010000 + $off} }
proc cmd_w {data le mode} { mwr [preg 0] [expr {($data & 0xFFFF) | (($le & 0x7F)<<16) | (($mode & 0x3)<<24)}] }
proc burst {n} { mwr [preg 4] $n }
proc row_adv {sdi} { mwr [preg 8] $sdi }
proc misc {v} { mwr [preg 0xC] $v }
set masks {0x49 0x92 0x124}
set names {R G B}
for {set i 0} {$i < 100000} {incr i} {
    set phase [expr {($i / 200) % 3}]
    if {$i % 200 == 0} { log "===== 现在驱动: [lindex $names $phase] 通道组 =====" }
    misc 0x80000001
    misc [expr {[lindex $masks $phase]}]
    burst 10
    cmd_w 0xF000 0 0
    cmd_w 0xF000 5 0
    row_adv [expr {($i % 96) < 1}]
    misc 0x80000000
    after 100
}
exit
