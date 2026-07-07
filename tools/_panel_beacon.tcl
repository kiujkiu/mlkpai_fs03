# 慢信标: OE 2Hz 方波 + 每秒 1 簇 DCLK/SDI, 任何采样率可见
proc log {m} { puts "\[beacon\] $m" }
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
proc preg {off} { expr {0x40010000 + $off} }
proc cmd_w {data le mode} { mwr [preg 0] [expr {($data & 0xFFFF) | (($le & 0x7F)<<16) | (($mode & 0x3)<<24)}] }
proc misc {v} { mwr [preg 0xC] $v }
proc row_adv {sdi} { mwr [preg 8] $sdi }
misc 0x000001FF
log "信标模式: OE(P3.10) 2Hz 方波 / DCLK(P3.8)+9路SDI(0xFFFF) 每秒1簇1.28us / 行C(P3.25) 1s翻转"
for {set i 0} {$i < 100000} {incr i} {
    misc 0x80000000            ;# OE=0
    after 250
    misc 0x80000001            ;# OE=1
    after 250
    if {$i & 1} { cmd_w 0xFFFF 0 0; row_adv [expr {($i>>1) & 1}] }
    if {$i % 40 == 0} { log "beacon $i" }
}
exit
