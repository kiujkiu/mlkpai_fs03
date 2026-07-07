# 纯线序检查: OE 恒 1 (消隐, 无电流), 所有信号连续跑 ~15 分钟
# 前提: bit 已烧 + ps7_init 已跑 (跑过 _panel_light.tcl 之后即可)
proc log {m} { puts "\[wires\] $m" }
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
proc preg {off} { expr {0x40010000 + $off} }
proc cmd_w {data le mode} { mwr [preg 0] [expr {($data & 0xFFFF) | (($le & 0x7F)<<16) | (($mode & 0x3)<<24)}] }
proc burst {n} { mwr [preg 4] $n }
proc row_adv {sdi} { mwr [preg 8] $sdi }
proc misc {v} { mwr [preg 0xC] $v }

misc 0x80000001      ;# OE=1 消隐, 全程不开显示
misc 0x000001FF      ;# 9 路 SDI 全开
log "线序模式: 全信号连续跑, OE 恒高 (屏不亮属预期), Ctrl-C 停"
for {set i 0} {$i < 500000} {incr i} {
    burst 10
    cmd_w 0xAAAA 0 0          ;# 11 words 密集数据
    cmd_w 0xAAAA 5 0          ;# 尾字 LE=5 (LAT 上可见 5 拍宽脉冲)
    row_adv [expr {$i & 1}]   ;# 行链 SDI 0/1 交替, DCLK 每轮一个 1.28us 脉冲
    after 50
    if {$i % 100 == 0} { log "loop $i" }
}
exit
