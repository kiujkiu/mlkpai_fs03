# 连续刷信号 ~10 分钟, 给逻辑分析仪/示波器探测用
# 量点 (屏 J1): pin10=DCLKIN(应见 12.5MHz 突发) pin12=LATIN(每帧 5 拍宽脉冲)
#              pin14=GCLKIN/OE(方波) pin24=R1IN(数据) pin28=AIN(3019 时钟脉冲)
# 前提: bit 已烧 + ps7_init 已跑 (先跑过 _panel_light.tcl); 本脚本只 poke 寄存器
set BASE 0x40010000
proc log {m} { puts "\[probe\] $m" }
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
proc preg {off} { expr {0x40010000 + $off} }
proc cmd_w {data le mode} { mwr [preg 0] [expr {($data & 0xFFFF) | (($le & 0x7F)<<16) | (($mode & 0x3)<<24)}] }
proc burst {n} { mwr [preg 4] $n }
proc row_adv {sdi} { mwr [preg 8] $sdi }
proc misc {v} { mwr [preg 0xC] $v }

misc 0x00000049   ;# R lanes
log "开始连续刷 (Ctrl-C 停或等 3000 轮)"
for {set i 0} {$i < 100000} {incr i} {
    misc 0x80000001              ;# OE=1 消隐
    burst 10
    cmd_w 0xF000 0 0             ;# 11 words 无 LE
    cmd_w 0xF000 5 0             ;# 尾字 LE=5 首行锁存
    row_adv [expr {($i % 96) < 1}]  ;# 3019 链里保持 ~4 个 '1' 走灯 (限流), 扫遍全部行
    misc 0x80000000              ;# OE=0 显示 (下降沿转移 reg2)
    after 100
    if {$i % 50 == 0} { log "loop $i STATUS=[format 0x%x [lindex [mrd -value [preg 0]] 0]]" }
}
exit
