# 8 色循环 · 自愈版: 单 session 烧+初始化+无限循环
# 每步幂等重写 rows/mask/auto + 回读校验, STATUS 异常自动重初始化
set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
proc log {m} { puts "\[8c\] $m" }

# 独立端口, 绕开可能僵死的默认 3121 hw_server
exec cmd /c start /b hw_server -d -s tcp::3122
after 2000
connect -url tcp:127.0.0.1:3122
targets -set -filter {name =~ "*Cortex-A9*#0*"}
catch { stop }
catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
fpga -file $BIT
targets -set -filter {name =~ "APU*"}
source ps7/ps7_init.tcl
ps7_init
ps7_post_config
memmap -addr 0x40010000 -size 0x10000 -flags 3
log "board ready"

# fb 全 1 (fb 模式每行全灌 12 词, 瞬态错位一行内自愈; 颜色用 mask 切)
set ones {}
for {set i 0} {$i < 64} {incr i} { lappend ones 0xFFFFFFFF }
for {set lane 0} {$lane < 9} {incr lane} {
    for {set blk 0} {$blk < 8} {incr blk} {
        mwr -size w [expr {0x40018000 + $lane*0x800 + $blk*0x100}] $ones 64
    }
}
log "fb filled"

proc set_color {mask pat} {
    mwr 0x4001000C [expr {0x80000000 | (162 << 16) | 1}]              ;# rows=54 (幂等)
    mwr 0x4001000C $mask                                             ;# 颜色
    mwr 0x4001000C 0xC1000003                                        ;# auto+use_fb, disp=1024 (半占空)
    set st [lindex [mrd -value 0x40010000] 0]
    if {($st & 0xFFFFFFC0) != 0 || (($st >> 4) & 1) != 1} {
        log "STATUS 异常 [format 0x%08x $st], 重写"
        after 50
        mwr 0x4001000C [expr {0x80000000 | (162 << 16) | 1}]
        mwr 0x4001000C $mask
        mwr 0x4001000C 0xC1000003
    }
}
# 青/白整场会超 3.8V 轨 (5.4A>3A), fb 全1模式下先跳过
set seq {
    {红 0x049 0} {绿 0x092 0} {蓝 0x124 0}
}
while {1} {
    foreach c $seq {
        lassign $c name mask pat
        if {[catch { set_color $mask $pat } err]} {
            log "JTAG 异常: $err — 3s 后重试"
            after 3000
            catch { set_color $mask $pat }
        }
        log "===== $name ====="
        after 6000
    }
}
