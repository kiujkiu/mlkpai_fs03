# 8 色循环 · 自愈版: 单 session 烧+初始化+无限循环
# 每步幂等重写 rows/mask/auto + 回读校验, STATUS 异常自动重初始化
# v2: PL 配置丢失(brownout)时同会话重烧 fpga+ps7_init 自愈, 上限 3 次防 DAP churn
set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
proc log {m} { puts "\[8c [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

proc program_board {} {
    global BIT
    catch { targets -set -filter {name =~ "*Cortex-A9*#0*"}; stop }
    catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
    targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
    fpga -file $BIT
    targets -set -filter {name =~ "APU*"}
    ps7_init
    ps7_post_config
    memmap -addr 0x40010000 -size 0x10000 -flags 3
    log "board ready (fpga+ps7_init)"
}

# fb 50% 棋盘 (行 0x20=8词, 奇偶行交替 0xAAAA/0x5555): 点亮像素减半
# → 双色/白也不超 3.8V 轨 3A (全1时青/白 5.4A 会压塌 buck)
proc fill_fb {} {
    set chess {}
    for {set i 0} {$i < 64} {incr i} {
        lappend chess [expr {(($i / 8) % 2) ? 0x55555555 : 0xAAAAAAAA}]
    }
    for {set lane 0} {$lane < 9} {incr lane} {
        for {set blk 0} {$blk < 8} {incr blk} {
            mwr -size w [expr {0x40018000 + $lane*0x800 + $blk*0x100}] $chess 64
        }
    }
    log "fb filled (50% chess)"
}

# 独立端口, 绕开可能僵死的默认 3121 hw_server (已在跑则 start 失败无害)
catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
catch { jtag targets -set -filter {name =~ "*MLK*"}; jtag frequency 5000000 }
source ps7/ps7_init.tcl
# PL 活着就不重烧 (省 fpga 次数防 DAP sticky), 探测失败才走完整烧写
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
if {[catch { mrd -value 0x40010000 } st]} {
    log "PL 无响应 ($st) — 完整烧写"
    program_board
} else {
    log "PL 已在跑 (STATUS=[format 0x%08x [lindex $st 0]]), 跳过烧写"
}
fill_fb

proc set_color {mask pat} {
    mwr 0x4001000C [expr {0x80000000 | (162 << 16) | 1}]              ;# rows=54 (幂等)
    mwr 0x4001000C $mask                                             ;# 颜色
    mwr 0x4001000C 0xC1000003                                        ;# auto+use_fb, disp=1024 (半占空)
    set st [lindex [mrd -value 0x40010000] 0]
    # v4 起 [7:6]=dclk_fast/overlap_en 合法, 健康掩码只看 [31:8]
    if {($st & 0xFFFFFF00) != 0 || (($st >> 4) & 1) != 1} {
        log "STATUS 异常 [format 0x%08x $st], 重写"
        after 50
        mwr 0x4001000C [expr {0x80000000 | (162 << 16) | 1}]
        mwr 0x4001000C $mask
        mwr 0x4001000C 0xC1000003
    }
}
# 50% 棋盘下全 7 色安全: 单色1.35A / 双色2.7A / 白 3.8V轨2.7A+2.8V轨1.35A
set seq {
    {红 0x049 0} {绿 0x092 0} {蓝 0x124 0}
    {黄 0x0DB 0} {品红 0x16D 0} {青 0x1B6 0} {白 0x1FF 0}
}
set reflash_cnt 0
while {1} {
    foreach c $seq {
        lassign $c name mask pat
        if {[catch { set_color $mask $pat } err]} {
            log "JTAG 异常: $err"
            if {[string match "*not programmed*" $err] || [string match "*Blocked address*" $err]} {
                # PL 配置丢 = 板子 brownout 过. 同会话重烧, 不新开连接
                if {$reflash_cnt >= 3} {
                    log "已自愈重烧 3 次仍丢配置 — 供电有硬问题, 停循环等人工 (板子冷循环+查 24V 路)"
                    break
                }
                incr reflash_cnt
                log "PL 配置丢失 — 同会话重烧 (第 $reflash_cnt/3 次)"
                if {[catch { program_board; fill_fb } e2]} {
                    log "重烧失败: $e2 — 10s 后再试一次"
                    after 10000
                    catch { program_board; fill_fb }
                }
            } else {
                after 3000
            }
            catch { set_color $mask $pat }
        }
        log "===== $name ====="
        after 6000
    }
}
log "循环已退出"
