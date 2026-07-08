# v4 overlap+25M 7 色循环: 全屏实心 fb (oe_window=48 → 白场 3.8V 轨 ~1.9A 安全)
# 自愈: PL 配置丢 → 同会话重烧+重灌+重配, 限 3 次
set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
proc log {m} { puts "\[o8 [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

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
proc fill_fb {} {
    set ones {}
    for {set i 0} {$i < 64} {incr i} { lappend ones 0xFFFFFFFF }
    for {set lane 0} {$lane < 9} {incr lane} {
        for {set blk 0} {$blk < 8} {incr blk} {
            mwr -size w [expr {0x40018000 + $lane*0x800 + $blk*0x100}] $ones 64
        }
    }
    log "fb filled (全屏实心)"
}
proc apply_cfg {} {
    mwr 0x4001000C 0xC0000000                 ;# auto 停 (dclk_fast 不带载切)
    after 50
    mwr 0x4001000C [expr {0x80000000 | (1<<27) | (1<<29) | (1<<28) | (48<<8) | (54<<16) | 1}]
    log "cfg: overlap+25M oe_window=48 rows=54"
}

catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
catch { jtag targets -set -filter {name =~ "*MLK*"}; jtag frequency 5000000 }
source ps7/ps7_init.tcl
targets -set -filter {name =~ "APU*"}
memmap -addr 0x40010000 -size 0x10000 -flags 3
if {$argc > 0 && [lindex $argv 0] eq "force"} {
    log "force: 停 auto + 完整烧写 (新 bit)"
    catch { mwr 0x4001000C 0xC0000000; after 100 }
    program_board
} elseif {[catch { mrd -value 0x40010000 } st]} {
    log "PL 无响应 ($st) — 完整烧写"
    program_board
} else {
    log "PL 已在跑 (STATUS=[format 0x%08x [lindex $st 0]]), 跳过烧写"
}
fill_fb
apply_cfg

proc set_color {mask} {
    mwr 0x4001000C [expr {0x80000000 | (54 << 16) | 1}]   ;# rows=54 幂等 (cfg_we=0 不碰 overlap 配置)
    mwr 0x4001000C $mask
    mwr 0x4001000C 0xC1000003                             ;# auto+use_fb (disp 字段 overlap 下不用)
    set st [lindex [mrd -value 0x40010000] 0]
    # 健康 = 高位干净 + auto(bit4) + overlap(bit6) + fast(bit7) 全在
    if {($st & 0xFFFFFF00) != 0 || ($st & 0xD0) != 0xD0} {
        log "STATUS 异常 [format 0x%08x $st], 重配"
        after 50
        apply_cfg
        mwr 0x4001000C $mask
        mwr 0x4001000C 0xC1000003
    }
}
set seq {
    {红 0x049} {绿 0x092} {蓝 0x124}
    {黄 0x0DB} {品红 0x16D} {青 0x1B6} {白 0x1FF}
}
set reflash_cnt 0
while {1} {
    foreach c $seq {
        lassign $c name mask
        if {[catch { set_color $mask } err]} {
            log "JTAG 异常: $err"
            if {[string match "*not programmed*" $err] || [string match "*Blocked address*" $err]} {
                if {$reflash_cnt >= 3} {
                    log "自愈重烧 3 次仍丢配置 — 停循环等人工"
                    break
                }
                incr reflash_cnt
                log "PL 配置丢失 — 同会话重烧 (第 $reflash_cnt/3 次)"
                if {[catch { program_board; fill_fb; apply_cfg } e2]} {
                    log "重烧失败: $e2 — 10s 后再试一次"
                    after 10000
                    catch { program_board; fill_fb; apply_cfg }
                }
            } else {
                after 3000
            }
            catch { set_color $mask }
        }
        log "===== $name ====="
        after 6000
    }
}
log "循环已退出"
