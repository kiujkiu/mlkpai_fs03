# v5 POV 上板一条龙: 烧 bit + DDR 灌 slice 镜像 + POV 模式
# 用法: xsct _panel_pov.tcl [fake|sensor] [rps]   (fake 默认, rps 默认 0.5 慢转肉眼看切片)
# fake = 无电机无光电, angle_tracker 自由跑, 静止屏轮播切片 = 全管线验证
set MODE [expr {$argc > 0 ? [lindex $argv 0] : "fake"}]
set RPS  [expr {$argc > 1 ? [lindex $argv 1] : 0.5}]
set BIT  build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit
set BIN  tools/anime_slices.bin
set BASE 0x10000000
proc log {m} { puts "\[pov [clock format [clock seconds] -format %H:%M:%S]\] $m"; flush stdout }

catch { exec cmd /c start /b hw_server -d -s tcp::3122 }
after 2000
connect -url tcp:127.0.0.1:3122
catch { jtag targets -set -filter {name =~ "*MLK*"}; jtag frequency 5000000 }
source ps7/ps7_init.tcl
catch { targets -set -filter {name =~ "*Cortex-A9*#0*"}; stop }
catch { targets -set -filter {name =~ "*Cortex-A9*#1*"}; stop }
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
fpga -file $BIT
targets -set -filter {name =~ "APU*"}
ps7_init
ps7_post_config
memmap -addr 0x40010000 -size 0x10000 -flags 3
log "v5 bit + ps7 OK"

# DDR 灌 slice 镜像 (dow -data 直载, 远快于 mwr)
dow -data $BIN $BASE
set w0 [lindex [mrd -value $BASE] 0]
log "DDR 镜像已灌 @[format 0x%08x $BASE], word0=[format 0x%08x $w0]"

# panel 基础配置: 9 路全开 + overlap 25M 1/4 亮度 + rows 54
mwr 0x4001000C 0x000001FF
mwr 0x4001000C [expr {0x80000000 | (1<<27) | (1<<29) | (1<<28) | (48<<8) | (54<<16) | 1}]

# POV 配置
mwr 0x40010018 $BASE
if {$MODE eq "fake"} {
    set fp [expr {int(50000000.0 / ($RPS * 360))}]
    mwr 0x40010014 $fp
    mwr 0x40010010 [expr {(360 << 16) | 0x3}]     ;# n_slices=360 | fake_en | pov_en
    log "fake 模式: $RPS rps, fake_period=$fp aclk/slice"
} else {
    mwr 0x40010010 [expr {(360 << 16) | 0x1}]     ;# 真光电: 只 pov_en
    log "sensor 模式: 等 W6 光电脉冲 (0x14 读 rev_period 验证)"
}
mwr 0x4001000C 0xC1000003                          ;# auto + use_fb

after 500
set s1 [lindex [mrd -value 0x40010010] 0]
after 500
set s2 [lindex [mrd -value 0x40010010] 0]
log "STATUS=[format 0x%08x [lindex [mrd -value 0x40010000] 0]]"
log "slice_idx: [expr {$s1 & 0xFFFF}] -> [expr {$s2 & 0xFFFF}] (在动=管线活)"
log "rev_period=[format 0x%08x [lindex [mrd -value 0x40010014] 0]]"
exit
