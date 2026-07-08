# 分步验证烧写: 每步显式打印, 不静默
proc log {m} { puts "\[fc\] $m" }
connect
log "jtag: [jtag targets]"
catch { jtag targets -set -filter {name =~ "*MLK*"} }
catch { jtag frequency 5000000 }
log "freq set 5M"
catch { targets -set -filter {name =~ "*Cortex-A9*#0*"}; stop } r1
log "stop core0: $r1"
targets -set -filter {jtag_cable_name =~ "*2515BCEF4DEA*" && name =~ "*xc7z020*"}
if {[catch { fpga -file build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit } r2]} {
    log "FPGA FAIL: $r2"
} else {
    log "FPGA OK: $r2"
}
targets -set -filter {name =~ "APU*"}
memmap -addr 0xF8007000 -size 0x100 -flags 3
catch { log "devcfg INT_STS=[format 0x%08x [lindex [mrd -value 0xF800700C] 0]] (bit2=PCFG_DONE)" } r3
exit
