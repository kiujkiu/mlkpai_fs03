# MLKPAI PS7 下载测试 — 每步写 ddr_progress.txt + flush (挂住也能看到最后步)
set ::LOG [open "D:/claude_workspace/pov3d/mlkpai_fs03/test/ddr_progress.txt" w]
proc plog {m} { puts $m; catch {flush stdout}; puts $::LOG $m; flush $::LOG }

plog "STEP0 connect"
connect
after 1500
plog "STEP1 选 Cortex-A9 #0"
targets -set -nocase -filter {name =~ "*Cortex-A9*#0*"}
plog "STEP1a 核已选"
if {[catch {stop} e]} { plog "  stop 警告: $e" }
plog "STEP1b 已 stop (跳过 rst -processor)"
after 500

plog "STEP2-pre source ps7_init.tcl"
if {[catch {source ps7_init.tcl} e]} { plog "  source FAIL: $e"; exit 1 }
plog "STEP2 跑 ps7_init (DDR/MIO/时钟) — 若卡这里=DDR配置不对"
if {[catch {ps7_init} e]} { plog "  ps7_init FAIL: $e"; exit 1 }
catch {ps7_post_config}
plog "STEP3 ps7_init 完成! 开始 DDR 读写"
configparams force-mem-access 1

set ok 1
foreach {addr pat} {0x00100000 0xA5A5A5A5 0x10000000 0xDEADBEEF 0x1FF00000 0xCAFEBABE} {
  if {[catch {mwr -force $addr $pat; set rb [lindex [mrd -force -value $addr] 0]} e]} {
    plog "  DDR $addr FAIL: $e"; set ok 0; continue }
  if {$rb == $pat} { plog [format "  DDR %s = 0x%08X OK" $addr $rb] } \
  else { plog [format "  DDR %s FAIL wrote 0x%08X read 0x%08X" $addr $pat $rb]; set ok 0 }
}
plog "STEP4 块测试 8 点"
set fail 0
for {set a 0x100000} {$a < 0x108000} {incr a 0x1000} {
  mwr -force $a $a; if {[lindex [mrd -force -value $a] 0] != $a} { incr fail } }
plog "  块: [expr {8-$fail}]/8 OK"
if {$ok && $fail==0} { plog "RESULT ✅ DDR 跑通" } else { plog "RESULT ⚠ DDR 有错" }
close $::LOG
disconnect
exit 0
