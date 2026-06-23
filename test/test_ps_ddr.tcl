# MLKPAI PS7 基础配置下载测试 (xsct): 跑我们 XSA 出的 ps7_init + DDR 读写验证
# 前提: 板子 boot mode = JTAG (PIN1 PIN2 ON-ON) + 冷循环 (PS idle, DDR 未初始化)
# 用法 (WSL): cmd.exe /c "...xsct.bat ...\test\test_ps_ddr.tcl"  (cwd 在 test/)
connect
after 600
targets -set -nocase -filter {name =~ "APU*" || name =~ "*Cortex-A9*#0*" || name =~ "ARM*#0"}
catch { stop }
rst -processor
after 300
puts "=== source 我们的 ps7_init (DDR3L MT41K256M16/1066 + MIO + 33.33M 时钟) ==="
source ps7_init.tcl
ps7_init
ps7_post_config
puts "=== ps7_init 完成. 开始 DDR 读写测试 ==="
configparams force-mem-access 1

# 1) 几个关键地址定模式读回
set ok 1
foreach {addr pat} {0x00100000 0xA5A5A5A5 0x08000000 0x5A5A5A5A 0x10000000 0xDEADBEEF 0x1FF00000 0xCAFEBABE} {
  mwr -force $addr $pat
  set rb [lindex [mrd -force -value $addr] 0]
  if {$rb == $pat} {
    puts [format "  DDR %s = 0x%08X  OK" $addr $rb]
  } else {
    puts [format "  DDR %s FAIL: wrote 0x%08X read 0x%08X" $addr $pat $rb]; set ok 0
  }
}

# 2) 1MB 地址递增块测试 (addr=data, 步进 4KB)
puts "=== 块测试 0x00100000..0x00200000 (256 点) ==="
set fail 0
for {set a 0x100000} {$a < 0x200000} {incr a 0x1000} {
  mwr -force $a $a
  set rb [lindex [mrd -force -value $a] 0]
  if {$rb != $a} { incr fail }
}
puts "  块测试: [expr {256 - $fail}]/256 OK"

if {$ok && $fail == 0} {
  puts ">>> ✅ DDR 配置验证通过 — 芯片 PS (DDR/时钟) 跑通"
} else {
  puts ">>> ⚠ DDR 有错 — 检查 DDR 时序配置 (MT41K256M16/1066 通用配可能要对齐米联客)"
}
disconnect
exit 0
