# 只读: 查 PLL 状态 + FCLK0 配置
connect
targets -set -filter {name =~ "APU*"}
memmap -addr 0xF8000000 -size 0x1000 -flags 3
foreach {name addr} {ARM_PLL_CTRL 0xF8000100 DDR_PLL_CTRL 0xF8000104 IO_PLL_CTRL 0xF8000108 PLL_STATUS 0xF800010C FPGA0_CLK_CTRL 0xF8000170 ARM_CLK_CTRL 0xF8000120} {
    puts "$name = [format 0x%08x [lindex [mrd -value $addr] 0]]"
}
exit
