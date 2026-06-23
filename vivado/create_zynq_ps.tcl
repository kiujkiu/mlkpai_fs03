# MLKPAI-FS03 Zynq PS7 基础工程 — Vivado 2024.2
# 依据: docs/board_config.md (米联客手册 + 出厂 dtb 反编译)
# 用法 (WSL 经 cmd.exe 调 Windows vivado, 见 reference_vivado_batch_tcl):
#   cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03 && \
#     call C:\Xilinx\Vivado\2024.2\settings64.bat && \
#     vivado -mode batch -source vivado\create_zynq_ps.tcl"
#
# ⚠ DDR 时序: 米联客没给 ps7 preset, 这里按 MT41K256M16/1066(512MB/16bit) 通用配,
#   上板验内存前先确认 (理想: 拿米联客参考 Vivado 工程对齐 DDR). 出厂 Linux 已证硬件 work.

set PROJ  mlkpai_fs03
set PART  xc7z020clg484-1
set DIR   [file normalize [file dirname [info script]]/..]
set BUILD $DIR/build

create_project $PROJ $BUILD -part $PART -force
create_bd_design system

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_0]
# FIXED_IO + DDR 外部口, 不套 board preset (无 MLKPAI board file, 手动配)
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
  -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps

set_property -dict [list \
  CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
  CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_USE_M_AXI_GP0 {0} \
  CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125} \
  CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {16 Bit} \
  CONFIG.PCW_UIPARAM_DDR_FREQ_MHZ {533.333313} \
  CONFIG.PCW_DDR_RAM_HIGHADDR {0x1FFFFFFF} \
  CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_UART1_UART1_IO {MIO 48 .. 49} \
  CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
  CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
  CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53} \
  CONFIG.PCW_USB0_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_USB0_USB0_IO {MIO 28 .. 39} \
  CONFIG.PCW_SD0_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45} \
  CONFIG.PCW_SD1_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_SD1_SD1_IO {MIO 10 .. 15} \
  CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {1} \
  CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
  CONFIG.PCW_FCLK_CLK0_BUF {TRUE} \
] $ps

validate_bd_design
save_bd_design
make_wrapper -files [get_files system.bd] -top
add_files -norecurse $BUILD/$PROJ.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

# 生成 XSA (给 PetaLinux / 内核 DT / 后续 POV PL)
# write_hw_platform -fixed -force $DIR/mlkpai_fs03.xsa
puts "=== PS7 base 工程建好. 取消注释 write_hw_platform 那行可出 XSA ==="
