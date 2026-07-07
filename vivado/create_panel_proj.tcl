# MLKPAI-FS03 + icnd2049_panel_seq — 首点亮工程 (2026-07-07)
# 用法:
#   cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03 && \
#     call C:\Xilinx\Vivado\2024.2\settings64.bat && \
#     vivado -mode batch -source vivado\create_panel_proj.tcl"
# 产物: build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit + mlkpai_panel.xsa

set PROJ  mlkpai_panel
set PART  xc7z020clg484-1
set DIR   [file normalize [file dirname [info script]]/..]
set BUILD $DIR/build_panel

create_project $PROJ $BUILD -part $PART -force

# RTL + XDC
add_files -norecurse $DIR/vivado/hdl/icnd2049_panel_fb.v
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins.xdc

create_bd_design system

# ---- PS7 (与 create_zynq_ps.tcl 同配置 + GP0 开 + FCLK0=50M) ----
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
  -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps

set_property -dict [list \
  CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
  CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_USE_M_AXI_GP0 {1} \
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
  CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {50} \
  CONFIG.PCW_FCLK_CLK0_BUF {TRUE} \
] $ps

# ---- panel IP (module reference) ----
set panel [create_bd_cell -type module -reference icnd2049_panel_fb panel_0]

# AXI 连接自动化 (生成 axi 互联 + proc reset, 时钟用 FCLK0)
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config \
  [list Master {/ps7_0/M_AXI_GP0} Clk {Auto}] [get_bd_intf_pins panel_0/s_axi]

# 基址对齐老 panel_seq 习惯: 0x40010000 / 4KB... 用 64K 段
assign_bd_address
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7_0/Data] -filter {NAME =~ *panel*}]
if {[llength $seg] == 1} {
    set_property offset 0x40010000 $seg
    set_property range  64K        $seg
}

# ---- 外部端口 (P1 + P3 双口同波形, 屏插哪个都亮) ----
proc out_port2 {name pin} {
    create_bd_port -dir O $name
    create_bd_port -dir O ${name}_2
    connect_bd_net [get_bd_pins $pin] [get_bd_ports $name] [get_bd_ports ${name}_2]
}
out_port2 panel_dclk     panel_0/dclk_out
out_port2 panel_lat      panel_0/le_out
out_port2 panel_oe       panel_0/oe_out
out_port2 panel_row_dclk panel_0/icnd_dclk_out
out_port2 panel_row_rclk panel_0/icnd_rclk_out
out_port2 panel_row_sdi  panel_0/icnd_sdi_out
create_bd_port -dir O -from 8 -to 0 panel_sdi
create_bd_port -dir O -from 8 -to 0 panel_sdi_2
connect_bd_net [get_bd_pins panel_0/sdi_out] [get_bd_ports panel_sdi] [get_bd_ports panel_sdi_2]

validate_bd_design
save_bd_design

generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top
add_files -norecurse $BUILD/$PROJ.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

# ---- build ----
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    puts "BUILD_FAILED: impl_1 progress [get_property PROGRESS [get_runs impl_1]]"
    exit 1
}
open_run impl_1
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "TIMING_WNS: $wns"

write_hw_platform -fixed -include_bit -force $DIR/mlkpai_panel.xsa
puts "BUILD_OK: $BUILD/$PROJ.runs/impl_1/system_wrapper.bit"
