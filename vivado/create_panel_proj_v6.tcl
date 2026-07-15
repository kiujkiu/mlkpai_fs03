# MLKPAI-FS03 + pov_dual_top (v6) — 双屏 ICND2047 双沿 POV 工程 (2026-07-15)
#
# 相对 v5 的差异 (其余照抄 create_panel_proj_v5.tcl, 含全部坑规避):
#   - module ref: icnd2049_panel_pov → pov_dual_top (全新模块名, 天然无缓存坑)
#   - 源码 4 文件: pov_dual_top + panel_engine_2047(适配壳) + icnd2047_panel_core
#     + row_drv_icnd1028  (stub 不入工程!)
#   - 外部口: v5 的 P1/P3 镜像 → A/B 独立 (端口名不变: 原名=A/P1, _2=B/P3,
#     panel_pins.xdc 零改动)
#   - 跑法同 v5:
#     cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03 && \
#       call C:\Xilinx\Vivado\2024.2\settings64.bat && \
#       vivado -mode batch -source vivado\create_panel_proj_v6.tcl"

set PROJ  mlkpai_panel
set PART  xc7z020clg484-1
set DIR   [file normalize [file dirname [info script]]/..]
set BUILD $DIR/build_panel

file delete -force $BUILD
create_project $PROJ $BUILD -part $PART -force

foreach f {angle_tracker.v pov_dual_top.v panel_engine_2047.v icnd2047_panel_core.v row_drv_icnd1028.v} {
    if {![file exists $DIR/vivado/hdl/$f]} { puts "FATAL: vivado/hdl/$f 不存在"; exit 1 }
    add_files -norecurse $DIR/vivado/hdl/$f
}
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins.xdc
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins_v5.xdc

create_bd_design system

# ---- PS7 (与 v5 完全一致, ps7_init 不变 → fsbl.elf 可复用) ----
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
  -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps
set_property -dict [list \
  CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
  CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
  CONFIG.PCW_USE_M_AXI_GP0 {1} \
  CONFIG.PCW_USE_S_AXI_HP0 {1} \
  CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {32} \
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

# ---- panel IP v6 (cell 名保持 panel_0 → OOC run 名 / tools 不变) ----
set panel [create_bd_cell -type module -reference pov_dual_top panel_0]

if {[llength [get_bd_intf_pins -quiet panel_0/s_axi]] == 0} {
    puts "FATAL: panel_0/s_axi 接口没推断出来"; exit 1 }
if {[llength [get_bd_intf_pins -quiet panel_0/m_axi]] == 0} {
    puts "FATAL: panel_0/m_axi 接口没推断出来"; exit 1 }
if {[llength [get_bd_pins -quiet panel_0/spin_sync]] == 0} {
    puts "FATAL: panel_0/spin_sync pin 不存在"; exit 1 }
catch { set_property CONFIG.ASSOCIATED_BUSIF {s_axi:m_axi} [get_bd_pins panel_0/s_axi_aclk] }

apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config \
  [list Master {/ps7_0/M_AXI_GP0} Clk {Auto}] [get_bd_intf_pins panel_0/s_axi]

set smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_smc_hp0]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $smc
connect_bd_intf_net [get_bd_intf_pins panel_0/m_axi]       [get_bd_intf_pins axi_smc_hp0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_smc_hp0/M00_AXI] [get_bd_intf_pins ps7_0/S_AXI_HP0]
set clk_net [get_bd_nets -of_objects [get_bd_pins ps7_0/FCLK_CLK0]]
connect_bd_net -net $clk_net [get_bd_pins axi_smc_hp0/aclk]
connect_bd_net -net $clk_net [get_bd_pins ps7_0/S_AXI_HP0_ACLK]
set rst_cell [lindex [get_bd_cells -filter {VLNV =~ "xilinx.com:ip:proc_sys_reset:*"}] 0]
set rst_net  [get_bd_nets -of_objects [get_bd_pins $rst_cell/peripheral_aresetn]]
connect_bd_net -net $rst_net [get_bd_pins axi_smc_hp0/aresetn]

assign_bd_address
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7_0/Data] -filter {NAME =~ *panel*}]
if {[llength $seg] == 1} {
    set_property offset 0x40010000 $seg
    set_property range  64K        $seg
} else { puts "FATAL: AXI-Lite panel 段找不到/不唯一: $seg"; exit 1 }
set mseg [get_bd_addr_segs -quiet -of_objects [get_bd_addr_spaces panel_0/m_axi] -filter {NAME =~ *HP0*}]
if {[llength $mseg] == 0} {
    assign_bd_address -target_address_space [get_bd_addr_spaces panel_0/m_axi] \
        [get_bd_addr_segs ps7_0/S_AXI_HP0/HP0_DDR_LOWADDR]
    set mseg [get_bd_addr_segs -of_objects [get_bd_addr_spaces panel_0/m_axi] -filter {NAME =~ *HP0*}]
}
if {[llength $mseg] != 1} { puts "FATAL: m_axi HP0 DDR 段找不到/不唯一: $mseg"; exit 1 }
set_property offset 0x00000000 $mseg
set_property range  512M       $mseg
puts "ADDR_MAP m_axi -> [get_property NAME $mseg] @ [get_property OFFSET $mseg] range [get_property RANGE $mseg]"

# ---- 外部端口: A 屏=原名 (P1), B 屏=_2 (P3) — 独立波形, XDC 端口名不变 ----
proc out_ab {name pinA pinB} {
    create_bd_port -dir O $name
    create_bd_port -dir O ${name}_2
    connect_bd_net [get_bd_pins $pinA] [get_bd_ports $name]
    connect_bd_net [get_bd_pins $pinB] [get_bd_ports ${name}_2]
}
out_ab panel_dclk     panel_0/dclk_out      panel_0/dclk_out_2
out_ab panel_lat      panel_0/le_out        panel_0/le_out_2
out_ab panel_oe       panel_0/oe_out        panel_0/oe_out_2
out_ab panel_row_dclk panel_0/icnd_dclk_out panel_0/icnd_dclk_out_2
out_ab panel_row_rclk panel_0/icnd_rclk_out panel_0/icnd_rclk_out_2
out_ab panel_row_sdi  panel_0/icnd_sdi_out  panel_0/icnd_sdi_out_2
create_bd_port -dir O -from 8 -to 0 panel_sdi
create_bd_port -dir O -from 8 -to 0 panel_sdi_2
connect_bd_net [get_bd_pins panel_0/sdi_out]   [get_bd_ports panel_sdi]
connect_bd_net [get_bd_pins panel_0/sdi_out_2] [get_bd_ports panel_sdi_2]

create_bd_port -dir I spin_sync_i
connect_bd_net [get_bd_ports spin_sync_i] [get_bd_pins panel_0/spin_sync]

validate_bd_design

proc verify_spin_sync {} {
    set pin [get_bd_pins panel_0/spin_sync]
    set net [get_bd_nets -quiet -of_objects $pin]
    if {$net eq ""} { puts "SPIN_SYNC_FAIL: panel_0/spin_sync 悬空"; exit 1 }
    foreach p [get_bd_pins -quiet -of_objects $net] {
        set c [get_bd_cells -quiet -of_objects $p]
        if {$c ne "" && [string match -nocase "*xlconstant*" [get_property VLNV $c]]} {
            puts "SPIN_SYNC_WARN: 发现 $c 拴住 spin_sync, 删除重连"
            delete_bd_objs $c
        }
    }
    if {[get_bd_nets -quiet -of_objects [get_bd_pins panel_0/spin_sync]] eq ""} {
        connect_bd_net [get_bd_ports spin_sync_i] [get_bd_pins panel_0/spin_sync]
    }
    set net   [get_bd_nets -of_objects [get_bd_pins panel_0/spin_sync]]
    set ports [get_bd_ports -quiet -of_objects $net]
    set ok 0
    foreach po $ports { if {[string match "*spin_sync_i" [get_property PATH $po]]} { set ok 1 } }
    puts "SPIN_SYNC_NET: $net  端点 pins=[get_bd_pins -quiet -of_objects $net] ports=$ports"
    if {!$ok} { puts "SPIN_SYNC_FAIL: spin_sync_i port 不在 net 上"; exit 1 }
    puts "SPIN_SYNC_OK"
}
verify_spin_sync
validate_bd_design
save_bd_design

generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top
add_files -norecurse $BUILD/$PROJ.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

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
