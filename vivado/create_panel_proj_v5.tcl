# MLKPAI-FS03 + icnd2049_panel_pov (v5) — POV 工程: spin_sync 输入 + M_AXI 读 DDR (2026-07-08)
#
# ══ v5 一键重建操作序列 (~13 分钟) ══════════════════════════════════════════
#  0) 前置: vivado/hdl/icnd2049_panel_pov.v 必须已存在 (主线在写, 见下方 RTL 接口约定)
#  1) 删 build_panel 全新建 —— 本脚本开头 file delete -force 已自动做。
#     [坑1 规避] BD module_ref 加端口/改位宽会被 BD 缓存旧端口, 所以:
#     模块已改名 icnd2049_panel_pov (≠ 旧 icnd2049_panel_fb) + build 目录整个删掉重建。
#  2) 跑 (WSL 侧发起):
#     cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03 && \
#       call C:\Xilinx\Vivado\2024.2\settings64.bat && \
#       vivado -mode batch -source vivado\create_panel_proj_v5.tcl"
#     [坑3 规避] batch 模式必须先 call settings64.bat, 否则 vivado 不在 PATH / 环境不全。
#  3) 产物 (与 v4 同名, tools/*.tcl 与 refresh_bit 不用改):
#     build_panel/mlkpai_panel.runs/impl_1/system_wrapper.bit + mlkpai_panel.xsa
#  4) 之后只改 RTL/XDC (BD 不动) → vivado\rebuild_panel.tcl 增量 (~5 分钟)。
#     OOC run 名仍是 system_panel_0_0_synth_1 (由 BD 名 system + cell 名 panel_0 决定,
#     与 Verilog 模块名无关), rebuild_panel.tcl 已改成通配 reset 全部 OOC synth run。
# ══════════════════════════════════════════════════════════════════════════
#
# RTL 接口约定 (icnd2049_panel_pov.v 必须满足, 否则本脚本会带错误信息退出):
#   - AXI-Lite slave 端口前缀 s_axi_* (同 v4), 推断出 BD 接口 pin "s_axi"
#   - AXI4 读主端口前缀 m_axi_* (只需 AR/R 通道, 32-bit data), 推断出接口 pin "m_axi"
#   - 单时钟设计: s_axi_aclk 上要标
#       (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axi" *)
#     否则 validate 会报 m_axi 无关联时钟 (脚本里也有 catch 的 set_property 兜底)
#   - input wire spin_sync  (光电, 异步输入)
#   - panel 输出口与 v4 完全同名 (dclk_out/le_out/oe_out/sdi_out[8:0]/icnd_*_out)

set PROJ  mlkpai_panel
set PART  xc7z020clg484-1
set DIR   [file normalize [file dirname [info script]]/..]
set BUILD $DIR/build_panel

# [坑1] 删 build 目录全新建, 杜绝 module_ref 端口缓存
file delete -force $BUILD

create_project $PROJ $BUILD -part $PART -force

# RTL + XDC (v5 源文件; spin_sync 引脚单独放 panel_pins_v5.xdc, 理由见该文件头)
if {![file exists $DIR/vivado/hdl/icnd2049_panel_pov.v]} {
    puts "FATAL: vivado/hdl/icnd2049_panel_pov.v 不存在 (主线还没写完?)"
    exit 1
}
add_files -norecurse $DIR/vivado/hdl/icnd2049_panel_pov.v $DIR/vivado/hdl/ddr_slice_fetch.v
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins.xdc
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins_v5.xdc

create_bd_design system

# ---- PS7 (同 v4 + S_AXI_HP0 32-bit) ----
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

# ---- panel IP v5 (module reference; cell 名保持 panel_0 → OOC run 名 / tools 不变) ----
set panel [create_bd_cell -type module -reference icnd2049_panel_pov panel_0]

# 接口推断验证: 端口前缀不对时立刻报错, 不要等 validate 才炸
if {[llength [get_bd_intf_pins -quiet panel_0/s_axi]] == 0} {
    puts "FATAL: panel_0/s_axi 接口没推断出来 (RTL s_axi_* 前缀?)"; exit 1
}
if {[llength [get_bd_intf_pins -quiet panel_0/m_axi]] == 0} {
    puts "FATAL: panel_0/m_axi 接口没推断出来 (RTL m_axi_* 前缀 / X_INTERFACE_INFO?)"; exit 1
}
if {[llength [get_bd_pins -quiet panel_0/spin_sync]] == 0} {
    puts "FATAL: panel_0/spin_sync pin 不存在"; exit 1
}
# m_axi 时钟关联兜底 (RTL 里没写 X_INTERFACE_PARAMETER 时救一把; 写了则此行无害)
catch { set_property CONFIG.ASSOCIATED_BUSIF {s_axi:m_axi} [get_bd_pins panel_0/s_axi_aclk] }

# ---- AXI-Lite: GP0 → panel_0/s_axi (自动化生成互联 + proc_sys_reset, 时钟 FCLK0) ----
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config \
  [list Master {/ps7_0/M_AXI_GP0} Clk {Auto}] [get_bd_intf_pins panel_0/s_axi]

# ---- M_AXI: panel_0/m_axi → axi_smc → S_AXI_HP0 (手工连, 确定性优先) ----
# 选 SmartConnect 不选 axi_interconnect: 1:1 直通, 自动做 AXI4→AXI3(HP 口) 协议转换,
# 零配置; interconnect 只在需要手调仲裁/register slice 时才值得。
set smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_smc_hp0]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $smc

connect_bd_intf_net [get_bd_intf_pins panel_0/m_axi]    [get_bd_intf_pins axi_smc_hp0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_smc_hp0/M00_AXI] [get_bd_intf_pins ps7_0/S_AXI_HP0]

# 时钟/复位挂到 GP0 自动化已建好的 FCLK0 网络 + proc_sys_reset
set clk_net [get_bd_nets -of_objects [get_bd_pins ps7_0/FCLK_CLK0]]
connect_bd_net -net $clk_net [get_bd_pins axi_smc_hp0/aclk]
connect_bd_net -net $clk_net [get_bd_pins ps7_0/S_AXI_HP0_ACLK]
set rst_cell [lindex [get_bd_cells -filter {VLNV =~ "xilinx.com:ip:proc_sys_reset:*"}] 0]
set rst_net  [get_bd_nets -of_objects [get_bd_pins $rst_cell/peripheral_aresetn]]
connect_bd_net -net $rst_net [get_bd_pins axi_smc_hp0/aresetn]

# ---- 地址映射 ----
assign_bd_address

# AXI-Lite 段维持 0x40010000 / 64K (tools/panel_seq.h 工具链依赖此地址, 不许动)
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7_0/Data] -filter {NAME =~ *panel*}]
if {[llength $seg] == 1} {
    set_property offset 0x40010000 $seg
    set_property range  64K        $seg
} else {
    puts "FATAL: AXI-Lite panel 段找不到/不唯一: $seg"; exit 1
}

# M_AXI 看全 DDR: 0x00000000 / 512M (HP 口低 1MB 也直达 DDR, 无 GP 口 OCM 混叠问题)
set mseg [get_bd_addr_segs -quiet -of_objects [get_bd_addr_spaces panel_0/m_axi] -filter {NAME =~ *HP0*}]
if {[llength $mseg] == 0} {
    # assign_bd_address 没自动分到就手动指
    assign_bd_address -target_address_space [get_bd_addr_spaces panel_0/m_axi] \
        [get_bd_addr_segs ps7_0/S_AXI_HP0/HP0_DDR_LOWADDR]
    set mseg [get_bd_addr_segs -of_objects [get_bd_addr_spaces panel_0/m_axi] -filter {NAME =~ *HP0*}]
}
if {[llength $mseg] != 1} { puts "FATAL: m_axi HP0 DDR 段找不到/不唯一: $mseg"; exit 1 }
set_property offset 0x00000000 $mseg
set_property range  512M       $mseg
puts "ADDR_MAP m_axi -> [get_property NAME $mseg] @ [get_property OFFSET $mseg] range [get_property RANGE $mseg]"

# ---- 外部端口: 屏输出 (P1 + P3 双口同波形, 与 v4 相同) ----
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

# ---- 外部输入: spin_sync_i → panel_0/spin_sync ----
# [坑2 规避] 不用 make_bd_pins_external (鹿小班 sensor_pulse 曾被自动拴 const0),
# 而是显式建 port + connect_bd_net, validate 后还要再验一次 (见下)。
create_bd_port -dir I spin_sync_i
connect_bd_net [get_bd_ports spin_sync_i] [get_bd_pins panel_0/spin_sync]

validate_bd_design

# ---- [坑2] spin_sync const0 终检: validate 之后查 net 端点, 有 xlconstant 就杀掉重连 ----
proc verify_spin_sync {} {
    set pin [get_bd_pins panel_0/spin_sync]
    set net [get_bd_nets -quiet -of_objects $pin]
    if {$net eq ""} { puts "SPIN_SYNC_FAIL: panel_0/spin_sync 悬空"; exit 1 }
    # 网上任何 xlconstant → 删掉 (delete 会顺带断 net, 之后重连 port)
    foreach p [get_bd_pins -quiet -of_objects $net] {
        set c [get_bd_cells -quiet -of_objects $p]
        if {$c ne "" && [string match -nocase "*xlconstant*" [get_property VLNV $c]]} {
            puts "SPIN_SYNC_WARN: 发现 $c 拴住 spin_sync, 删除重连"
            delete_bd_objs $c
        }
    }
    # 确保 port 仍在 net 上 (const 删除可能连带断线)
    if {[get_bd_nets -quiet -of_objects [get_bd_pins panel_0/spin_sync]] eq ""} {
        connect_bd_net [get_bd_ports spin_sync_i] [get_bd_pins panel_0/spin_sync]
    }
    set net   [get_bd_nets -of_objects [get_bd_pins panel_0/spin_sync]]
    set ports [get_bd_ports -quiet -of_objects $net]
    set ok 0
    foreach po $ports { if {[string match "*spin_sync_i" [get_property PATH $po]]} { set ok 1 } }
    puts "SPIN_SYNC_NET: $net  端点 pins=[get_bd_pins -quiet -of_objects $net] ports=$ports"
    if {!$ok} { puts "SPIN_SYNC_FAIL: spin_sync_i port 不在 net 上"; exit 1 }
    puts "SPIN_SYNC_OK: spin_sync_i -> panel_0/spin_sync 直连, 无 xlconstant"
}
verify_spin_sync

# const 清理后再 validate 一次确认干净
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
