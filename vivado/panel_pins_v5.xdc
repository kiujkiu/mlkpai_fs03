# v5 增量约束: spin_sync 光电输入 (2026-07-08)
# 单独成文件的理由: v4 工程 (create_panel_proj.tcl) 的 BD 没有 spin_sync_i port,
# 若写进共享 panel_pins.xdc, v4 build 会对空 get_ports 报 critical warning;
# 屏引脚主表继续以 panel_pins.xdc 为单一权威, v5 只叠加本文件。
#
# 来源: docs/led_panel_chain.md 权威表 — SPIN_SYNC = 光电 (接口板 P5.1 经 R5 10R) → FPGA W6 (CEP_11P)
# Bank13, FS03 跳帽 J1 = 1-2 (ADJ_BANK13 = VCC_3V3) → LVCMOS33, 与 panel_pins.xdc 现有一致

set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports spin_sync_i]

# 异步传感器脉冲, IP 内部需两级同步; 不做 IO 时序约束
set_false_path -from [get_ports spin_sync_i]
