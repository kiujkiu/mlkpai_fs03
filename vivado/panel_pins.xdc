# FS03 J12 -> mlp_panel_v1.0 -> panel_0.93cob_trans -> P0.9375 COB 屏 (ICND2049 + ICND3019)
# 依据 docs/led_panel_chain.md 端到端映射 (2026-07-07)
# Bank13, FS03 跳帽 J1 = 1-2 (ADJ_BANK13 = VCC_3V3) — 用户已设

# ---- 列驱 ICND2049 (屏 1, 接口板 P1) ----
set_property -dict {PACKAGE_PIN AB4  IOSTANDARD LVCMOS33} [get_ports panel_dclk]
set_property -dict {PACKAGE_PIN AB5  IOSTANDARD LVCMOS33} [get_ports panel_lat]
set_property -dict {PACKAGE_PIN Y5   IOSTANDARD LVCMOS33} [get_ports panel_oe]        ;# 线名 GCLK, 实为 2049 OE
set_property -dict {PACKAGE_PIN AB12 IOSTANDARD LVCMOS33} [get_ports {panel_sdi[0]}]  ;# R1
set_property -dict {PACKAGE_PIN AB11 IOSTANDARD LVCMOS33} [get_ports {panel_sdi[1]}]  ;# G1
set_property -dict {PACKAGE_PIN Y9   IOSTANDARD LVCMOS33} [get_ports {panel_sdi[2]}]  ;# B1
set_property -dict {PACKAGE_PIN Y11  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[3]}]  ;# R2
set_property -dict {PACKAGE_PIN Y8   IOSTANDARD LVCMOS33} [get_ports {panel_sdi[4]}]  ;# G2
set_property -dict {PACKAGE_PIN Y10  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[5]}]  ;# B2
set_property -dict {PACKAGE_PIN AA6  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[6]}]  ;# R3
set_property -dict {PACKAGE_PIN AB10 IOSTANDARD LVCMOS33} [get_ports {panel_sdi[7]}]  ;# G3
set_property -dict {PACKAGE_PIN Y6   IOSTANDARD LVCMOS33} [get_ports {panel_sdi[8]}]  ;# B3

# ---- 行驱 ICND3019 (A=DCLK, B=RCLK, C=SDI; 2026-05-27 老屏实测约定) ----
set_property -dict {PACKAGE_PIN W12  IOSTANDARD LVCMOS33} [get_ports panel_row_dclk]  ;# 屏 AIN
set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS33} [get_ports panel_row_rclk]  ;# 屏 BIN
set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS33} [get_ports panel_row_sdi]   ;# 屏 CIN

# ---- P3 口 (_2 组, 与 P1 同波形) ----
set_property -dict {PACKAGE_PIN W8   IOSTANDARD LVCMOS33} [get_ports panel_dclk_2]
set_property -dict {PACKAGE_PIN V8   IOSTANDARD LVCMOS33} [get_ports panel_lat_2]
set_property -dict {PACKAGE_PIN W7   IOSTANDARD LVCMOS33} [get_ports panel_oe_2]
set_property -dict {PACKAGE_PIN AA4  IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[0]}]  ;# R1_2
set_property -dict {PACKAGE_PIN U4   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[1]}]  ;# G1_2
set_property -dict {PACKAGE_PIN V5   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[2]}]  ;# B1_2
set_property -dict {PACKAGE_PIN AB7  IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[3]}]  ;# R2_2
set_property -dict {PACKAGE_PIN V4   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[4]}]  ;# G2_2
set_property -dict {PACKAGE_PIN AB6  IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[5]}]  ;# B2_2
set_property -dict {PACKAGE_PIN T6   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[6]}]  ;# R3_2
set_property -dict {PACKAGE_PIN AB2  IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[7]}]  ;# G3_2
set_property -dict {PACKAGE_PIN V7   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[8]}]  ;# B3_2
set_property -dict {PACKAGE_PIN W5   IOSTANDARD LVCMOS33} [get_ports panel_row_dclk_2]  ;# A_2
set_property -dict {PACKAGE_PIN T4   IOSTANDARD LVCMOS33} [get_ports panel_row_rclk_2]  ;# B_2
set_property -dict {PACKAGE_PIN Y4   IOSTANDARD LVCMOS33} [get_ports panel_row_sdi_2]   ;# C_2

# ---- 暂未用 ----
# 屏1 SPI Flash: CS=AB9 MOSI=AA9 CLK=AA8 MISO=AA7 / 屏2 SPI: CS=AB1 MOSI=U6 CLK=U5 MISO=R6
# 光电 SPIN_SYNC = W6 (接口板 P5, CEP_11P)
