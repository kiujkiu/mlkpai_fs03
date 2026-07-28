# FS03 J12 -> mlp_panel_v1.0 -> panel_0.93cob_trans **v1.2** -> P0.9375 COB 屏
# 依据: 三块板原理图 PDF 网表逐 pin 抽取交叉核对 (2026-07-27, tools/sch_netlist.py)
#   FS03 侧      : ball <-> CEP2_kP/N <-> J12 pin  (= 2k+1 / 2k+2)   [未变]
#   接口板 v1.0  : J12 pin --10R--> P1 pin,  P1_pin = 30 - J12_pin    [未变]
#                                  P3 pin,  P3_pin = 50 - J12_pin    [未变]
#   转接板 v1.2  : P1 pin -> 屏 J1 pin        [**本次唯一变更**]
#
# ⚠ 变更性质: 屏侧 J1 的 19 个信号 pin 号与 v1.1 **完全一致**, 只有 P1 (30pin,
#   接米联派那一侧) 被重排 -> 表现为同一 FPGA ball 现在喂到屏上的是别的信号。
#   新映射是旧 19 个 ball 的**纯置换**: 不新增引脚 / 不换 bank / RTL 端口名不变,
#   所以除本文件外全设计零改动。
#   回滚 (插回 v1.1 转接板): 用 vivado/panel_pins_trans_v11.xdc 替换本文件。
#
# ✅ 副作用是好的: 旧版 DCLK/LAT/OE 落在 J12.22/21/20 三根相邻线上 (2026-07-16 LA
#   实测 CLK 2-4ns 振铃假沿 + 行边界像素错乱的根因)。v1.2 把 DCLK 挪到 J12.12,
#   两侧邻线 J12.11/13 = SPI_MOSI/MISO 均**未使用** -> 高速线天然被静默线隔离。
#   (J12.11 与 DCLK 同属 CEP2_5 差分对, 必要时可把 SPI_MOSI 驱成常低当回流地。)
#   LAT(J12.15)/OE(J12.14) 仍相邻, 但两者每行才翻一次, 保留下面的弱驱动整形即可。
#
# Bank13, FS03 跳帽 J1 = 1-2 (ADJ_BANK13 = VCC_3V3) — 用户已设
#
# ⚠ 本表由原理图推导, **尚未上板验证**。首次点亮请先跑单色 R/G/B 验九线映射。

# ---- 列驱 ICND2049/2047 (屏 1, 接口板 P1) ----
#                                          新 P1  (旧 ball)
set_property -dict {PACKAGE_PIN Y10  IOSTANDARD LVCMOS33} [get_ports panel_dclk]       ;# P1.18 (旧 AB4)
set_property -dict {PACKAGE_PIN AA9  IOSTANDARD LVCMOS33} [get_ports panel_lat]        ;# P1.15 (旧 AB5)
set_property -dict {PACKAGE_PIN AB9  IOSTANDARD LVCMOS33} [get_ports panel_oe]         ;# P1.16 线名 GCLK, 实为 OE (旧 Y5)
set_property -dict {PACKAGE_PIN AB5  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[0]}]   ;# R1  P1.9  (旧 AB12)
set_property -dict {PACKAGE_PIN W12  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[1]}]   ;# G1  P1.26 (旧 AB11)
set_property -dict {PACKAGE_PIN AA6  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[2]}]   ;# B1  P1.12 (旧 Y9)
set_property -dict {PACKAGE_PIN Y9   IOSTANDARD LVCMOS33} [get_ports {panel_sdi[3]}]   ;# R2  P1.23 (旧 Y11)
set_property -dict {PACKAGE_PIN Y6   IOSTANDARD LVCMOS33} [get_ports {panel_sdi[4]}]   ;# G2  P1.11 (旧 Y8)
set_property -dict {PACKAGE_PIN AB12 IOSTANDARD LVCMOS33} [get_ports {panel_sdi[5]}]   ;# B2  P1.24 (旧 Y10)
set_property -dict {PACKAGE_PIN AA8  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[6]}]   ;# R3  P1.14 (旧 AA6)
set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS33} [get_ports {panel_sdi[7]}]   ;# G3  P1.21 (旧 AB10)
set_property -dict {PACKAGE_PIN AA7  IOSTANDARD LVCMOS33} [get_ports {panel_sdi[8]}]   ;# B3  P1.13 (旧 Y6)

# ---- 行驱 ICND3019/1028 (A=DCLK, B=RCLK, C=SDI; 2026-05-27 老屏实测约定) ----
set_property -dict {PACKAGE_PIN AB4  IOSTANDARD LVCMOS33} [get_ports panel_row_dclk]   ;# A 屏 AIN P1.8  (旧 W12)
set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS33} [get_ports panel_row_rclk]   ;# B 屏 BIN P1.25 (旧 AA11)
set_property -dict {PACKAGE_PIN Y5   IOSTANDARD LVCMOS33} [get_ports panel_row_sdi]    ;# C 屏 CIN P1.10 (旧 AA12)

# ---- P3 口 (_2 组, 与 P1 同波形) ----
# ⚠ 前提: 屏2 也换成 v1.2 转接板。若屏2 仍插 v1.1 板, 把本段换成
#   panel_pins_trans_v11.xdc 里对应的 _2 段 (两块板可以混插, 各自独立)。
# P3 与 P1 结构同构 (P3_pin = 50 - J12_pin), 故套用同一置换。
set_property -dict {PACKAGE_PIN AB6  IOSTANDARD LVCMOS33} [get_ports panel_dclk_2]     ;# P3.18 (旧 W8)
set_property -dict {PACKAGE_PIN U6   IOSTANDARD LVCMOS33} [get_ports panel_lat_2]      ;# P3.15 (旧 V8)
set_property -dict {PACKAGE_PIN AB1  IOSTANDARD LVCMOS33} [get_ports panel_oe_2]       ;# P3.16 (旧 W7)
set_property -dict {PACKAGE_PIN V8   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[0]}] ;# R1_2 P3.9  (旧 AA4)
set_property -dict {PACKAGE_PIN W5   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[1]}] ;# G1_2 P3.26 (旧 U4)
set_property -dict {PACKAGE_PIN T6   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[2]}] ;# B1_2 P3.12 (旧 V5)
set_property -dict {PACKAGE_PIN V5   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[3]}] ;# R2_2 P3.23 (旧 AB7)
set_property -dict {PACKAGE_PIN V7   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[4]}] ;# G2_2 P3.11 (旧 V4)
set_property -dict {PACKAGE_PIN AA4  IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[5]}] ;# B2_2 P3.24 (旧 AB6)
set_property -dict {PACKAGE_PIN U5   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[6]}] ;# R3_2 P3.14 (旧 T6)
set_property -dict {PACKAGE_PIN T4   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[7]}] ;# G3_2 P3.21 (旧 AB2)
set_property -dict {PACKAGE_PIN R6   IOSTANDARD LVCMOS33} [get_ports {panel_sdi_2[8]}] ;# B3_2 P3.13 (旧 V7)
set_property -dict {PACKAGE_PIN W8   IOSTANDARD LVCMOS33} [get_ports panel_row_dclk_2] ;# A_2 P3.8  (旧 W5)
set_property -dict {PACKAGE_PIN Y4   IOSTANDARD LVCMOS33} [get_ports panel_row_rclk_2] ;# B_2 P3.25 (旧 T4)
set_property -dict {PACKAGE_PIN W7   IOSTANDARD LVCMOS33} [get_ports panel_row_sdi_2]  ;# C_2 P3.10 (旧 Y4)

# ---- 暂未用 ----
# 屏1 SPI Flash (新序): CS=Y8  MOSI=Y11 CLK=AB11 MISO=AB10   (P1.22/19/20/17)
# 屏2 SPI Flash (新序): CS=V4  MOSI=AB7 CLK=U4   MISO=AB2    (P3.22/19/20/17)
# 光电 SPIN_SYNC = W6 (接口板 P5.1 经 R5, CEP_11P = J12.23) — 不过转接板, 未变

# ---- SI 整形 (2026-07-16 LA 实测 CLK 2-4ns 振铃假沿, 边界像素错乱根因) ----
# 全部屏输出: 慢摆率 + 8mA — 钝化边沿抑制反射双击
set_property SLEW SLOW [get_ports panel_*]
set_property DRIVE 8    [get_ports panel_*]

# ---- 分层驱动 (2026-07-17 行边界串扰案) ----
# CLK = 受害线, 强驱动压住耦合噪声; LE/OE/行驱 = 攻击线, 弱驱动减 dV/dt
# v1.2 起 DCLK 已被未用线隔离, 本段可作为冗余保留; 若 SI 已干净可试着回 DRIVE 8。
set_property DRIVE 16 [get_ports {panel_dclk panel_dclk_2}]
set_property DRIVE 4  [get_ports {panel_lat panel_lat_2 panel_oe panel_oe_2}]
set_property DRIVE 4  [get_ports panel_row_*]
