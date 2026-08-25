# MLKPAI-FS03 + pov_dual_top (v6) — 双屏 ICND2047 双沿 POV 工程 (2026-07-15)
#
# 相对 v5 的差异 (其余照抄 create_panel_proj_v5.tcl, 含全部坑规避):
#   - module ref: icnd2049_panel_pov → pov_dual_top (全新模块名, 天然无缓存坑)
#   - 源码 4 文件: pov_dual_top + panel_engine_2047(适配壳) + icnd2047_panel_core
#     + row_drv_icnd1028  (stub 不入工程!)
#   - 外部口: v5 的 P1/P3 镜像 → A/B 独立 (端口名不变: 原名=A/P1, _2=B/P3,
#     panel_pins.xdc 零改动)
#
# [2026-08-24 feature/3bit-color] 加 PL lz4 解码引擎 (NENG 见下):
#   - RTL: vivado/hdl/lz4/{lz4_decode_core,lz4_axi_top}.v = dr1v90/lz4hw/rtl 原样复制
#          + lz4_engine_axi.v (本工程新写的适配壳, 理由见该文件头)
#   - NENG=0 一行切回加 lz4 之前的行为 (PS7 配置逐字不变)
#
# [2026-08-25] NENG 3 → **4** (上板实测 74.1 ms/帧 > 62 ms 圈周期 ⇒ 只能隔圈翻页,
#   7.3 fps; 4 引擎 55.5 ms < 62 ms ⇒ 每圈翻 ⇒ 16 fps, 完整推导见 NENG 处):
#   - HP 口只有 4 个且 HP0 被面板占死 ⇒ lz4_0 与 lz4_3 **共享 HP3**
#     (经一个 NUM_SI=2 的 SmartConnect), lz4_1→HP1, lz4_2→HP2。
#     选 HP3 共享是因为 DDRC 那一级 HP1/HP2 本来就共口, 放 HP3 才能 2:2 均衡 ——
#     完整取舍 (含否决掉的 S_AXI_GP / ACP 方案) 写在 HP_ORDER 处。
#   - AXI-Lite: panel 0x40010000, lz4_0..3 = 0x40020000/30000/40000/**50000**
#   - PS7 配置与 NENG=3 逐字相同 ⇒ ps7_init 不变 ⇒ fsbl.elf / BOOT.BIN 不用重做
#
#   - 跑法同 v5:
#     cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03 && \
#       call C:\Xilinx\Vivado\2024.2\settings64.bat && \
#       vivado -mode batch -notrace -source vivado\create_panel_proj_v6.tcl"
#
#   🔴 **-notrace 不是可选的**, 少了它日志会变成"grep 骗人"的日志:
#     Vivado 默认把 source 进来的每条命令回显进日志。花括号块(if/foreach/proc)
#     是**整块**回显的, 连块内的中文注释一起 —— 而 cmd.exe 按 GBK 写出去,
#     于是日志里混进非法 UTF-8 字节。GNU grep 一旦判定文件是**二进制**,
#     就不再报告匹配, 而且 **exit status 也是 1(没找到)**。后果:
#       grep -q BUILD_OK   -> exit 1  ⇒ "构建没成功"  (其实成功了)
#       grep -q "ERROR:"   -> exit 1  ⇒ "干净, 没报错" (其实构建失败了)
#     🔴 第二条是要命的那个方向: **失败的构建会被检查脚本判成通过**。
#     2026-08-24 实测 (同一份脚本, 只差 -notrace):
#       默认    : 日志 110 个非 ASCII 字节, grep -q MARK 的 exit = 1  ✗
#       -notrace: 日志 0 个非 ASCII 字节,   grep -q MARK 的 exit = 0  ✓
#     (同一天还因此让三个 `until grep -q BUILD_OK` 的等待循环永远转下去。)
#   ⚠ 已经生成的老日志救不回来, 读它们一律加 **grep -a**(--text)。

set PROJ  mlkpai_panel
set PART  xc7z020clg484-1
set DIR   [file normalize [file dirname [info script]]/..]
set BUILD $DIR/build_panel

# ---- PL lz4 解码引擎数 (2026-08-24; 2026-08-25 扩到 4) --------------------
# 0 = 完全不加 lz4, 逐位回到加 lz4 之前的 v6 (回归/对照用一行就能切)
# 1..3 = 每个引擎独占一个 HP **slave 口** (2026-08-24 上板版就是 NENG=3)
# 4    = 其中一对引擎**共享**一个 HP 口 (HP_ORDER 里出现重复项 = 共享)
#
# 🔴 为什么要第 4 个 (2026-08-25 上板实测):
#    3 引擎解一帧 74.1 ms (0.95 B/clk, 稳定), 而转速已提到 969 RPM = 16.15 rev/s
#    ⇒ 圈周期 62 ms。74 > 62 ⇒ 每帧解完时翻页窗口刚过去, 只能等下一圈,
#    实际周期 74+62 = 136 ms ⇒ 7.3 fps (实测 flip 2-7/s, 对得上)。
#    4 引擎 ⇒ 74 × 3/4 = 55.5 ms < 62 ms ⇒ 每圈都能翻 ⇒ 16 fps。
#    ⚠ 本文件末尾原来写着 "不要靠加第 4 个引擎, 去提频" —— 那条建议的前提是
#      "瓶颈是总带宽"。实测瓶颈不是带宽 (pair_miss 增长率 0.0/s, 零丢对),
#      而是**单帧墙钟 74 ms vs 圈周期 62 ms** 这条门限。提频和加引擎等价地
#      都能过线, 但提频要新开第二个 BUFG 时钟域, 对 WHS 只有 0.020 ns 的
#      本设计风险更大 ⇒ 这次选加引擎。提频路线仍留在文件末尾。
#
# 🔴 "上限 3" 的老理由 —— feedback_pov_4x_ip_breaks_hdmi: 当年 4 个 HLS IP × 2 master
#    = **NUM_SI=8 挤在一个 axi_smc 上打 HP1**, 单独跑每个都对, 一起跑 HDMI 变噪点,
#    至今没定位、没修复, 只能退回 1× IP。原脚本据此立了条不变式:
#    "每个引擎独占一个 HP 口 ⇒ SmartConnect 这一级零共享仲裁"。
#    ⇒ 本版**确实破了这条不变式**, 老实写在这里: 有一个口是 NUM_SI=2。
#      但离当年那个 8:1 差了两个数量级 (2 个主端、同一时钟域、同位宽、无跨域,
#      而当年是 8 个主端 + HLS 自己的多口), 且另外两个口仍是 1:1 直通。
#      共享口选在哪、共享谁, 下面写死了理由; 上板后要盯的第一个量仍是 pair_miss。
#
# 🔴 HP 口只有 4 个, HP0 被面板 (ddr_slice_fetch256, 110.8 MB/s **纯读**) 独占且不能动
#    ⇒ 4 个引擎必然有两个共享一个口。选哪个口由 **DDRC 那一级**决定, 不是 HP 那一级:
#    AMD 文档 (Embedded Design Tutorials, Evaluating High-Performance Ports):
#      "Throughput of HP1 and HP2 is lower than HP0 and HP3.
#       It is because **HP1 and HP2 shares one DDR input port**."
#      "read channels have higher priority than write channels when DDRC has congestions."
#    ⇒ DDRC 入口只有 3 个: {HP0}, {HP1,HP2}, {HP3}。4 引擎 ≈ 4 × 46.5 = 186 MB/s 写。
#      共享口放 HP3: DDRC 分布 = 面板 110.8(读) | {HP1,HP2} 两引擎 93 | {HP3} 两引擎 93
#                    ⇒ 解码侧的两个 DDRC 入口 **2:2 均衡**            ← 选这个
#      共享口放 HP1: DDRC 分布 = 面板 110.8(读) | {HP1,HP2} **三**引擎 139 | {HP3} 一引擎 46.5
#                    ⇒ 把 3/4 的写流量压在 AMD 明说"吞吐更低"的那个共享 DDRC 口上。差。
#    HP 口那一级不是瓶颈: 64 bit @ 50 MHz = 400 MB/s 理论, 共享口上两个引擎才 93 MB/s,
#    4.3× 余量 ⇒ 2:1 仲裁是**延迟**风险而不是带宽风险, 而解码器有 FIFO 扛延迟。
#    利好没变: 面板在 HP0 且纯读, 解码器绝大部分是写, DDRC 拥塞时读优先于写。
#
# 评估过但**否决**的其它拓扑 (留着免得下次再想一遍):
#   - S_AXI_GP0/GP1: 走中央互连, 是真正的**第 4 个独立 DDRC 入口**, 看着最诱人。
#     否决理由: 口宽只有 32 bit ⇒ 必须加位宽转换器 (又几百 LUT/FF 挂到 BUFG 网上,
#     而 WHS 就是被这根网的偏斜卡住的), 且 64→32 让**事务数翻倍** —— "小事务打 DDR
#     的真实效率" 正是本次集成最没底的一项。更要命的是中央互连与 CPU 访外设/访 DDR
#     同路, 会把风险引到一条从没测过的通路上, 而面板是硬实时的。
#   - S_AXI_ACP: 64 bit 且延迟低, 但写会在 L2 里分配/命中 ⇒ 面板经 HP0 直读 DDR
#     会读到**旧数据**。这是正确性问题不是性能问题, 除非改 RTL 里的 AxCACHE。否决。
#   - 把面板挪出 HP0 腾口给引擎: 面板是全系统唯一的硬实时门限, 不动。否决。
#
# HP_ORDER[i] = 第 i 个引擎挂哪个 HP 口。**允许重复**, 重复即共享 (N:1 汇聚)。
#   ⚠ NENG<=3 时取 [lrange $HP_ORDER 0 NENG-1] = {3 1 2}, 与 2026-08-24 上板版
#     **逐字相同、全 1:1**, 单元名也不变 ⇒ 一行改回 3 就是完整回退路径。
#   NENG=4 追加的 lz4_3 挂回 HP3 与 lz4_0 共享 ⇒ lz4_0..2 的口分配一个字没改,
#   AXI-Lite 也不动 (0x40020000/30000/40000), 新引擎在 **0x40050000**。
#   ⚠ PS7 侧配置与 NENG=3 **完全一致** (HP0..HP3 本来就全开、全 64 bit) ⇒
#     ps7_init 不变 ⇒ **fsbl.elf / BOOT.BIN 不用重做**, 换 PL bit 即可。
set NENG 4
set HP_ORDER {3 1 2 3}
set LZ4_CONV protocol_converter   ;# 独占口的 1:1 转换器 (理由见下面 LZ4_CONV 处)
# 共享口的 N:1 汇聚器: smartconnect | axi_interconnect
#   smartconnect      : 本工程验证过的 IP (1:1 时 1716 LUT/1646 FF, 2:1 更大)。默认。
#   axi_interconnect  : 配 STRATEGY=1 (minimize area = SASD 共享总线) 时 LUT 少得多,
#                       但本工程没验证过 ⇒ 只有 WHS 收不住时才换过去。
set LZ4_SHARE_CONV smartconnect
# --------------------------------------------------------------------------

file delete -force $BUILD
create_project $PROJ $BUILD -part $PART -force

foreach f {angle_tracker.v pov_dual_top.v panel_engine_2047.v icnd2047_panel_core.v row_drv_icnd1028.v} {
    if {![file exists $DIR/vivado/hdl/$f]} { puts "FATAL: vivado/hdl/$f 不存在"; exit 1 }
    add_files -norecurse $DIR/vivado/hdl/$f
}
if {$NENG > 0} {
    # lz4_engine_axi = 适配壳 (标准 AXI4-Lite + 补齐 AXI4 信号);
    # lz4_axi_top / lz4_decode_core = dr1v90/lz4hw/rtl 原样复制, **一行没改**
    foreach f {lz4/lz4_decode_core.v lz4/lz4_axi_top.v lz4/lz4_engine_axi.v} {
        if {![file exists $DIR/vivado/hdl/$f]} { puts "FATAL: vivado/hdl/$f 不存在"; exit 1 }
        add_files -norecurse $DIR/vivado/hdl/$f
    }
}
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins.xdc
add_files -fileset constrs_1 -norecurse $DIR/vivado/panel_pins_v5.xdc

create_bd_design system

# ---- PS7 ----
# ⚠ v5/v6 原注释是 "ps7_init 不变 → fsbl.elf 可复用"。**NENG 从 0 变到 >0 时这句是假的**:
#   置 PCW_USE_S_AXI_HP1/2/3 会改 ps7_init 里的 AFI 配置。那一次必须:
#   重导 XSA → 重建 FSBL → 核对 ELF 里 HP0 那 3 处 0xF8008000 mask-write 还在
#   (那是 HP0 的 32-bit AFI 配置, 丢了就宽度错) → 重打 BOOT.BIN → 冷启动。
#   NENG=0 时下面这段与改动前逐字相同, 老 fsbl.elf 仍可复用。
#   🔴 但 **NENG 3 → 4 不触发这件事**: 4 个引擎用的还是 HP1/HP2/HP3 那三个口
#   (第 4 个是与 lz4_0 共享 HP3, 共享发生在 **PL 侧**的汇聚器里, PS7 看不见),
#   下面这段 set_property 逐字不变 ⇒ ps7_init 不变 ⇒ 现役 fsbl.elf/BOOT.BIN 继续用。
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

# ---- lz4 引擎用的 HP 口, 全 64-bit (哪个引擎挂哪个口见 HP_ORDER) ----
# lsort -unique: HP_ORDER 允许重复(共享口), 同一个口只能 set_property 一次
if {$NENG > 0} {
    set hp_cfg {}
    foreach i [lsort -unique [lrange $HP_ORDER 0 [expr {$NENG - 1}]]] {
        lappend hp_cfg CONFIG.PCW_USE_S_AXI_HP$i {1}
        # 64-bit: 核每 8 字节发一笔单拍事务, 32-bit 会让事务数翻倍 —— 而
        # "小事务打 DDR 的真实效率" 正是本次集成最没底的一项, 不要主动加倍
        lappend hp_cfg CONFIG.PCW_S_AXI_HP${i}_DATA_WIDTH {64}
    }
    set_property -dict $hp_cfg $ps
}

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

# ---- PL lz4 解码引擎 ×NENG (2026-08-24; 2026-08-25 加共享 HP 口支持) ------
# 每个引擎:
#   s_axi (AXI4-Lite) → ps7_0_axi_periph (GP0), 窗口 0x40020000 + i*0x10000, 64K
#   m_axi (AXI4 64b)  → 该 HP 口的转换器/汇聚器 → ps7_0/S_AXI_HP[lindex $HP_ORDER $i]
# 时钟/复位一律用 FCLK_CLK0 (50 MHz) 与 panel 同域 —— **不新开时钟域**:
#   05_3bit_bcm.md §11 记着 hold 余量只有 0.036 ns、build 日志里 WHS 见过 +0.015,
#   而且 XDC 里一条 create_clock/set_output_delay 都没有 (ODDR→pad 从没被分析过)。
#   在这种状态下多开一个 BUFG 域纯属给自己找事。lz4 OOC Fmax 136.8 MHz,
#   跑 50 MHz 是 2.7× 余量, setup 侧完全不是问题。
for {set i 0} {$i < $NENG} {incr i} {
    set cel [create_bd_cell -type module -reference lz4_engine_axi lz4_$i]
    if {[llength [get_bd_intf_pins -quiet lz4_$i/s_axi]] == 0} {
        puts "FATAL: lz4_$i/s_axi 接口没推断出来"; exit 1 }
    if {[llength [get_bd_intf_pins -quiet lz4_$i/m_axi]] == 0} {
        puts "FATAL: lz4_$i/m_axi 接口没推断出来"; exit 1 }
    # 🔴 检查 m_axi 真的是 64 bit: feedback_vivado_bd_addr_width_cache 记过
    #    module_ref 的端口位宽被 IP 缓存写死、源码改了 BD 里还是旧宽度的坑
    #    (症状是 awaddr 高位被拴到 GND, 花一小时 8 次 build 才定位)。
    #    本脚本每次 file delete -force $BUILD 重建工程 + 模块名全新 ⇒ 理论上碰不到,
    #    但这条检查便宜, 留着。
    set dw [get_property CONFIG.DATA_WIDTH [get_bd_intf_pins lz4_$i/m_axi]]
    if {$dw ne "64"} { puts "FATAL: lz4_$i/m_axi DATA_WIDTH=$dw, 期望 64"; exit 1 }

    catch { set_property CONFIG.ASSOCIATED_BUSIF {s_axi:m_axi} \
                [get_bd_pins lz4_$i/s_axi_aclk] }
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config \
        [list Master {/ps7_0/M_AXI_GP0} Clk {Auto}] [get_bd_intf_pins lz4_$i/s_axi]
}

# ---- 按 HP 口分组: 同一个口上的引擎共用一个汇聚器 ----
# HP_ORDER 里出现一次 ⇒ 1:1 直通转换器 (单元名 axi_smc_lz4_$i, 与 2026-08-24 版一致);
# 出现 N>1 次      ⇒ 一个 NUM_SI=N 的汇聚器 (单元名 axi_smc_lz4_share_hp$hp)。
array unset HP_ENG
set hp_uniq {}
for {set i 0} {$i < $NENG} {incr i} {
    set hp [lindex $HP_ORDER $i]
    if {![info exists HP_ENG($hp)]} { set HP_ENG($hp) {} ; lappend hp_uniq $hp }
    lappend HP_ENG($hp) $i
}
foreach hp $hp_uniq {
    set engs [set HP_ENG($hp)]
    set n    [llength $engs]

    if {$n == 1} {
        # ---- 独占口: AXI4 → AXI3(HP 口) 协议转换 ----
        # 🔴 这里**不用 SmartConnect**, 用 axi_protocol_converter。理由是实测的:
        #    3 个 1:1 SmartConnect 每个 **1716 LUT / 1646 FF**, 三个加起来 5148 LUT ——
        #    比三个解码引擎本身(3×1187)还大, 而且那 4938 个 FF 全挂在 50 MHz 的
        #    BUFG 网上。本设计的 WHS 几乎完全由这根网的**时钟偏斜**决定
        #    (最差路径: 0 逻辑级 FDRE→SRLC32E, 数据延迟 0.475 ns, 偏斜 0.282 ns),
        #    fanout 越大偏斜越大 ⇒ 少挂几千个负载是直接的 hold 余量。
        #    实测对照 (同一份 RTL, 只差这一项): smartconnect 12432 LUT / WHS 0.010,
        #    protocol_converter 8010 LUT / WHS **0.020** —— hold 余量直接翻倍。
        #    SmartConnect 的仲裁/位宽转换独占口一样都不需要 (1:1, 同位宽, 同时钟),
        #    它唯一干的活就是 AXI4→AXI3, 那正是 protocol_converter 的本职。
        #    ⚠ 想退回 SmartConnect 就把上面 LZ4_CONV 改成 smartconnect (两条路都留着)。
        set i     [lindex $engs 0]
        set cname axi_smc_lz4_$i
        if {$LZ4_CONV eq "smartconnect"} {
            set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $cname]
            set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc
            set silist S00_AXI; set mi M00_AXI
            connect_bd_net -net $rst_net [get_bd_pins $cname/aresetn]
            connect_bd_net -net $clk_net [get_bd_pins $cname/aclk]
        } else {
            set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_protocol_converter:2.1 $cname]
            set_property -dict [list CONFIG.SI_PROTOCOL {AXI4} CONFIG.MI_PROTOCOL {AXI3} \
                                     CONFIG.TRANSLATION_MODE {2}] $sc
            set silist S_AXI; set mi M_AXI
            connect_bd_net -net $rst_net [get_bd_pins $cname/aresetn]
            connect_bd_net -net $clk_net [get_bd_pins $cname/aclk]
        }
    } else {
        # ---- 共享口: N:1 汇聚 (本版只会走到 N=2, HP3) ----
        # 🔴 protocol_converter 干不了这活 —— 它是 1:1 的, 没有仲裁器。共享口必须用
        #    带 crossbar 的 IP。两条路都实现了, 由 LZ4_SHARE_CONV 选:
        #    smartconnect     : 本工程唯一验证过的互连 IP。已知代价是 FF 多、BUFG
        #                       fanout 涨 ⇒ 吃 WHS。但 2026-08-24 那次全 smartconnect
        #                       的构建 (12432 LUT / 11842 FF / fanout 14423) WHS 仍有
        #                       +0.010 ⇒ 只多加**一个** 2:1 的量级是能兜住的。
        #    axi_interconnect : STRATEGY=1 = minimize area = SASD 共享总线, 面积小得多,
        #                       代价是读写共用一条地址通路、串行化。本工程 93 MB/s 打
        #                       400 MB/s 的口, 串行化不是问题。没验证过, 作后备。
        set cname axi_smc_lz4_share_hp$hp
        if {$LZ4_SHARE_CONV eq "axi_interconnect"} {
            set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 $cname]
            set_property -dict [list CONFIG.NUM_SI $n CONFIG.NUM_MI {1} \
                                     CONFIG.STRATEGY {1}] $sc
            connect_bd_net -net $clk_net [get_bd_pins $cname/ACLK]
            connect_bd_net -net $rst_net [get_bd_pins $cname/ARESETN]
            connect_bd_net -net $clk_net [get_bd_pins $cname/M00_ACLK]
            connect_bd_net -net $rst_net [get_bd_pins $cname/M00_ARESETN]
            set silist {}
            for {set k 0} {$k < $n} {incr k} {
                set sn [format S%02d $k]
                connect_bd_net -net $clk_net [get_bd_pins $cname/${sn}_ACLK]
                connect_bd_net -net $rst_net [get_bd_pins $cname/${sn}_ARESETN]
                lappend silist ${sn}_AXI
            }
            set mi M00_AXI
        } else {
            set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $cname]
            set_property -dict [list CONFIG.NUM_SI $n CONFIG.NUM_MI {1}] $sc
            connect_bd_net -net $rst_net [get_bd_pins $cname/aresetn]
            connect_bd_net -net $clk_net [get_bd_pins $cname/aclk]
            set silist {}
            for {set k 0} {$k < $n} {incr k} { lappend silist [format S%02d_AXI $k] }
            set mi M00_AXI
        }
    }

    # 引擎 m_axi → 汇聚器各 SI (顺序 = HP_ORDER 里出现的顺序)
    if {[llength $silist] != $n} {
        puts "FATAL: HP$hp 汇聚器 SI 数 [llength $silist] != 引擎数 $n"; exit 1 }
    foreach i $engs si $silist {
        connect_bd_intf_net [get_bd_intf_pins lz4_$i/m_axi] \
                            [get_bd_intf_pins $cname/$si]
    }
    connect_bd_intf_net [get_bd_intf_pins $cname/$mi] \
                        [get_bd_intf_pins ps7_0/S_AXI_HP$hp]
    connect_bd_net -net $clk_net [get_bd_pins ps7_0/S_AXI_HP${hp}_ACLK]
    puts "LZ4: lz4_{$engs} -> $cname (NUM_SI=$n) -> S_AXI_HP$hp"
}

assign_bd_address

# ---- AXI-Lite 地址: 先全部搬到临时区, 再逐个落到最终地址 ----
# 🔴 为什么要两步 (2026-08-24 实测踩过): assign_bd_address 的自动落点**不可预测** ——
#    加了 lz4 之后它把 lz4_0 放在了 0x40010000, 于是 `set_property offset 0x40010000`
#    给 panel 时直接 ERROR: "overlaps with slave segment /lz4_0/s_axi/reg0"。
#    只要有两个段要互换位置, 单步 set_property 就一定会在中途撞车。
#    临时区选 0x70000000 起 (仍在 M_AXI_GP0 的 0x40000000-0x7FFFFFFF 内, 且与
#    最终的 0x4001_0000..0x4005_0000 完全不相交) ⇒ 两个阶段都不可能重叠。
set lite_segs {}
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7_0/Data] -filter {NAME =~ *panel*}]
if {[llength $seg] != 1} { puts "FATAL: AXI-Lite panel 段找不到/不唯一: $seg"; exit 1 }
lappend lite_segs $seg 0x40010000
for {set i 0} {$i < $NENG} {incr i} {
    set ls [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7_0/Data] \
                -filter "NAME =~ *lz4_$i*"]
    if {[llength $ls] != 1} { puts "FATAL: lz4_$i AXI-Lite 段找不到/不唯一: $ls"; exit 1 }
    lappend lite_segs $ls [format 0x%08X [expr {0x40020000 + $i * 0x10000}]]
}
set tmp 0x70000000
foreach {s off} $lite_segs {
    set_property offset [format 0x%08X $tmp] $s
    set_property range  64K $s
    set tmp [expr {$tmp + 0x10000}]
}
foreach {s off} $lite_segs {
    set_property offset $off $s
    puts "ADDR_MAP lite [get_property NAME $s] @ [get_property OFFSET $s] range [get_property RANGE $s]"
}

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

# ---- lz4 引擎 m_axi 地址 (AXI-Lite 已在上面两步搬好) ----
# AXI-Lite: 0x40020000 / 0x40030000 / 0x40040000 (panel 在 0x40010000, 不动)
#   ⚠ lz4_axi_top 只译码 awaddr[7:0] ⇒ 64K 窗内每 256 B 一个镜像, 软件只用窗首 0x00..0x18
# m_axi:    整个 512M 低端 DDR (帧区 0x10000000..0x1FFFFFFF 在内)
for {set i 0} {$i < $NENG} {incr i} {
    set hp   [lindex $HP_ORDER $i]
    set ms [get_bd_addr_segs -quiet -of_objects [get_bd_addr_spaces lz4_$i/m_axi] \
                -filter "NAME =~ *HP$hp*"]
    if {[llength $ms] == 0} {
        assign_bd_address -target_address_space [get_bd_addr_spaces lz4_$i/m_axi] \
            [get_bd_addr_segs ps7_0/S_AXI_HP$hp/HP${hp}_DDR_LOWADDR]
        set ms [get_bd_addr_segs -of_objects [get_bd_addr_spaces lz4_$i/m_axi] \
                    -filter "NAME =~ *HP$hp*"]
    }
    if {[llength $ms] != 1} { puts "FATAL: lz4_$i m_axi HP$hp DDR 段找不到/不唯一: $ms"; exit 1 }
    set_property offset 0x00000000 $ms
    set_property range  512M       $ms
    puts "ADDR_MAP lz4_$i m_axi -> [get_property NAME $ms] @ [get_property OFFSET $ms] range [get_property RANGE $ms]"
}

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
# 🔴 WHS 必须一起报: 本工程 hold 余量历来极薄 (05_3bit_bcm.md §11 记 0.036 ns,
#    build_v6 日志里见过 +0.015)。只看 WNS 会漏掉真正会翻车的那一侧。
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "TIMING_WNS: $wns"
puts "TIMING_WHS: $whs"
if {$whs < 0} {
    puts "BUILD_FAILED: TIMING_HOLD_FAIL WHS=$whs (<0)"
    puts "  hold 是本设计的紧项 (加 lz4 前 WHS 只有 0.033 ns, 9 条路径 <0.1 ns,"
    puts "  全是 0 逻辑级 FDRE->RAMD32, slack 几乎全来自 BUFG 网上的时钟偏斜)。"
    puts "  一次 hold 违例绝不能被 BUILD_OK 盖过去 —— 这个 bit 不要上板。"
    exit 1
}
report_utilization -file $BUILD/util_impl.rpt
report_timing_summary -file $BUILD/timing_impl.rpt
foreach {rn re} {LUT {PRIMITIVE_GROUP == LUT} FF {PRIMITIVE_GROUP == FLOP_LATCH} \
                 RAMB36 {REF_NAME =~ RAMB36*} RAMB18 {REF_NAME =~ RAMB18*}} {
    puts "UTIL_$rn: [llength [get_cells -hier -filter $re]]"
}
write_hw_platform -fixed -include_bit -force $DIR/mlkpai_panel.xsa
puts "BUILD_OK: $BUILD/$PROJ.runs/impl_1/system_wrapper.bit"

#=============================================================================
# 2026-08-25 更新: 下面这条 "不要靠加第 4 个引擎" 的旧建议**已被实测推翻一半**。
#   旧建议的前提是 "瓶颈是总带宽" ⇒ 提频比加引擎划算。
#   实测的瓶颈不是带宽 (3 引擎 + 面板, pair_miss 增长率 0.0/s, 零丢对),
#   而是**单帧墙钟 74.1 ms > 圈周期 62 ms (969 RPM)** 这条门限: 解完时翻页窗口
#   刚过, 只能等下一圈 ⇒ 实际 74+62=136 ms ⇒ 7.3 fps。
#   过这条门限只需要把单帧墙钟压到 62 ms 以下, **加引擎和提频等价**:
#     4 引擎 @50 MHz ⇒ 74×3/4 = 55.5 ms ✓   (本版走这条: 不新开时钟域)
#     3 引擎 @75 MHz ⇒ 74×2/3 = 49.4 ms ✓   (要新开 BUFG 域, 对 WHS=0.020 的
#                                            本设计风险更大, 见下)
#   两条路不互斥, 真需要 30 fps 时可以叠 (4 引擎 @75 MHz ⇒ 37 ms)。
#
# 提频路线 (仍然有效, 作为下一步)
#   3 引擎 @50 MHz  = 139–144 MB/s   (需求 116.2 MB/s, 余量 1.20×)
#   3 引擎 @75 MHz  = 209–216 MB/s   (余量 1.8×)
#   3 引擎 @100 MHz = 279–288 MB/s   (余量 2.4×)
# 做法: PS7 加 CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {75} + PCW_EN_CLK1_PORT,
#   lz4_* 与 axi_smc_lz4_* 与 S_AXI_HP{1,2,3}_ACLK 全部改接 FCLK_CLK1,
#   再给 lz4 域配一个 proc_sys_reset。AXI-Lite 的跨域由 ps7_0_axi_periph 自动
#   插时钟转换器 (apply_bd_automation 的 Clk 参数给 FCLK_CLK1 即可)。
#   🔴 **不要**顺手把 FCLK_CLK0 从 50 提到 100 —— 那是 panel 域, 05_3bit_bcm.md §11
#      算过最差 setup data path 11.987 ns ⇒ fmax ≈ 76 MHz, 提了必挂, 是另一件事。
#   🔴 加第二个 BUFG 域后 WHS 必须重新盯 (上面已经打印)。
#=============================================================================
