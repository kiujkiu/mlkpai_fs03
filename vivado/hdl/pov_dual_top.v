//==============================================================================
// pov_dual_top.v — 双屏 POV 顶层 (背靠背 180°, ICND2047 引擎黑盒版)
//
// 依据: docs/design_icnd2047/02_dual_panel_arch.md + 00_overview.md 裁决节.
// 基线: vivado/hdl/icnd2049_panel_pov.v (v5, 原文件零改动) —
//   angle_tracker 直接实例化 v5 文件里的模块 (零改动);
//   ddr_slice_fetch 按 02 文档改 burst 16→256 → 本文件内新模块
//   ddr_slice_fetch256 (v5 的 ddr_slice_fetch.v 不动), 另加 1-bit target 透传.
//
// 结构 (02 §1.3 方案 A):
//   angle_tracker ─ slice_idx ─→ pair 触发器 (idx 变化拍快照 slice_base/idx_B)
//        └→ fetch_go(A, idx) → ddr_slice_fetch256 → 引擎 A fb 写口
//           A done → fetch_go(B, (idx+PHASE_B) mod n_slices) → 引擎 B fb 写口
//   panel_engine ×2 黑盒实例 (接口 = v5 引擎 fb 写口 + 控制口语义; 真引擎
//   icnd2047_panel_core 另一 agent 在做, xsim 用 panel_engine_stub.v 占位).
//
// 寄存器 (awaddr[5:2] 解码; 0x0C/0x10/0x14/0x18 v5 原样兼容):
//   W 0x0C 杂项: v5 原样 (subcmd 00=sdi_mask / 10=oe+rows+cfg_we{dclk_fast,
//         overlap_en,oe_window} / 11=auto_en,use_fb,pattern,disp 窗).
//         subcmd 01 (2026-08-20 起) = 3-bit 行内 BCM 配置:
//           [7:0]=oe_w1 (plane1 沿数, 0=不变) [15:8]=oe_w2 (plane2, 0=不变)
//           [16]=bpp_mode 0=1-bit(旧行为) 1=3-bit BCM
//           [17]=le_plane_mode plane1/2 的 LE 沿数: 0=3 沿(datasheet 默认)
//                1=与 plane0 同 4/5 沿 (上板逃生门, 免一次重综合)
//           oe_w0 = 复用 subcmd10 的 oe_window[15:8]; 默认 27/54/108 沿.
//           (原先规划给 per-chain 预载的编码作废, 见 05_3bit_bcm.md)
//   W 0x10 POV_CTRL: [0]=pov_en [1]=fake_en [31:16]=n_slices (v5 原样);
//         新 [2]=dual_en (0=纯单屏, B 引擎静默消隐, 行为=v5)
//         新 [3]=fb_sel_b (AXI fb 窗 awaddr[15]=1 的写落 fb_B, pov_en=0 调试直灌;
//              置位时 B 引擎也使能 — 02 §7 阶段1 "dual_en=0 直灌 fb_B + auto 扫" 用)
//         新 [4]=mirror_b / [5]=mirror_a (各屏写 fb 时做左右镜像, 见下方 MIRROR 段;
//              复位=0 保持旧行为。两位都置 1 = 全局镜像, 可直接复用未加镜像渲染的旧内容)
//         新 [6]=fold_a_en (面 A 半圈折叠, 机械 v3.1; 复位=0 保持旧行为, 见下方 FOLD 段)
//   W 0x14 fake_period / W 0x18 slice_base (v5 原样; 翻页原子性由 pair 快照兜底)
//   W 0x1C PHASE_B: [8:0] 屏 B slice 偏移, 复位默认 180 (要求 < n_slices)
//         ⚠ 机械 v3.1 (两面各一份独立切片数据, 见下方 "机械 v3.1" 段) 下软件应把
//           本寄存器写 0: 屏 B 有自己那份数据 (0x28), 该取的是同一个 idx 而非 idx+180。
//           复位默认仍是 180 — 那是老的"共用一份对称数据"路径, 不改以保回归。
//   W 0x20 BRIGHT_B: [7:0] oe_window_B, 0=跟随屏 A oe_window (复位默认 0)
//   W 0x24 ROW_CFG: [31:0] 行驱时序/极性 (透传两引擎)
//   W 0x28 SLICE_BASE_B: [31:0] 屏 B 独立切片数据基址 (机械 v3.1 新增)
//         = 0 (复位默认) → 屏 B 回落用 0x18 的 slice_base, 即老的"两面共用一份数据"
//           行为, 逐位不变; 老软件不写 0x28 完全不受影响。
//         != 0 → 屏 B 的 fetch 用这份基址, 与屏 A 的 0x18 各走各的。
//           快照与 0x18 严格同拍 (见 pair 级翻页原子性), 两面保证同帧, 不会撕裂。
//   R 0x00 status: v5 原位 [0]=engineA_busy [3]=oe_A [4]=auto_en [5]=use_fb
//         [6]=overlap_en [7]=dclk_fast [8]=locked [9]=pov_en [10]=fetch/pair busy
//         + 新 [11]=dual_en [12]=engine_B_busy [13]=mirror_b [14]=mirror_a ([1]/[2] cmd_pending/icnd_busy
//         归引擎黑盒, 恒 0; 02 §3 表把 [11]/[12] 写在 "0x10 R" 是地址笔误 —
//         0x10 读保持 v5 {locked,idx} 布局不破坏老脚本, 状态位落 0x00)
//         + 新 [15]=fold_a_en (回读 0x10[6], 供软件确认写进去了)
//         + 新 [16]=base_b_act (slice_base_b != 0, 即屏 B 走独立数据; 供软件确认 0x28 生效)
//         [15]/[16] 复位默认都是 0 → 复位态回读值与改动前逐位一致。
//   R 0x10 {locked,15'b0,slice_idx} / R 0x14 rev_period /
//   R 0x18 {locked_ever,15'b0,slice_max}  (v5 原样)
//   R 0x24 POV_CTRL 影子: 最近一次写进 0x10 的控制字, 位序与写口**逐位对齐**
//         ([0]=pov_en [1]=fake_en [2]=dual_en [3]=fb_sel_b [4]=mirror_b
//          [5]=mirror_a [6]=fold_a_en [15:7]=0 [31:16]=n_slices)
//         为什么要有它: 0x10 是**只写**的 (读 0x10 是 v5 的 {locked,slice_idx},
//         老脚本依赖, 不能动)。板端守护进程要按帧开关 0x10[6] fold_a_en, 若从自
//         己维护的陈旧影子做 read-modify-write, 会误伤 pov_en/fake_en — 尤其当
//         JTAG 调试脚本也在并发写 0x10 时。有了 0x24 读口, 任何软件都能
//         "读 0x24 → 改位 → 写 0x10" 安全 RMW, 不必维护影子也不怕抢写。
//         (写 0x24 仍是 row_cfg_r, 读写不同义, 有意为之: 读口地址已经很紧张)
//   R 0x28 FRAME_PERIOD: [31:0] 引擎 A 最近**一整屏**的 aclk 拍数 (2026-08-20 接通,
//         这个地址当初就是给它留的)。1-bit = rows×195 (54 行 ⇒ 10530);
//         3-bit = rows×3×195 (54 行 ⇒ 31590); oe>111 的 LWAIT 和 q_gap 死区都计入。
//         ⚠ 写 0x28 仍是 SLICE_BASE_B, 读写不同义 (与 0x24 同套路)。
//         用途 (aclk 悬案): 本值是**纯 PL 侧计数**, 不含任何频率假设; 用已验证
//         正确的 CPU 时基测同一整屏的墙钟秒数, 两者相除 = aclk 真实频率。
//         BD/SLCR/约束 五条证据说 50MHz、实测行周期 7.805µs 反推 25MHz —— 这
//         一个寄存器就能判谁对, 不用示波器。给的是引擎 A 的; 引擎 B 扫描节拍
//         与 A 同 (同时钟/同 rows/同 row_cfg), 只有 oe_window_b>111 时才会因
//         LWAIT 不同而分叉, 那种配置下这个读数只代表 A。
//   R 0x1C {locked, 6'b0, idx_B[8:0], pair_miss[15:0]} — idx_B 实时算 (核相位),
//         pair_miss = pair_busy 期间 slice_idx 变化 (整对丢弃) 的饱和计数
//
// pair 级翻页原子性 (02 §2.2): idx 变化那拍把 slice_base_r 快照进 base_lat,
//   A/B 两次 fetch 都用 base_lat → pov_rxd 写 0x18 的时机完全不用管;
//   pair 期间新 idx 整对丢弃 (丢帧跳最新, v5 同语义), 绝不只丢 B 不丢 A.
//   机械 v3.1 起 slice_base_b_r 在**同一拍**快照进 base_lat_b (同一个 if 分支,
//   同一个 always, 同一个时钟沿) → 两面数据必然来自同一帧, 不会一面新一面旧。
//
// 机械 v3.1 偏心改版 (两面不再对称) —— 本文件相关的两处改动:
//   背景: 屏整体偏心 6.7mm 后, 两屏面到转轴的垂距变成 0mm(面 A, 穿心) 和
//   13.4mm(面 B)。原来两屏关于转轴对称, "屏B@θ ≡ 屏A@(θ+180)" 严格成立, 于是
//   两个引擎共用同一个 slice_base, 屏 B 只把 slice 索引 +PHASE_B(180) 即可。
//   偏心后该等价关系作废 ⇒ 两面必须各用一份独立的切片数据。
//   (a) 0x28 slice_base_b: 屏 B 的独立数据基址 (=0 回落老行为)。配套软件应把
//       0x1C PHASE_B 写 0 — 屏 B 自己那份数据里, idx 就是 idx。
//   (b) 0x10[6] fold_a_en: 面 A 穿心 ⇒ slice_i ≡ mirror(slice_{i+180}), 所以面 A
//       只需在 DDR 里存前半圈 (n_slices/2 片), 后半圈由 PL 取 idx-n_slices/2 再做
//       镜像置换补齐。见下方 FOLD 段。
//       省的是**面 A 的 DDR 占用**(半圈) 和上位机的渲染量, **不是**读带宽 —
//       每个 slice_idx 照样整帧 fetch 一次 (TOTAL_WORDS 不变), 只是后半圈重复读
//       前半圈那块地址 (反而更 cache/row-buffer 友好)。带宽账仍按 02 §1.1 算。
//   两个新特性复位默认都是"关", 不写新寄存器的老软件行为逐位不变。
//
// 向后兼容: dual_en=0 (复位默认) 时 pair FSM 只发 A, B 引擎 enable=0 消隐,
//   寄存器 0x0C-0x18 语义 = v5 → 行为回归单屏.
//
// 遗留 (v6 集成项):
//   * 0x00/0x04/0x08 手动命令 + 0x0C subcmd01 per-chain 预载: 属列驱引擎内部
//     sequencer, 等 icnd2047_panel_core 定稿后按其接口接入 (POV 路径不依赖).
//   * panel_engine 端口表是与引擎 agent 的接口契约, 真引擎按此口子替换 stub.
//   * BD module_ref 加端口有缓存坑, 按 feedback_vivado_bd_module_ref_update 配方.
//==============================================================================
`timescale 1ns / 1ps

module pov_dual_top #(
    parameter DCLK_DIV = 4    // 50 MHz / 4 = 12.5 MHz DCLK (透传引擎)
)(
    // AXI-Lite slave
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axi" *)
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    input  wire [15:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg [1:0]   s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [15:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg [31:0]  s_axi_rdata,
    output reg [1:0]   s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // 屏 A (P1 引脚组)
    output wire        dclk_out,
    output wire        le_out,
    output wire        oe_out,
    output wire [8:0]  sdi_out,
    output wire        icnd_sdi_out,
    output wire        icnd_dclk_out,
    output wire        icnd_rclk_out,

    // 屏 B (P3 引脚组, XDC *_2 已备)
    output wire        dclk_out_2,
    output wire        le_out_2,
    output wire        oe_out_2,
    output wire [8:0]  sdi_out_2,
    output wire        icnd_sdi_out_2,
    output wire        icnd_dclk_out_2,
    output wire        icnd_rclk_out_2,

    // 光电, 1 脉冲/圈 (单传感器 + idx 偏移, 02 §5)
    input  wire        spin_sync,

    // AXI4 读主 (32bit, 经 BD smc → PS7 HP0 → DDR)
    output wire [31:0] m_axi_araddr,
    output wire [7:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output wire        m_axi_arlock,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready
);

    //---------- 寄存器 ----------
    reg  [8:0]  sdi_mask;
    reg         oe_set_pulse, oe_set_val;
    reg         fb_we;
    reg  [3:0]  fb_wlane;
    reg  [8:0]  fb_waddr;
    reg  [31:0] fb_wdata;
    reg         fb_wtgt;         // AXI fb 窗写目标 (写时快照 fb_sel_b)
    reg         use_fb, auto_en;
    reg  [15:0] auto_pattern;
    reg  [19:0] auto_disp_cyc;
    reg  [8:0]  au_rows_max;
    reg         dclk_fast, overlap_en;
    reg  [7:0]  oe_window;       // 屏 A 亮度 (0x0C subcmd10, 老脚本不动)
    reg         pov_en, fake_en_r;
    reg         dual_en;         // 0x10[2]
    reg         fb_sel_b;        // 0x10[3]
    reg         mirror_b;        // 0x10[4] 屏 B 左右镜像 (2026-07-28)
    reg         mirror_a;        // 0x10[5] 屏 A 左右镜像 (2026-07-28)
    reg         fold_a_en;       // 0x10[6] 面 A 半圈折叠 (机械 v3.1)
    reg  [15:0] n_slices_r;
    reg  [31:0] fake_period_r;
    reg  [31:0] slice_base_r;
    reg  [8:0]  phase_b_r;       // 0x1C, 默认 180
    reg  [7:0]  oe_window_b_r;   // 0x20, 0 = 跟随 A
    reg  [31:0] row_cfg_r;       // 0x24 行驱时序/极性
    reg  [31:0] slice_base_b_r;  // 0x28 屏 B 独立数据基址, 0 = 回落 slice_base_r
    reg  [7:0]  oe_w1_r, oe_w2_r; // 0x0C sub01 3-bit BCM plane1/2 OE 沿数
    reg         bpp3_r;           // 0x0C sub01 [16] 1 = 3-bit 行内 BCM
    reg         le_pl_r;          // 0x0C sub01 [17] plane1/2 LE 沿数选择

    //---------- 前向声明 ----------
    wire [15:0] at_slice_idx;
    wire [31:0] at_rev_period;
    wire        at_locked;
    wire        df_busy_w, df_done_w;
    wire        eng_a_busy, eng_b_busy;
    wire [31:0] eng_a_frame_period;      // R 0x28: 引擎 A 整屏 aclk 拍数
    wire        oe_a_w;
    wire        oe_a_state;
    reg         locked_ever;
    reg  [15:0] slice_max;
    reg  [15:0] pair_miss;

    // idx_B 实时计算 (02 §2.1: idx 域加偏移, 一个加法器 + 一个比较器;
    // 约束 phase_b < n_slices, 0..359 全范围有效)
    wire [16:0] idxb_sum   = {1'b0, at_slice_idx} + {8'b0, phase_b_r};
    wire [16:0] nsl_ext    = {1'b0, n_slices_r};
    wire [15:0] idx_b_live = (idxb_sum >= nsl_ext) ? (idxb_sum - nsl_ext)
                                                   : idxb_sum[15:0];

    // FOLD (面 A 半圈折叠, 0x10[6], 机械 v3.1): 面 A 穿转轴 ⇒ 后半圈的切片就是前
    // 半圈对应片的左右镜像, 于是 DDR 里只存 n_slices/2 片, PL 侧补齐:
    //   取片索引  idx >= half ? idx-half : idx      (half = n_slices>>1)
    //   镜像使能  idx >= half 时对面 A 额外翻一次 (与全局 mirror_a **异或**叠加)
    // half 跟 0x10[31:16] 的 n_slices 走, 不写死 180 (n_slices 可配, TB 里用 4/352)。
    // half==0 (n_slices<=1, 病态配置) 时整个折叠强制不生效, 免得 idx-0 白翻镜像。
    wire [15:0] half_slices = {1'b0, n_slices_r[15:1]};          // n_slices >> 1
    wire        fold_a_hit  = fold_a_en && (half_slices != 16'd0) &&
                              (at_slice_idx >= half_slices);
    wire [15:0] idx_a_live  = fold_a_hit ? (at_slice_idx - half_slices)
                                         : at_slice_idx;

    //---------- AXI-Lite WRITE FSM (套 v5 骨架) ----------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            sdi_mask      <= 9'b111_111_111;
            oe_set_pulse  <= 1'b0;
            oe_set_val    <= 1'b1;
            fb_we         <= 1'b0;
            fb_wlane      <= 4'd0;
            fb_waddr      <= 9'd0;
            fb_wdata      <= 32'd0;
            fb_wtgt       <= 1'b0;
            use_fb        <= 1'b0;
            auto_en       <= 1'b0;
            auto_pattern  <= 16'h8000;
            auto_disp_cyc <= 20'd3072;
            au_rows_max   <= 9'd383;
            dclk_fast     <= 1'b0;
            overlap_en    <= 1'b0;
            oe_window     <= 8'd48;      // 裁决: 默认 48 沿 (25% 占空, 供电顶格)
            pov_en        <= 1'b0;
            fake_en_r     <= 1'b0;
            dual_en       <= 1'b0;       // 复位 = 纯单屏 (v5 行为)
            fb_sel_b      <= 1'b0;
            mirror_b      <= 1'b0;   // 复位关, 保持旧行为
            mirror_a      <= 1'b0;
            fold_a_en     <= 1'b0;   // 复位关, 保持旧行为
            n_slices_r    <= 16'd360;
            fake_period_r <= 32'd6944;
            slice_base_r  <= 32'h1000_0000;
            phase_b_r     <= 9'd180;     // 背靠背默认 180° (机械 v3.1 软件应写 0)
            oe_window_b_r <= 8'd0;       // 0 = 跟随屏 A
            row_cfg_r     <= 32'h0;      // 全默认
            slice_base_b_r <= 32'h0;     // 0 = 屏 B 回落用 slice_base_r (旧行为)
            oe_w1_r       <= 8'd92;      // BCM 权重 2 (plane1); 184/92/46 = 4:2:1
            oe_w2_r       <= 8'd46;      // BCM 权重 1 (plane2 = LSB, 唯一受 ≤111 约束的)
            bpp3_r        <= 1'b0;       // 复位 = 1-bit, 旧内容/空闲动画照跑
            le_pl_r       <= 1'b0;       // 复位 = plane1/2 用 LE 3 沿 (datasheet)
        end else begin
            oe_set_pulse <= 1'b0;
            fb_we        <= 1'b0;
            if (!s_axi_awready && !s_axi_wready &&
                s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                if (s_axi_awaddr[15]) begin
                    // fb 窗口写: [14:11]=lane, [10:2]={row,word}; 目标由 fb_sel_b
                    fb_we    <= 1'b1;
                    fb_wlane <= s_axi_awaddr[14:11];
                    fb_waddr <= s_axi_awaddr[10:2];
                    fb_wdata <= s_axi_wdata;
                    fb_wtgt  <= fb_sel_b;
                end else case (s_axi_awaddr[5:2])
                    4'd3: begin                                  // 0x0C 杂项 (v5 原样)
                        if (s_axi_wdata[31:30] == 2'b00) begin
                            sdi_mask <= s_axi_wdata[8:0];
                        end else if (s_axi_wdata[31:30] == 2'b10) begin
                            oe_set_pulse <= 1'b1;
                            oe_set_val   <= s_axi_wdata[0];
                            if (s_axi_wdata[24:16] != 9'd0)
                                au_rows_max <= s_axi_wdata[24:16] - 1'b1;
                            if (s_axi_wdata[27]) begin           // v4 cfg_we
                                dclk_fast  <= s_axi_wdata[29];
                                overlap_en <= s_axi_wdata[28];
                                if (s_axi_wdata[15:8] != 8'd0)
                                    oe_window <= s_axi_wdata[15:8];
                            end
                        end else if (s_axi_wdata[31:30] == 2'b01) begin
                            // 3-bit 行内 BCM cfg (05_3bit_bcm.md 契约 v1)
                            // 沿数写 0 = 保持原值 (与本寄存器 oe_window 同惯例)
                            if (s_axi_wdata[7:0]  != 8'd0) oe_w1_r <= s_axi_wdata[7:0];
                            if (s_axi_wdata[15:8] != 8'd0) oe_w2_r <= s_axi_wdata[15:8];
                            bpp3_r  <= s_axi_wdata[16];
                            le_pl_r <= s_axi_wdata[17];
                        end else if (s_axi_wdata[31:30] == 2'b11) begin
                            auto_en       <= s_axi_wdata[0];
                            use_fb        <= s_axi_wdata[1];
                            auto_pattern  <= s_axi_wdata[23:8];
                            auto_disp_cyc <= (s_axi_wdata[29:24] == 6'd0) ? 20'd3072
                                             : {4'b0, s_axi_wdata[29:24], 10'b0};
                        end
                    end
                    4'd4: begin                                  // 0x10 POV_CTRL
                        pov_en    <= s_axi_wdata[0];
                        fake_en_r <= s_axi_wdata[1];
                        dual_en   <= s_axi_wdata[2];
                        fb_sel_b  <= s_axi_wdata[3];
                        mirror_b  <= s_axi_wdata[4];
                        mirror_a  <= s_axi_wdata[5];
                        fold_a_en <= s_axi_wdata[6];
                        if (s_axi_wdata[31:16] != 16'd0)
                            n_slices_r <= s_axi_wdata[31:16];
                    end
                    4'd5: fake_period_r <= s_axi_wdata;          // 0x14
                    4'd6: slice_base_r  <= s_axi_wdata;          // 0x18
                    4'd7: phase_b_r     <= s_axi_wdata[8:0];     // 0x1C PHASE_B
                    4'd8: oe_window_b_r <= s_axi_wdata[7:0];     // 0x20 BRIGHT_B
                    4'd9: row_cfg_r     <= s_axi_wdata;          // 0x24 行驱 cfg
                    4'd10: slice_base_b_r <= s_axi_wdata;        // 0x28 SLICE_BASE_B
                    default: ;   // 0x00/04/08 手动命令归引擎黑盒, 0x2C-0x3C 未分配 (遗留)
                endcase
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    s_axi_bvalid <= 1'b0;
            end
        end
    end

    //---------- pair 触发器 (02 §2.2 翻页原子点) ----------
    localparam [1:0] P_IDLE = 2'd0, P_A = 2'd1, P_B = 2'd2;
    reg  [1:0]  pstate;
    reg         df_go, df_tgt;
    reg  [8:0]  df_idx;
    reg  [31:0] base_lat;        // pair 级 slice_base 快照 (屏 A)
    reg  [31:0] base_lat_b;      // pair 级 slice_base_b 快照 (屏 B), 与 base_lat 同拍
    reg         fold_mir;        // 本 pair 的屏 A fetch 是否落在折叠后半圈 (要额外镜像)
    reg  [15:0] df_last_slice;
    reg  [15:0] idx_b_lat;
    reg  [15:0] idx_prev;
    wire        pair_busy = (pstate != P_IDLE);

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            pstate        <= P_IDLE;
            df_go         <= 1'b0;
            df_tgt        <= 1'b0;
            df_idx        <= 9'd0;
            base_lat      <= 32'h1000_0000;
            base_lat_b    <= 32'h1000_0000;
            fold_mir      <= 1'b0;
            df_last_slice <= 16'hFFFF;
            idx_b_lat     <= 16'd0;
            idx_prev      <= 16'd0;
            pair_miss     <= 16'd0;
        end else begin
            df_go    <= 1'b0;
            idx_prev <= at_slice_idx;
            case (pstate)
                P_IDLE: if (pov_en && at_slice_idx != df_last_slice) begin
                    df_last_slice <= at_slice_idx;
                    base_lat      <= slice_base_r;   // 翻页原子点: 每 pair 采样一次
                    // 屏 B 基址与 base_lat 同一拍快照 → 两面必来自同一帧, 不撕裂。
                    // slice_base_b_r==0 时取 slice_base_r (与 base_lat 同源同拍),
                    // 即老的"两面共用一份数据"行为, 逐位不变。
                    base_lat_b    <= (slice_base_b_r == 32'd0) ? slice_base_r
                                                               : slice_base_b_r;
                    idx_b_lat     <= idx_b_live;
                    df_idx        <= idx_a_live[8:0];  // 折叠关时 == at_slice_idx[8:0]
                    fold_mir      <= fold_a_hit;       // 与取片索引同拍锁存
                    df_tgt        <= 1'b0;
                    df_go         <= 1'b1;
                    pstate        <= P_A;
                end
                P_A: if (df_done_w) begin
                    if (dual_en) begin
                        df_idx <= idx_b_lat[8:0];
                        df_tgt <= 1'b1;
                        df_go  <= 1'b1;
                        pstate <= P_B;
                    end else
                        pstate <= P_IDLE;            // dual_en=0: 行为 = v5 单 fetch
                end
                P_B: if (df_done_w) pstate <= P_IDLE;
                default: pstate <= P_IDLE;
            endcase
            // pair_miss: pair 期间 idx 变化 = 该取帧对被整对丢弃 (饱和哨兵)
            if (pov_en && pair_busy && (at_slice_idx != idx_prev) &&
                pair_miss != 16'hFFFF)
                pair_miss <= pair_miss + 16'd1;
            if (!pov_en) begin
                df_last_slice <= 16'hFFFF;
                pstate        <= P_IDLE;   // 在飞 fetch 自跑完, 写被 pov_en mux 丢弃
                fold_mir      <= 1'b0;     // pov_en=0 走 AXI fb 窗直灌, 折叠镜像必须撤掉
            end
        end
    end

    //---------- AXI-Lite READ FSM ----------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'b0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                case (s_axi_araddr[5:2])
                    4'd4:    s_axi_rdata <= {at_locked, 15'b0, at_slice_idx};   // 0x10
                    4'd5:    s_axi_rdata <= at_rev_period;                      // 0x14
                    4'd6:    s_axi_rdata <= {locked_ever, 15'b0, slice_max};    // 0x18
                    4'd7:    s_axi_rdata <= {at_locked, 6'b0,                   // 0x1C
                                             idx_b_live[8:0], pair_miss};
                    // 0x24 R: POV_CTRL 影子回读 (0x10 是只写的, 读 0x10 已被
                    // {locked,idx} 占死不能动) — 把 0x10 那组寄存器的当前值原样拼
                    // 回去, 位序与写口逐位对齐, 软件可直接 RMW 后写回 0x10。
                    // 0x28 R: 引擎 A 最近一整屏 aclk 拍数 (纯 PL 计数, 无频率假设)
                    // —— 与 CPU 墙钟测的整屏耗时相除即得 aclk 真频率。
                    // 写 0x28 是 SLICE_BASE_B, 读写不同义 (同 0x24 套路)。
                    4'd10:   s_axi_rdata <= eng_a_frame_period;                 // 0x28
                    4'd9:    s_axi_rdata <= {n_slices_r, 9'b0, fold_a_en,        // 0x24
                                             mirror_a, mirror_b, fb_sel_b,
                                             dual_en, fake_en_r, pov_en};
                    // 0x00 status: [16]=base_b_act [15]=fold_a_en (机械 v3.1 新增,
                    // 复位默认都是 0 → 复位态回读值与加这两位之前逐位一致)
                    default: s_axi_rdata <= {15'b0, (slice_base_b_r != 32'd0), fold_a_en,
                                             mirror_a, mirror_b, eng_b_busy, dual_en,
                                             (pair_busy | df_busy_w), pov_en,
                                             at_locked, dclk_fast, overlap_en,
                                             use_fb, auto_en, oe_a_state,
                                             2'b00, eng_a_busy};
                endcase
                s_axi_rresp  <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

    //---------- angle_tracker (v5 文件里的模块, 零改动) ----------
    angle_tracker #(.CLK_HZ(50000000)) u_angle (
        .clk         (s_axi_aclk),
        .rst_n       (s_axi_aresetn),
        .sensor_in   (spin_sync),
        .fake_en     (fake_en_r),
        .fake_period (fake_period_r),
        .n_slices    (n_slices_r),
        .slice_idx   (at_slice_idx),
        .rev_period  (at_rev_period),
        .locked      (at_locked)
    );

    // 停转诊断锁存 (v5 原样)
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            locked_ever <= 1'b0;
            slice_max   <= 16'd0;
        end else begin
            if (at_locked) locked_ever <= 1'b1;
            if (at_slice_idx > slice_max) slice_max <= at_slice_idx;
        end
    end

    //---------- 取帧引擎 (256-beat + target) ----------
    wire        df_fb_we;
    wire        df_fb_wtgt;
    wire [3:0]  df_fb_wlane;
    wire [8:0]  df_fb_waddr;
    wire [1:0]  df_fb_wplane;
    wire [31:0] df_fb_wdata;

    // A/B 两次 fetch 是**串行**复用同一个 u_fetch (P_A → P_B), 所以基址按当前 pair
    // 阶段的 df_tgt 选: A 用 base_lat, B 用 base_lat_b。df_tgt 与 df_go 是同一拍
    // 寄存器输出, u_fetch 又是在 fetch_go 那拍才把 slice_base 算进 cur_addr,
    // 所以选中的基址在采样沿上已经稳定, 无竞争。
    // slice_base_b_r==0 时 base_lat_b 快照的就是 slice_base_r → 与 base_lat 全等,
    // 老行为逐位不变 (TB 里拿 dut.base_lat 当 A/B 共同期望值也仍然成立)。
    wire [31:0] base_lat_sel = df_tgt ? base_lat_b : base_lat;

    ddr_slice_fetch256 u_fetch (
        .aclk          (s_axi_aclk),
        .aresetn       (s_axi_aresetn),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_arsize  (m_axi_arsize),
        .m_axi_arburst (m_axi_arburst),
        .m_axi_arlock  (m_axi_arlock),
        .m_axi_arcache (m_axi_arcache),
        .m_axi_arprot  (m_axi_arprot),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rresp   (m_axi_rresp),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .slice_base    (base_lat_sel),
        .bpp3          (bpp3_r),
        .slice_idx     (df_idx),
        .fetch_target  (df_tgt),
        .fetch_go      (df_go),
        .fetch_busy    (df_busy_w),
        .fetch_done    (df_done_w),
        .fb_we         (df_fb_we),
        .fb_wtarget    (df_fb_wtgt),
        .fb_wlane      (df_fb_wlane),
        .fb_waddr      (df_fb_waddr),
        .fb_wplane     (df_fb_wplane),
        .fb_wdata      (df_fb_wdata)
    );

    //---------- fb 写仲裁 + 按 target 路由 (v5 语义: pov_en 时取帧引擎独占) ----------
    wire        fbw_we   = pov_en ? df_fb_we    : fb_we;
    wire        fbw_tgt  = pov_en ? df_fb_wtgt  : fb_wtgt;
    wire [3:0]  fbw_lane = pov_en ? df_fb_wlane : fb_wlane;
    wire [8:0]  fbw_addr = pov_en ? df_fb_waddr : fb_waddr;
    // 3-bit BCM 的位平面号只有取帧引擎会给; AXI fb 窗 (调试/空闲动画路径) 地址位
    // 只有 9 bit ({row,word}), 够不着 plane 字段 ⇒ 恒写 plane0。
    // ⇒ bpp_mode=1 时 AXI fb 窗只能改 plane0, 完整 3-bit 内容走 DDR 取帧。
    wire [1:0]  fbw_plane = pov_en ? df_fb_wplane : 2'd0;
    wire [31:0] fbw_data = pov_en ? df_fb_wdata : fb_wdata;
    wire        fbA_we   = fbw_we & ~fbw_tgt;
    wire        fbB_we   = fbw_we &  fbw_tgt;

    //---------- MIRROR: 屏 B 左右镜像 (0x10[4], 2026-07-28) ----------
    // 两屏背靠背, 屏 B 的 X 轴在世界坐标里与屏 A 相反 → 需对中心轴做轴对称。
    // 因 mirror(slice_j) 不等于切片集合里任何一片 (翻 û 必然同时翻 n̂), 软件侧
    // 只能靠第二份数据解决; 放到这里做则零额外 DDR/带宽。
    //
    // 观察者 X → 159-X 在打包域 (lane_base, row) 的置换 (tools/pack_obs.py 权威,
    // 条布局 X 0..52=lane基6(53宽) / 53..105=lane基3(53宽) / 106..159=lane基0(54宽)):
    //   (6, r)  → (0, 52-r)   r=0..52      左条 ↔ 右条
    //   (0, r)  → (6, 52-r)   r=0..52
    //   (3, r)  → (3, 51-r)   r=0..51      中条自映射, 注意常数是 51 不是 52
    //   (3, 52) → (0, 53)                  ← 例外: 三条宽 53/53/54 不对称所致
    //   (0, 53) → (3, 52)                  ← 例外
    //   (6,53)/(3,53) 是 53 宽条的无效槽 (打包时补 0), 原地映射即可。
    // 只改**写地址**, DDR 突发读仍是线性的 → 不占额外带宽, 不加 BRAM 端口。
    wire [1:0] mb_grp  = (fbw_lane >= 4'd6) ? 2'd2 :          // lane 基 6
                         (fbw_lane >= 4'd3) ? 2'd1 : 2'd0;    // 基 3 / 基 0
    wire [1:0] mb_col  = fbw_lane - {mb_grp, 1'b0} - {1'b0, mb_grp};  // lane - grp*3
    wire [5:0] mb_row  = fbw_addr[8:3];
    wire [2:0] mb_word = fbw_addr[2:0];

    reg  [1:0] mb_grp2;
    reg  [5:0] mb_row2;
    always @* begin
        case (mb_grp)
            2'd2: if (mb_row <= 6'd52) begin mb_grp2 = 2'd0; mb_row2 = 6'd52 - mb_row; end
                  else                 begin mb_grp2 = 2'd2; mb_row2 = mb_row;         end
            2'd0: if (mb_row <= 6'd52) begin mb_grp2 = 2'd2; mb_row2 = 6'd52 - mb_row; end
                  else                 begin mb_grp2 = 2'd1; mb_row2 = 6'd52;          end
            default: if (mb_row <= 6'd51) begin mb_grp2 = 2'd1; mb_row2 = 6'd51 - mb_row; end
                     else if (mb_row == 6'd52) begin mb_grp2 = 2'd0; mb_row2 = 6'd53;  end
                     else begin mb_grp2 = 2'd1; mb_row2 = mb_row; end
        endcase
    end
    wire [3:0] mb_lane2 = {2'b0, mb_grp2} + {1'b0, mb_grp2, 1'b0} + {2'b0, mb_col}; // grp2*3+col
    //---------- FOLD: 面 A 折叠的后半圈镜像 (0x10[6], 机械 v3.1) ----------
    // 折叠时后半圈 (idx >= n_slices/2) 取的是前半圈那片的数据, 要再做一次上面同一个
    // 左右镜像置换才是正确内容。用**异或**叠加到全局 mirror_a 上, 而不是覆盖:
    //   mirror_a=0, fold 未命中 → 不镜像 (= 旧行为)
    //   mirror_a=1, fold 未命中 → 镜像一次 (= 旧行为)
    //   mirror_a=0, fold 命中   → 镜像一次 (折叠补齐)
    //   mirror_a=1, fold 命中   → 两次镜像抵消 (置换是对合的, f(f(x))==x, 已独立验证)
    // 复用同一套 mb_grp2/mb_row2 组合逻辑, 不加面积; 置换本身一个字都没动。
    // fold_mir 是 pair 级锁存 (与 df_idx 同拍), A/B fetch 串行不重叠, 所以整个 A
    // fetch 期间它恒定; B 的写走 fbB_* 分支, 不受它影响; pov_en=0 时被清 0。
    wire       mirror_a_eff = mirror_a ^ fold_mir;
    wire [3:0] fbB_lane = mirror_b     ? mb_lane2            : fbw_lane;
    wire [8:0] fbB_addr = mirror_b     ? {mb_row2, mb_word}  : fbw_addr;
    wire [3:0] fbA_lane = mirror_a_eff ? mb_lane2            : fbw_lane;
    wire [8:0] fbA_addr = mirror_a_eff ? {mb_row2, mb_word}  : fbw_addr;

    //---------- 3-bit BCM: {row,word}+plane → 片内紧凑地址 (2026-08-20) ----------
    // 引擎读侧是纯递增计数器, 要求 raddr = row*18 + plane*6 + pair (0..971)。
    // 换算放在**镜像置换之后**: 置换只动 (lane,row), 与 plane 正交 ⇒ 镜像/折叠
    // 逻辑一个字都不用改, 三个平面各自被同样地置换。
    // row*18 = row<<4 + row<<1; plane*6 = plane<<2 + plane<<1 (无乘法器)。
    // bpp3_r=0 时直接零扩展旧地址 ⇒ 老路径逐 bit 不变。
    function [9:0] pack_addr;
        input [8:0] a9;
        input [1:0] pl;
        reg   [5:0] rw;
        reg   [2:0] wd;
        begin
            rw = a9[8:3];
            wd = a9[2:0];
            pack_addr = {rw, 4'b0} + {3'b0, rw, 1'b0}
                      + {6'b0, pl, 2'b0} + {7'b0, pl, 1'b0} + {7'b0, wd};
        end
    endfunction
    wire [9:0] fbA_waddr = bpp3_r ? pack_addr(fbA_addr, fbw_plane) : {1'b0, fbA_addr};
    wire [9:0] fbB_waddr = bpp3_r ? pack_addr(fbB_addr, fbw_plane) : {1'b0, fbB_addr};

    //---------- 双引擎 (黑盒; xsim 用 panel_engine_stub.v) ----------
    wire [7:0] oe_window_b_eff = (oe_window_b_r == 8'd0) ? oe_window
                                                         : oe_window_b_r;
    // B 使能: dual_en 或 fb_sel_b 调试直灌 (02 §7 阶段1); 双 0 = 静默消隐
    wire eng_b_en = dual_en | fb_sel_b;

    panel_engine #(.DCLK_DIV(DCLK_DIV)) u_eng_a (
        .clk           (s_axi_aclk),
        .rst_n         (s_axi_aresetn),
        .enable        (1'b1),
        .fb_we         (fbA_we),
        .fb_wlane      (fbA_lane),
        .fb_waddr      (fbA_waddr),
        .fb_wdata      (fbw_data),
        .auto_en       (auto_en),
        .use_fb        (use_fb),
        .auto_pattern  (auto_pattern),
        .auto_disp_cyc (auto_disp_cyc),
        .au_rows_max   (au_rows_max),
        .dclk_fast     (dclk_fast),
        .overlap_en    (overlap_en),
        .oe_window     (oe_window),
        .oe_w1         (oe_w1_r),
        .oe_w2         (oe_w2_r),
        .bpp_mode      (bpp3_r),
        .le_plane_mode (le_pl_r),
        .sdi_mask      (sdi_mask),
        .oe_set_pulse  (oe_set_pulse),
        .oe_set_val    (oe_set_val),
        .row_cfg       (row_cfg_r),
        .engine_busy   (eng_a_busy),
        .frame_period  (eng_a_frame_period),
        .dclk_out      (dclk_out),
        .le_out        (le_out),
        .oe_out        (oe_a_w),
        .oe_state      (oe_a_state),
        .sdi_out       (sdi_out),
        .icnd_sdi_out  (icnd_sdi_out),
        .icnd_dclk_out (icnd_dclk_out),
        .icnd_rclk_out (icnd_rclk_out)
    );
    assign oe_out = oe_a_w;

    panel_engine #(.DCLK_DIV(DCLK_DIV)) u_eng_b (
        .clk           (s_axi_aclk),
        .rst_n         (s_axi_aresetn),
        .enable        (eng_b_en),
        .fb_we         (fbB_we),
        .fb_wlane      (fbB_lane),
        .fb_waddr      (fbB_waddr),
        .fb_wdata      (fbw_data),
        .auto_en       (auto_en),
        .use_fb        (use_fb),
        .auto_pattern  (auto_pattern),
        .auto_disp_cyc (auto_disp_cyc),
        .au_rows_max   (au_rows_max),
        .dclk_fast     (dclk_fast),
        .overlap_en    (overlap_en),
        .oe_window     (oe_window_b_eff),
        .oe_w1         (oe_w1_r),
        .oe_w2         (oe_w2_r),
        .bpp_mode      (bpp3_r),
        .le_plane_mode (le_pl_r),
        .sdi_mask      (sdi_mask),
        .oe_set_pulse  (oe_set_pulse),
        .oe_set_val    (oe_set_val),
        .row_cfg       (row_cfg_r),
        .engine_busy   (eng_b_busy),
        .frame_period  (),
        .dclk_out      (dclk_out_2),
        .le_out        (le_out_2),
        .oe_out        (oe_out_2),
        .oe_state      (),
        .sdi_out       (sdi_out_2),
        .icnd_sdi_out  (icnd_sdi_out_2),
        .icnd_dclk_out (icnd_dclk_out_2),
        .icnd_rclk_out (icnd_rclk_out_2)
    );

endmodule

//==============================================================================
// ddr_slice_fetch256 — v5 ddr_slice_fetch 的双屏版 (v5 原文件不动):
//   * burst 上限 16 → 256 beat (1KB, v6 同款; 02 §1.1 带宽账 ~92% 有效 →
//     ~185 MB/s, 双屏 109.2 MB/s 需求余量 1.7×). 4KB 边界动态截断逻辑保留.
//   * 加 1-bit fetch_target, go 拍锁存, 随 fb 写口透传 (顶层按它路由 fb_A/fb_B).
//   其余 (单 outstanding / arvalid hold / INCR / RRESP 容忍 / lane-major
//   换算) 与 v5 逐行一致.
//
// [2026-08-20 feature/3bit-color] bpp3=1 时一次取**三个位平面**:
//   片内布局 = plane p 在 slice_base + idx*0x9000 + p*0x3000, 每 plane 内部沿用
//   现有 0x3000 布局 (lane-major, 2916 word 实占 / 3072 word 跨度)。
//   ⇒ 每 plane 一轮 TOTAL_WORDS 的突发序列, 跨 plane 时把 cur_addr 跳到下一个
//   0x3000 边界 (中间 624 B 空隙不读), lane/row/word 计数器归零重来。
//   fb_wplane 随写口透传, 顶层再折成片内紧凑地址。
//   bpp3=0 时全部逻辑退化成原来的单 plane 版本, 逐拍等价。
//==============================================================================
module ddr_slice_fetch256 #(
    parameter [31:0] SLICE_STRIDE = 32'h0000_3000,  // 每 slice 字节跨度
    parameter [11:0] TOTAL_WORDS  = 12'd2916        // 11664 B / 4
)(
    input  wire        aclk,
    input  wire        aresetn,

    output wire [31:0] m_axi_araddr,
    output reg  [7:0]  m_axi_arlen,     // 动态 0..255 (<=256 beats)
    output wire [2:0]  m_axi_arsize,    // 固定 2 = 4 byte/beat
    output wire [1:0]  m_axi_arburst,   // 固定 INCR
    output wire        m_axi_arlock,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,

    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,

    input  wire [31:0] slice_base,      // pair 级快照后的基址
    input  wire        bpp3,            // 1 = 3-bit BCM (每片 3 个位平面), go 拍锁存
    input  wire [8:0]  slice_idx,       // 0..359, fetch_go 时锁存
    input  wire        fetch_target,    // 0=fb_A 1=fb_B, fetch_go 时锁存
    input  wire        fetch_go,        // 1 拍脉冲
    output wire        fetch_busy,
    output reg         fetch_done,      // 1 拍脉冲

    output reg         fb_we,
    output reg         fb_wtarget,
    output reg  [3:0]  fb_wlane,
    output reg  [8:0]  fb_waddr,        // {row[5:0], word[2:0]} (plane 内偏移)
    output reg  [1:0]  fb_wplane,       // 位平面号 (bpp3=0 时恒 0)
    output reg  [31:0] fb_wdata
);

    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;

    localparam [1:0]
        F_IDLE = 2'd0,
        F_CALC = 2'd1,
        F_AR   = 2'd2,
        F_R    = 2'd3;

    reg [1:0]  state;
    reg [31:0] cur_addr;
    reg [11:0] words_left;
    reg [3:0]  lane;            // 0..8
    reg [5:0]  row;             // 0..53
    reg [2:0]  word;            // 0..5
    reg [3:0]  err_cnt;
    reg        tgt_r;
    reg [1:0]  plane;           // 0..2 (bpp3=0 时恒 0)
    reg        bpp3_l;          // fetch_go 拍锁存
    reg [31:0] plane_base;      // 当前 plane 的 0x3000 块基址

    assign m_axi_araddr = cur_addr;
    assign m_axi_rready = (state == F_R);
    assign fetch_busy   = (state != F_IDLE);

    wire beat = m_axi_rvalid && m_axi_rready;

    // 片跨度: 1-bit = idx*0x3000, 3-bit = idx*0x9000 (三个平面顺序排列)
    wire [31:0] slice_off = bpp3 ? ({8'b0,  slice_idx, 15'b0} + {11'b0, slice_idx, 12'b0})
                                 : ({10'b0, slice_idx, 13'b0} + {11'b0, slice_idx, 12'b0});

    // burst 长度: min(256, words_left, 到下一 4KB 边界的 beat 数)
    wire [10:0] beats_to_4k = 11'd1024 - {1'b0, cur_addr[11:2]};
    wire [11:0] lim_words   = (words_left < 12'd256) ? words_left : 12'd256;
    wire [11:0] burst_beats = ({1'b0, beats_to_4k} < lim_words)
                              ? {1'b0, beats_to_4k} : lim_words;  // 1..256

    always @(posedge aclk) begin
        if (!aresetn) begin
            state         <= F_IDLE;
            cur_addr      <= 32'd0;
            words_left    <= 12'd0;
            lane          <= 4'd0;
            row           <= 6'd0;
            word          <= 3'd0;
            err_cnt       <= 4'd0;
            tgt_r         <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_arlen   <= 8'd0;
            fetch_done    <= 1'b0;
            fb_we         <= 1'b0;
            fb_wtarget    <= 1'b0;
            fb_wlane      <= 4'd0;
            fb_waddr      <= 9'd0;
            fb_wplane     <= 2'd0;
            fb_wdata      <= 32'd0;
            plane         <= 2'd0;
            bpp3_l        <= 1'b0;
            plane_base    <= 32'd0;
        end else begin
            fetch_done <= 1'b0;
            fb_we      <= 1'b0;

            if (beat && m_axi_rresp[1] && err_cnt != 4'hF)
                err_cnt <= err_cnt + 4'd1;

            case (state)
                F_IDLE: begin
                    if (fetch_go) begin
                        // 1-bit: slice_base + idx*0x3000 (<<13 + <<12)
                        // 3-bit: slice_base + idx*0x9000 (<<15 + <<12), 3 个平面顺排
                        cur_addr   <= slice_base + slice_off;
                        plane_base <= slice_base + slice_off;
                        words_left <= TOTAL_WORDS;
                        lane       <= 4'd0;
                        row        <= 6'd0;
                        word       <= 3'd0;
                        plane      <= 2'd0;
                        bpp3_l     <= bpp3;
                        tgt_r      <= fetch_target;
                        state      <= F_CALC;
                    end
                end

                F_CALC: begin
                    // burst_beats=256 时 [7:0]=0, -1 回绕成 255 = arlen 正确
                    m_axi_arlen   <= burst_beats[7:0] - 8'd1;
                    m_axi_arvalid <= 1'b1;
                    state         <= F_AR;
                end

                F_AR: begin
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        state         <= F_R;
                    end
                end

                F_R: begin
                    if (beat) begin
                        fb_we      <= 1'b1;
                        fb_wtarget <= tgt_r;
                        fb_wlane   <= lane;
                        fb_waddr   <= {row, word};
                        fb_wplane  <= plane;
                        fb_wdata   <= m_axi_rdata;

                        if (word == 3'd5) begin
                            word <= 3'd0;
                            if (row == 6'd53) begin
                                row  <= 6'd0;
                                lane <= lane + 4'd1;
                            end else begin
                                row <= row + 6'd1;
                            end
                        end else begin
                            word <= word + 3'd1;
                        end

                        cur_addr   <= cur_addr + 32'd4;
                        words_left <= words_left - 12'd1;

                        if (m_axi_rlast) begin
                            if (words_left == 12'd1) begin
                                if (bpp3_l && plane != 2'd2) begin
                                    // 下一个位平面: 跳到 +0x3000 的块首重来
                                    // (这几条覆盖上面的 cur_addr/words_left/计数器)
                                    plane      <= plane + 2'd1;
                                    plane_base <= plane_base + 32'h0000_3000;
                                    cur_addr   <= plane_base + 32'h0000_3000;
                                    words_left <= TOTAL_WORDS;
                                    lane       <= 4'd0;
                                    row        <= 6'd0;
                                    word       <= 3'd0;
                                    state      <= F_CALC;
                                end else begin
                                    fetch_done <= 1'b1;
                                    state      <= F_IDLE;
                                end
                            end else begin
                                state <= F_CALC;
                            end
                        end
                    end
                end

                default: state <= F_IDLE;
            endcase
        end
    end

endmodule
