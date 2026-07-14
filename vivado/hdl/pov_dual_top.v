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
//         subcmd 01 (per-chain 预载) 属引擎手动命令路径, 顶层暂不接 (见遗留).
//   W 0x10 POV_CTRL: [0]=pov_en [1]=fake_en [31:16]=n_slices (v5 原样);
//         新 [2]=dual_en (0=纯单屏, B 引擎静默消隐, 行为=v5)
//         新 [3]=fb_sel_b (AXI fb 窗 awaddr[15]=1 的写落 fb_B, pov_en=0 调试直灌;
//              置位时 B 引擎也使能 — 02 §7 阶段1 "dual_en=0 直灌 fb_B + auto 扫" 用)
//   W 0x14 fake_period / W 0x18 slice_base (v5 原样; 翻页原子性由 pair 快照兜底)
//   W 0x1C PHASE_B: [8:0] 屏 B slice 偏移, 复位默认 180 (要求 < n_slices)
//   W 0x20 BRIGHT_B: [7:0] oe_window_B, 0=跟随屏 A oe_window (复位默认 0)
//   R 0x00 status: v5 原位 [0]=engineA_busy [3]=oe_A [4]=auto_en [5]=use_fb
//         [6]=overlap_en [7]=dclk_fast [8]=locked [9]=pov_en [10]=fetch/pair busy
//         + 新 [11]=dual_en [12]=engine_B_busy ([1]/[2] cmd_pending/icnd_busy
//         归引擎黑盒, 恒 0; 02 §3 表把 [11]/[12] 写在 "0x10 R" 是地址笔误 —
//         0x10 读保持 v5 {locked,idx} 布局不破坏老脚本, 状态位落 0x00)
//   R 0x10 {locked,15'b0,slice_idx} / R 0x14 rev_period /
//   R 0x18 {locked_ever,15'b0,slice_max}  (v5 原样)
//   R 0x1C {locked, 6'b0, idx_B[8:0], pair_miss[15:0]} — idx_B 实时算 (核相位),
//         pair_miss = pair_busy 期间 slice_idx 变化 (整对丢弃) 的饱和计数
//
// pair 级翻页原子性 (02 §2.2): idx 变化那拍把 slice_base_r 快照进 base_lat,
//   A/B 两次 fetch 都用 base_lat → pov_rxd 写 0x18 的时机完全不用管;
//   pair 期间新 idx 整对丢弃 (丢帧跳最新, v5 同语义), 绝不只丢 B 不丢 A.
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
    reg  [15:0] n_slices_r;
    reg  [31:0] fake_period_r;
    reg  [31:0] slice_base_r;
    reg  [8:0]  phase_b_r;       // 0x1C, 默认 180
    reg  [7:0]  oe_window_b_r;   // 0x20, 0 = 跟随 A

    //---------- 前向声明 ----------
    wire [15:0] at_slice_idx;
    wire [31:0] at_rev_period;
    wire        at_locked;
    wire        df_busy_w, df_done_w;
    wire        eng_a_busy, eng_b_busy;
    wire        oe_a_w;
    reg         locked_ever;
    reg  [15:0] slice_max;
    reg  [15:0] pair_miss;

    // idx_B 实时计算 (02 §2.1: idx 域加偏移, 一个加法器 + 一个比较器;
    // 约束 phase_b < n_slices, 0..359 全范围有效)
    wire [16:0] idxb_sum   = {1'b0, at_slice_idx} + {8'b0, phase_b_r};
    wire [16:0] nsl_ext    = {1'b0, n_slices_r};
    wire [15:0] idx_b_live = (idxb_sum >= nsl_ext) ? (idxb_sum - nsl_ext)
                                                   : idxb_sum[15:0];

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
            n_slices_r    <= 16'd360;
            fake_period_r <= 32'd6944;
            slice_base_r  <= 32'h1000_0000;
            phase_b_r     <= 9'd180;     // 背靠背默认 180°
            oe_window_b_r <= 8'd0;       // 0 = 跟随屏 A
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
                        end else if (s_axi_wdata[31:30] == 2'b11) begin
                            auto_en       <= s_axi_wdata[0];
                            use_fb        <= s_axi_wdata[1];
                            auto_pattern  <= s_axi_wdata[23:8];
                            auto_disp_cyc <= (s_axi_wdata[29:24] == 6'd0) ? 20'd3072
                                             : {4'b0, s_axi_wdata[29:24], 10'b0};
                        end
                        // subcmd 01 per-chain 预载: 引擎手动路径, 顶层不实现 (遗留)
                    end
                    4'd4: begin                                  // 0x10 POV_CTRL
                        pov_en    <= s_axi_wdata[0];
                        fake_en_r <= s_axi_wdata[1];
                        dual_en   <= s_axi_wdata[2];
                        fb_sel_b  <= s_axi_wdata[3];
                        if (s_axi_wdata[31:16] != 16'd0)
                            n_slices_r <= s_axi_wdata[31:16];
                    end
                    4'd5: fake_period_r <= s_axi_wdata;          // 0x14
                    4'd6: slice_base_r  <= s_axi_wdata;          // 0x18
                    4'd7: phase_b_r     <= s_axi_wdata[8:0];     // 0x1C PHASE_B
                    4'd8: oe_window_b_r <= s_axi_wdata[7:0];     // 0x20 BRIGHT_B
                    default: ;   // 0x00/04/08 手动命令归引擎黑盒 (遗留)
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
    reg  [31:0] base_lat;        // pair 级 slice_base 快照
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
                    idx_b_lat     <= idx_b_live;
                    df_idx        <= at_slice_idx[8:0];
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
                    default: s_axi_rdata <= {19'b0, eng_b_busy, dual_en,        // 0x00
                                             (pair_busy | df_busy_w), pov_en,
                                             at_locked, dclk_fast, overlap_en,
                                             use_fb, auto_en, oe_a_w,
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
    wire [31:0] df_fb_wdata;

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
        .slice_base    (base_lat),
        .slice_idx     (df_idx),
        .fetch_target  (df_tgt),
        .fetch_go      (df_go),
        .fetch_busy    (df_busy_w),
        .fetch_done    (df_done_w),
        .fb_we         (df_fb_we),
        .fb_wtarget    (df_fb_wtgt),
        .fb_wlane      (df_fb_wlane),
        .fb_waddr      (df_fb_waddr),
        .fb_wdata      (df_fb_wdata)
    );

    //---------- fb 写仲裁 + 按 target 路由 (v5 语义: pov_en 时取帧引擎独占) ----------
    wire        fbw_we   = pov_en ? df_fb_we    : fb_we;
    wire        fbw_tgt  = pov_en ? df_fb_wtgt  : fb_wtgt;
    wire [3:0]  fbw_lane = pov_en ? df_fb_wlane : fb_wlane;
    wire [8:0]  fbw_addr = pov_en ? df_fb_waddr : fb_waddr;
    wire [31:0] fbw_data = pov_en ? df_fb_wdata : fb_wdata;
    wire        fbA_we   = fbw_we & ~fbw_tgt;
    wire        fbB_we   = fbw_we &  fbw_tgt;

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
        .fb_wlane      (fbw_lane),
        .fb_waddr      (fbw_addr),
        .fb_wdata      (fbw_data),
        .auto_en       (auto_en),
        .use_fb        (use_fb),
        .auto_pattern  (auto_pattern),
        .auto_disp_cyc (auto_disp_cyc),
        .au_rows_max   (au_rows_max),
        .dclk_fast     (dclk_fast),
        .overlap_en    (overlap_en),
        .oe_window     (oe_window),
        .sdi_mask      (sdi_mask),
        .oe_set_pulse  (oe_set_pulse),
        .oe_set_val    (oe_set_val),
        .engine_busy   (eng_a_busy),
        .dclk_out      (dclk_out),
        .le_out        (le_out),
        .oe_out        (oe_a_w),
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
        .fb_wlane      (fbw_lane),
        .fb_waddr      (fbw_addr),
        .fb_wdata      (fbw_data),
        .auto_en       (auto_en),
        .use_fb        (use_fb),
        .auto_pattern  (auto_pattern),
        .auto_disp_cyc (auto_disp_cyc),
        .au_rows_max   (au_rows_max),
        .dclk_fast     (dclk_fast),
        .overlap_en    (overlap_en),
        .oe_window     (oe_window_b_eff),
        .sdi_mask      (sdi_mask),
        .oe_set_pulse  (oe_set_pulse),
        .oe_set_val    (oe_set_val),
        .engine_busy   (eng_b_busy),
        .dclk_out      (dclk_out_2),
        .le_out        (le_out_2),
        .oe_out        (oe_out_2),
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
    input  wire [8:0]  slice_idx,       // 0..359, fetch_go 时锁存
    input  wire        fetch_target,    // 0=fb_A 1=fb_B, fetch_go 时锁存
    input  wire        fetch_go,        // 1 拍脉冲
    output wire        fetch_busy,
    output reg         fetch_done,      // 1 拍脉冲

    output reg         fb_we,
    output reg         fb_wtarget,
    output reg  [3:0]  fb_wlane,
    output reg  [8:0]  fb_waddr,        // {row[5:0], word[2:0]}
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

    assign m_axi_araddr = cur_addr;
    assign m_axi_rready = (state == F_R);
    assign fetch_busy   = (state != F_IDLE);

    wire beat = m_axi_rvalid && m_axi_rready;

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
            fb_wdata      <= 32'd0;
        end else begin
            fetch_done <= 1'b0;
            fb_we      <= 1'b0;

            if (beat && m_axi_rresp[1] && err_cnt != 4'hF)
                err_cnt <= err_cnt + 4'd1;

            case (state)
                F_IDLE: begin
                    if (fetch_go) begin
                        // 起始地址 = slice_base + slice_idx*0x3000 (<<13 + <<12)
                        cur_addr   <= slice_base
                                      + {10'b0, slice_idx, 13'b0}
                                      + {11'b0, slice_idx, 12'b0};
                        words_left <= TOTAL_WORDS;
                        lane       <= 4'd0;
                        row        <= 6'd0;
                        word       <= 3'd0;
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
                                fetch_done <= 1'b1;
                                state      <= F_IDLE;
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
