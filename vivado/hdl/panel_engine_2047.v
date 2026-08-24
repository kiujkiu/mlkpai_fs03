`timescale 1ns / 1ps
//=============================================================================
// panel_engine_2047.v — pov_dual_top 的 panel_engine 真身 (顶替 panel_engine_stub)
//
// 结构: 私有 fb (9 lane × 1024×32, 写口=top / 读口=核, 同步读 1 拍) +
//       icnd2047_panel_core (双沿引擎, pads 内含 ODDR) 。
// 端口与 panel_engine_stub 完全一致 —— top 零改动; stub/真身二选一参与编译。
//
// 语义映射 (v5 寄存器位 → 2047 核):
//   dclk_fast (0x0C sub10 [29]) → ddr_slow   ⚠ 位复用改义, 老脚本迁移见 00_overview
//   use_fb / auto_pattern / auto_disp_cyc / overlap_en → 不用 (核 fb-only, overlap 恒开)
//   手动命令 0x00/04/08 与 0x24 行驱 cfg 本版未从 top 引出: tie 0 (row_cfg=0→默认时序)
//   frame_period (0x28R) 2026-08-20 起**已引出** → 顶层 R 0x28。
//     语义 = 最近一整屏的 aclk 拍数 (1-bit: rows×195; 3-bit: rows×3×195)。
//     用途: 与 CPU 墙钟测出的整屏耗时相除 ⇒ 直接算出 aclk 真实频率, 两边都不
//     依赖对方的假设 (BD/SLCR 说 50MHz vs 实测行周期反推 25MHz 的悬案)。
//
// [2026-08-20 feature/3bit-color] 3-bit 行内 BCM:
//   fb 深度 512→1024 (BRAM 9 tile RAMB18 → 18 tile RAMB36, 140 tile 里无压力),
//   写/读口地址 9→10 bit。3-bit 时片内地址 = row*18 + plane*6 + pair (0..971),
//   由顶层写侧换算; 1-bit 时仍是 {row[5:0],pair[2:0]} (0..431), 老路径零改动。
//   oe_w1/oe_w2/bpp_mode 来自 0x0C subcmd01, 直通核。
//=============================================================================
module panel_engine #(
    parameter LANES    = 9,
    parameter DCLK_DIV = 4   // 兼容 stub 端口; 2047 核时钟由 ddr_slow 选, 本参数未用
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    input  wire        fb_we,
    input  wire [3:0]  fb_wlane,
    input  wire [9:0]  fb_waddr,
    input  wire [31:0] fb_wdata,

    input  wire        auto_en,
    input  wire        use_fb,          // 未用 (核恒 fb 源)
    input  wire [15:0] auto_pattern,    // 未用
    input  wire [19:0] auto_disp_cyc,   // 未用 (overlap 恒开, OE 窗调光)
    input  wire [8:0]  au_rows_max,
    input  wire        dclk_fast,       // → ddr_slow (位改义)
    input  wire        overlap_en,      // 未用
    input  wire [7:0]  oe_window,       // = BCM oe_w0 (plane0)
    input  wire [7:0]  oe_w1,           // 0x0C sub01 [7:0]  plane1 沿数
    input  wire [7:0]  oe_w2,           // 0x0C sub01 [15:8] plane2 沿数
    input  wire        bpp_mode,        // 0x0C sub01 [16] 0=1bit 1=3bit BCM
    input  wire        half_scan,       // 0x0C sub01 [18] 1=只发 96bit (屏高减半, 角分辨率翻倍)
    input  wire        le_plane_mode,   // 0x0C sub01 [17] plane1/2 LE: 0=3沿 1=同plane0
    input  wire [8:0]  sdi_mask,
    input  wire        oe_set_pulse,
    input  wire        oe_set_val,
    input  wire [31:0] row_cfg,          // 0x24 行驱时序/极性 (0=全默认)
    output wire        engine_busy,
    output wire        oe_state,        // OE fabric 镜像 (status 读回用, 别读 pad)
    output wire [31:0] frame_period,    // 最近一整屏 aclk 拍数 (顶层 R 0x28)

    output wire        dclk_out,
    output wire        le_out,
    output wire        oe_out,
    output wire [8:0]  sdi_out,
    output wire        icnd_sdi_out,
    output wire        icnd_dclk_out,
    output wire        icnd_rclk_out
);

    //---------- 私有 fb: 写=top, 读=核 (同步 1 拍, 9 lane 并读) ----------
    wire [9:0]   fb_raddr;
    wire [287:0] fb_dout_flat;
    genvar gi;
    generate for (gi = 0; gi < LANES; gi = gi + 1) begin: g_fb
        reg [31:0] mem [0:1023];
        reg [31:0] dout;
        always @(posedge clk) begin
            if (fb_we && fb_wlane == gi[3:0]) mem[fb_waddr] <= fb_wdata;
            dout <= mem[fb_raddr];
        end
        assign fb_dout_flat[gi*32 +: 32] = dout;
    end endgenerate

    //---------- 2047 双沿引擎核 ----------
    wire core_busy, core_row_busy;
    icnd2047_panel_core u_core (
        .aclk            (clk),
        .aresetn         (rst_n && enable),      // enable=0 时复位核 → pads 消隐
        .auto_en         (auto_en && enable),
        .rows            (au_rows_max + 9'd1),   // v5 语义 max 行号 → 核要行数 (写54存53, +1还原)
        .oe_window       (oe_window),
        .oe_w1           (oe_w1),
        .oe_w2           (oe_w2),
        .bpp_mode        (bpp_mode),
        .half_scan       (half_scan),
        .le_plane_mode   (le_plane_mode),
        .ddr_slow        (dclk_fast),            // 位改义: 1=25Mbps 降级
        .oe_set_pulse    (oe_set_pulse && enable),
        .oe_set_val      (oe_set_val),
        .sdi_mask        (sdi_mask),
        .fb_raddr        (fb_raddr),
        .fb_dout_flat    (fb_dout_flat),
        .cmd_start       (1'b0),
        .cmd_data        (16'h0),
        .cmd_le          (7'd0),
        .cmd_mode        (2'b00),
        .cmd_burst       (16'h0),
        .chain_data_flat (144'h0),
        .row_man_go      (1'b0),
        .row_man_type    (1'b0),
        .row_man_sdi     (1'b0),
        .row_man_reg     (4'h0),
        .row_cfg         (row_cfg),
        .busy            (core_busy),
        .cmd_pending_o   (),
        .row_busy_o      (core_row_busy),
        .eg_state_o      (),
        .shift_row_o     (),
        .plane_o         (),
        .frame_count_o   (),
        .frame_period_o  (frame_period),
        .oe_done_o       (),
        .oe_state_o      (oe_state),
        .adv_fired_o     (),
        .dclk_pad        (dclk_out),
        .le_pad          (le_out),
        .oe_pad          (oe_out),
        .sdi_pad         (sdi_out),
        .row_sdi         (icnd_sdi_out),
        .row_dclk        (icnd_dclk_out),
        .row_lck         (icnd_rclk_out)
    );

    assign engine_busy = (auto_en && enable) | core_busy | core_row_busy;

endmodule
