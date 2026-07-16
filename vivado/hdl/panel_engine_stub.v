//==============================================================================
// panel_engine_stub.v — pov_dual_top 的列驱引擎占位 (仅 xsim 用)
//
// ⚠ 这是 stub, 不是真引擎. 真引擎 icnd2047_panel_core (双沿列驱, 另一 agent
//   在做) 定稿后, 按本模块端口表包一层 (或直接同名同口) 替换, 本文件退出编译.
//   端口表 = pov_dual_top ↔ 引擎的接口契约 (语义 = v5 icnd2049 引擎的
//   fb 写口 + 控制口, 见 icnd2049_panel_pov.v 头注):
//
//   enable        : 0 = 静默消隐 (oe=1, 所有输出静止). 顶层给 B 屏接
//                   dual_en|fb_sel_b, A 屏恒 1.
//   fb 写口       : 9 lane × 512 × 32bit 私有 BRAM, {row[5:0],word[2:0]} 编址,
//                   与 ddr_slice_fetch fb 口/v5 AXI fb 窗完全同构.
//   auto_en/use_fb/auto_pattern/auto_disp_cyc/au_rows_max : v5 0x0C subcmd11
//   dclk_fast/overlap_en/oe_window : v5 0x0C subcmd10 cfg_we 组
//                   (2047 引擎可自行忽略不适用位, 如 dclk_fast)
//   sdi_mask      : v5 0x0C subcmd00, per-chain enable / 颜色选择
//   oe_set_pulse/oe_set_val : 手动 OE (auto 停时)
//   engine_busy   : 引擎在扫 (顶层回读 0x00[0]/[12])
//
// stub 行为 (够 TB 用即可):
//   * fb BRAM 真实现 (TB 层级引用抽查取帧落数);
//   * enable&&auto_en 时输出脚给点活动 (TB 区分 A 活 B 静), 波形无协议意义;
//   * enable=0 → oe=1/sdi=0/时钟脚静止 (dual_en=0 时 B 屏消隐判据).
//==============================================================================
`timescale 1ns / 1ps

module panel_engine #(
    parameter DCLK_DIV = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    // fb 写口 (取帧引擎 / AXI fb 窗经顶层仲裁后灌入)
    input  wire        fb_we,
    input  wire [3:0]  fb_wlane,
    input  wire [8:0]  fb_waddr,
    input  wire [31:0] fb_wdata,

    // 控制口 (v5 语义)
    input  wire        auto_en,
    input  wire        use_fb,
    input  wire [15:0] auto_pattern,
    input  wire [19:0] auto_disp_cyc,
    input  wire [8:0]  au_rows_max,
    input  wire        dclk_fast,
    input  wire        overlap_en,
    input  wire [7:0]  oe_window,
    input  wire [8:0]  sdi_mask,
    input  wire        oe_set_pulse,
    input  wire        oe_set_val,
    input  wire [31:0] row_cfg,
    output wire        engine_busy,
    output wire        oe_state,

    // 屏引脚
    output reg         dclk_out,
    output reg         le_out,
    output reg         oe_out,
    output reg  [8:0]  sdi_out,
    output reg         icnd_sdi_out,
    output reg         icnd_dclk_out,
    output reg         icnd_rclk_out
);

    //---------- 私有 fb: 9 lane × 512 × 32bit (真引擎同构) ----------
    genvar gi;
    generate for (gi = 0; gi < 9; gi = gi + 1) begin: g_fb
        reg [31:0] mem [0:511];
        always @(posedge clk)
            if (fb_we && fb_wlane == gi[3:0]) mem[fb_waddr] <= fb_wdata;
    end endgenerate

    assign oe_state = oe_out;
    wire run = enable && auto_en;
    assign engine_busy = run;

    //---------- 占位活动 (无协议意义, 只为 TB 区分活/静) ----------
    reg [15:0] scan;
    always @(posedge clk) begin
        if (!rst_n) begin
            scan          <= 16'd0;
            dclk_out      <= 1'b0;
            le_out        <= 1'b0;
            oe_out        <= 1'b1;      // 复位消隐 (v5 同)
            sdi_out       <= 9'd0;
            icnd_sdi_out  <= 1'b0;
            icnd_dclk_out <= 1'b0;
            icnd_rclk_out <= 1'b0;
        end else if (run) begin
            scan          <= scan + 16'd1;
            dclk_out      <= scan[0];
            le_out        <= (scan[7:0] == 8'hFF);
            sdi_out       <= {9{scan[4]}} & sdi_mask;
            oe_out        <= (scan[7:0] >= oe_window);   // 占空 ~oe_window/256
            icnd_dclk_out <= scan[9];
            icnd_sdi_out  <= scan[10];
            icnd_rclk_out <= 1'b0;
        end else begin
            dclk_out      <= 1'b0;
            le_out        <= 1'b0;
            sdi_out       <= 9'd0;
            icnd_sdi_out  <= 1'b0;
            icnd_dclk_out <= 1'b0;
            icnd_rclk_out <= 1'b0;
            if (enable && oe_set_pulse) oe_out <= oe_set_val;  // 手动 OE
            else if (!enable)           oe_out <= 1'b1;        // 静默消隐
        end
    end

endmodule
