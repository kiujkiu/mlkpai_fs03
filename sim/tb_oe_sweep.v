//-----------------------------------------------------------------------------
// tb_oe_sweep.v — 扫 oe_window 实测整屏刷新周期 (2026-07-29)
//
// 起因: 板上 0x28 frame_period 在 panel_engine_2047.v 里**悬空未引出**
// (`.frame_period_o ()`), 读到的是噪声; 而 "LWAIT = oe_window - 111" 只是从
// TB 两个点外推的猜测 —— LWAIT 实际是等 disp_ready (上一行 OE 显示窗收完),
// 不是固定公式。故直接仿真扫一遍拿真值。
//
// 输出: 每档 oe 的 frame_period(拍) / 行周期 / 整屏时间 / 面板刷新率,
//       并按当前实测转速 16.22 rps 给出"每片能扫几遍"。
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_oe_sweep;
    localparam real    FCLK = 50.0e6;
    localparam integer ROWS = 54;
    localparam real    RPS  = 16.22;      // 板上实测转速

    reg clk = 0, rstn = 0;
    always #10 clk = ~clk;                // 50 MHz

    reg  [7:0] oe_window = 8'd48;
    reg        auto_en   = 1'b0;
    reg  [8:0] rows      = 9'd54;
    reg  [8:0] sdi_mask  = 9'h1FF;

    wire [8:0]   fb_raddr;
    wire [287:0] fb_dout_flat = {9{32'hA5A5_5A5A}};   // 只关心时序, 内容任意

    wire        busy, cmd_pending_o, row_busy_o, oe_done_o, adv_fired_o;
    wire [2:0]  eg_state_o;
    wire [5:0]  shift_row_o;
    wire [15:0] frame_count_o;
    wire [31:0] frame_period_o;
    wire        dclk_pad, le_pad, oe_pad;
    wire [8:0]  sdi_pad;
    wire        row_sdi, row_dclk, row_lck, row_bk;

    icnd2047_panel_core dut (
        .aclk(clk), .aresetn(rstn),
        .auto_en(auto_en), .rows(rows), .oe_window(oe_window),
        .ddr_slow(1'b0), .oe_set_pulse(1'b0), .oe_set_val(1'b0),
        .sdi_mask(sdi_mask),
        .fb_raddr(fb_raddr), .fb_dout_flat(fb_dout_flat),
        .cmd_start(1'b0), .cmd_data(16'd0), .cmd_le(7'd0),
        .cmd_mode(2'd0), .cmd_burst(16'd0),
        .chain_data_flat(144'd0),
        .row_man_go(1'b0), .row_man_type(1'b0),
        .row_man_sdi(1'b0), .row_man_reg(4'd0),
        .row_cfg(32'd0),
        .busy(busy), .cmd_pending_o(cmd_pending_o), .row_busy_o(row_busy_o),
        .eg_state_o(eg_state_o), .shift_row_o(shift_row_o),
        .frame_count_o(frame_count_o), .frame_period_o(frame_period_o),
        .oe_done_o(oe_done_o), .adv_fired_o(adv_fired_o),
        .dclk_pad(dclk_pad), .le_pad(le_pad), .oe_pad(oe_pad), .sdi_pad(sdi_pad),
        .row_sdi(row_sdi), .row_dclk(row_dclk), .row_lck(row_lck), .row_bk(row_bk)
    );

    integer   i;
    real      row_p, frame_us, refresh, slice_us, scans;
    reg [7:0] OE_LIST [0:7];
    reg [15:0] fc0;

    initial begin
        OE_LIST[0]=8'd8;   OE_LIST[1]=8'd24;  OE_LIST[2]=8'd48;  OE_LIST[3]=8'd96;
        OE_LIST[4]=8'd128; OE_LIST[5]=8'd160; OE_LIST[6]=8'd187; OE_LIST[7]=8'd200;

        rstn = 0; repeat (20) @(posedge clk); rstn = 1;
        repeat (20) @(posedge clk);

        slice_us = 1.0e6 / (RPS * 360.0);            // 每片时间预算

        $display("");
        $display("每片预算 %.1f us  (%.2f rps x 360 片)", slice_us, RPS);
        $display("");
        $display(" oe | frame_period |  行周期 | 整屏时间  | 面板刷新率 | 每片可扫");
        $display("----+--------------+---------+-----------+------------+---------");

        for (i = 0; i < 8; i = i + 1) begin
            oe_window = OE_LIST[i];
            auto_en   = 1'b1;
            fc0 = frame_count_o;
            // 跑满 4 整屏后取稳态 frame_period
            wait (frame_count_o >= fc0 + 4);
            repeat (5) @(posedge clk);

            row_p    = frame_period_o / (1.0 * ROWS);
            frame_us = frame_period_o / FCLK * 1.0e6;
            refresh  = FCLK / frame_period_o;
            scans    = slice_us / frame_us;
            $display(" %3d|    %6d    | %6.1f  | %7.1fus | %8.0f Hz | %5.2f %s",
                     OE_LIST[i], frame_period_o, row_p, frame_us, refresh, scans,
                     (scans >= 1.0) ? "OK" : "<-- 扫不完");

            auto_en = 1'b0;
            repeat (600) @(posedge clk);
        end
        $display("");
        $display("=== OE SWEEP DONE ===");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
