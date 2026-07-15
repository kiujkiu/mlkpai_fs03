`timescale 1ns / 1ps
// angle_tracker — 从 icnd2049_panel_pov.v (v5, 含 2026-06-15 两 bug 修复) 原样抽出
// 供 v6 pov_dual_top 复用; v5 工程仍用大文件内定义, 两文件不同工程互斥使用
module angle_tracker #(
    parameter integer CLK_HZ = 50000000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sensor_in,       // 光电, 1 脉冲/圈, 异步
    input  wire        fake_en,         // 1 = 无传感器, 按 fake_period 自由跑
    input  wire [31:0] fake_period,     // fake 模式: 每 slice aclk 数
    input  wire [15:0] n_slices,        // 每圈切片数
    output reg  [15:0] slice_idx,       // 当前切片 0 .. n_slices-1
    output reg  [31:0] rev_period,      // 实测每圈周期 (debug)
    output reg         locked           // 连续两圈周期稳定
);
    localparam integer DEBOUNCE_CYC = CLK_HZ / 1000000;   // 1 us 去抖
    localparam [31:0]  MIN_REV_CYC  = CLK_HZ / 1000;      // <1ms 的"圈"当毛刺

    // 2FF 同步 + 去抖 + 上升沿
    reg  [1:0] sync_ff;
    reg        sens_clean, sens_clean_d;
    reg  [7:0] db_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_ff <= 2'b00; sens_clean <= 1'b0; db_cnt <= 8'd0;
        end else begin
            sync_ff <= {sync_ff[0], sensor_in};
            if (sync_ff[1] == sens_clean)           db_cnt <= 8'd0;
            else if (db_cnt == DEBOUNCE_CYC[7:0]) begin
                sens_clean <= sync_ff[1]; db_cnt <= 8'd0;
            end else                                 db_cnt <= db_cnt + 8'd1;
        end
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) sens_clean_d <= 1'b0; else sens_clean_d <= sens_clean;
    wire sens_rise = sens_clean & ~sens_clean_d;

    // 圈周期测量
    reg  [31:0] rev_cnt, prev_period;
    reg         have_pulse, div_start;
    wire        pulse_ok = sens_rise && (rev_cnt >= MIN_REV_CYC);
    wire [31:0] diff_ab  = (rev_cnt > prev_period) ? (rev_cnt - prev_period)
                                                   : (prev_period - rev_cnt);
    wire        stable   = have_pulse && (diff_ab < (prev_period >> 3));
    wire        stalled  = have_pulse && (rev_cnt > {rev_period[30:0], 1'b0});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rev_cnt <= 32'd0; rev_period <= 32'hFFFF_FFFF;
            prev_period <= 32'd0; have_pulse <= 1'b0; div_start <= 1'b0;
        end else begin
            div_start <= 1'b0;
            if (pulse_ok) begin
                rev_cnt     <= 32'd1;
                prev_period <= rev_cnt;
                have_pulse  <= 1'b1;
                rev_period  <= rev_cnt;      // bug2 修: 无条件更新
                div_start   <= 1'b1;
            end else if (rev_cnt != 32'hFFFF_FFFF)
                rev_cnt <= rev_cnt + 32'd1;
        end
    end

    // 串行除法: slice_period = rev_period / n_slices (32 拍)
    reg  [31:0] div_quot, div_dividend, slice_period;
    reg  [47:0] div_rem;
    reg  [5:0]  div_bit;
    reg         div_busy, slice_period_valid;
    wire [47:0] div_try = {div_rem[46:0], div_dividend[31]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_busy <= 1'b0; div_quot <= 32'd0; div_rem <= 48'd0;
            div_dividend <= 32'd0; div_bit <= 6'd0;
            slice_period <= 32'hFFFF_FFFF; slice_period_valid <= 1'b0;
        end else if (div_start && (n_slices != 16'd0)) begin
            div_busy     <= 1'b1;
            div_dividend <= rev_period;      // bug1 修: 用锁存周期, 不抓被复位的 rev_cnt
            div_rem <= 48'd0; div_quot <= 32'd0; div_bit <= 6'd0;
        end else if (div_busy) begin
            div_dividend <= {div_dividend[30:0], 1'b0};
            if (div_try >= {32'd0, n_slices}) begin
                div_rem <= div_try - {32'd0, n_slices};
                div_quot <= {div_quot[30:0], 1'b1};
            end else begin
                div_rem <= div_try;
                div_quot <= {div_quot[30:0], 1'b0};
            end
            if (div_bit == 6'd31) div_busy <= 1'b0;
            div_bit <= div_bit + 6'd1;
        end else if (div_bit == 6'd32) begin
            div_bit <= 6'd0;
            if (div_quot >= 32'd2) begin
                slice_period       <= div_quot;
                slice_period_valid <= 1'b1;
            end
        end
    end

    // slice 插值: 累加计拍, 每 slice_period 步进; 脉冲硬回零
    reg  [31:0] acc;
    wire [31:0] cur_step = fake_en ? fake_period : slice_period;
    wire        track_en = fake_en | (slice_period_valid & ~stalled);
    wire [15:0] last_idx = n_slices - 16'd1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slice_idx <= 16'd0; acc <= 32'd0; locked <= 1'b0;
        end else if (fake_en) begin
            locked <= 1'b1;
            if (acc + 32'd1 >= cur_step) begin
                acc       <= acc + 32'd1 - cur_step;
                slice_idx <= (slice_idx >= last_idx) ? 16'd0 : slice_idx + 16'd1;
            end else acc <= acc + 32'd1;
        end else begin
            if (pulse_ok) begin
                slice_idx <= 16'd0;
                acc       <= 32'd0;
                locked    <= stable & slice_period_valid;
            end else begin
                if (stalled) locked <= 1'b0;
                if (track_en) begin
                    if (acc + 32'd1 >= cur_step) begin
                        acc       <= acc + 32'd1 - cur_step;
                        slice_idx <= (slice_idx >= last_idx) ? 16'd0 : slice_idx + 16'd1;
                    end else acc <= acc + 32'd1;
                end
            end
        end
    end
endmodule
