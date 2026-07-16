//-----------------------------------------------------------------------------
// row_drv_icnd1028.v — ICND1028 共阳高侧行驱子模块 (v0 = ICND3019 时序克隆)
//
// 01 文档 §2.6 row_drv 抽象: 用户确认 ICND1028 协议与 ICND3019 同族, 行链逻辑
// 从 v5 icnd2049_panel_pov 的 ICND3019 FSM 原样抽出, 时序参数改 0x24 cfg 运行时
// 可调 (换芯片不重编译):
//   cfg[7:0]  = adv_high  DCLK 高电平拍数 (0→默认 64  = 1.28us, ≥500ns 规矩)
//   cfg[15:8] = pre/hold  SDI setup/hold 拍数 (0→默认 8 = 160ns; CFG 空白=2×)
//   cfg[16]   = row_bk 极性 (v0 恒不消隐, 该位选无效电平; 引擎 OE 负责消隐)
//   cfg[17]   = row_dclk 极性翻转
//   cfg[18]   = row_lck  极性翻转
//
// 三线映射 (2026-05-27 老屏约定): row_sdi/row_dclk/row_lck ↔ 老 icnd_sdi/
// icnd_dclk/icnd_rclk (A=DCLK, B=RCLK/LCK, C=SDI)。row_bk 新增, 接口板确认前悬空。
//
// 握手: row_go 1 拍脉冲推进一行 (row_first=1 → 链头灌 '1'); man_go 手动直控
// (v5 0x08: type 0=advance 带 man_sdi, 1=config 发 man_reg+8 个 LCK 脉冲)。
// row_busy 覆盖整个推进窗 (pre+high+hold); busy 期间引擎保证 OE=1 消隐。
// T_adv 默认 = 8+64+8 = 80 拍 (01 §2.5 藏尾预算)。
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module row_drv_icnd1028 (
    input  wire        clk,
    input  wire        rst_n,
    // 引擎握手
    input  wire        row_go,      // 1 拍: 推进到下一行
    input  wire        row_first,   // 本次推进回首行 (链头灌 '1')
    // 手动直控 (v5 0x08)
    input  wire        man_go,
    input  wire        man_type,    // 0=advance 1=config
    input  wire        man_sdi,
    input  wire [3:0]  man_reg,
    // 0x24 运行时时序参数
    input  wire [31:0] cfg,
    output reg         row_busy,
    // 物理线 (行选串行链 + 锁存/消隐, 共阳高侧)
    output wire        row_sdi,
    output wire        row_dclk,
    output wire        row_lck,
    output wire        row_bk
);

    wire [8:0] T_HIGH = (cfg[7:0]  == 8'd0) ? 9'd64 : {1'b0, cfg[7:0]};
    wire [8:0] T_PH   = (cfg[15:8] == 8'd0) ? 9'd8  : {1'b0, cfg[15:8]};
    wire [8:0] T_PH2  = {T_PH[7:0], 1'b0};          // CFG 前后空白 = 2×pre

    localparam S_IDLE     = 3'd0;
    localparam S_ADV_PRE  = 3'd1;
    localparam S_ADV_HIGH = 3'd2;
    localparam S_ADV_HOLD = 3'd3;
    localparam S_CFG_PRE  = 3'd4;
    localparam S_CFG_HIGH = 3'd5;
    localparam S_CFG_LOW  = 3'd6;
    localparam S_CFG_POST = 3'd7;

    reg [2:0] st;
    reg [8:0] div;
    reg [4:0] pcnt, ptgt;
    reg       sdi_i, dclk_i, lck_i;

    always @(posedge clk) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            row_busy <= 1'b0;
            div      <= 9'd0;
            pcnt     <= 5'd0;
            ptgt     <= 5'd0;
            sdi_i    <= 1'b0;
            dclk_i   <= 1'b0;
            lck_i    <= 1'b0;
        end else begin
            case (st)
                S_IDLE: begin
                    dclk_i <= 1'b0;
                    lck_i  <= 1'b0;
                    if (man_go) begin
                        row_busy <= 1'b1;
                        div      <= 9'd0;
                        if (!man_type) begin
                            sdi_i <= man_sdi;
                            st    <= S_ADV_PRE;
                        end else begin
                            ptgt  <= man_reg + 5'd8;
                            pcnt  <= 5'd0;
                            st    <= S_CFG_PRE;
                        end
                    end else if (row_go) begin
                        row_busy <= 1'b1;
                        div      <= 9'd0;
                        sdi_i    <= row_first;      // 首行灌 '1', 其余移 '0'
                        st       <= S_ADV_PRE;
                    end
                end
                S_ADV_PRE: begin                    // SDI setup
                    if (div == T_PH - 9'd1) begin
                        div    <= 9'd0;
                        dclk_i <= 1'b1;
                        st     <= S_ADV_HIGH;
                    end else div <= div + 9'd1;
                end
                S_ADV_HIGH: begin                   // DCLK 高 (默认 64 拍 = 1.28us)
                    if (div == T_HIGH - 9'd1) begin
                        div    <= 9'd0;
                        dclk_i <= 1'b0;
                        st     <= S_ADV_HOLD;
                    end else div <= div + 9'd1;
                end
                S_ADV_HOLD: begin                   // SDI hold
                    if (div == T_PH - 9'd1) begin
                        row_busy <= 1'b0;
                        st       <= S_IDLE;
                    end else div <= div + 9'd1;
                end
                S_CFG_PRE: begin                    // 配置前空白
                    if (div == T_PH2 - 9'd1) begin
                        div   <= 9'd0;
                        lck_i <= 1'b1;
                        st    <= S_CFG_HIGH;
                    end else div <= div + 9'd1;
                end
                S_CFG_HIGH: begin
                    if (div == T_PH - 9'd1) begin
                        div   <= 9'd0;
                        lck_i <= 1'b0;
                        st    <= S_CFG_LOW;
                    end else div <= div + 9'd1;
                end
                S_CFG_LOW: begin
                    if (div == T_PH - 9'd1) begin
                        div <= 9'd0;
                        if (pcnt == ptgt - 5'd1)
                            st <= S_CFG_POST;
                        else begin
                            pcnt  <= pcnt + 5'd1;
                            lck_i <= 1'b1;
                            st    <= S_CFG_HIGH;
                        end
                    end else div <= div + 9'd1;
                end
                S_CFG_POST: begin                   // 配置后空白
                    if (div == T_PH2 - 9'd1) begin
                        row_busy <= 1'b0;
                        st       <= S_IDLE;
                    end else div <= div + 9'd1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    // 极性可翻 (0x24 cfg), datasheet 到手前不焊死
    assign row_sdi  = sdi_i ^ cfg[19];   // [19]=行选数据反相 (1028 低有效选行假设)
    assign row_dclk = dclk_i ^ cfg[17];
    assign row_lck  = lck_i  ^ cfg[18];
    assign row_bk   = cfg[16];      // v0: 恒无效 (不消隐), 极性位选电平

endmodule
