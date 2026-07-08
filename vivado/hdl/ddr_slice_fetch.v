//==============================================================================
// ddr_slice_fetch.v — AXI4 read-master slice-frame fetch engine
//
// 从 DDR 自主拉取一个 slice 帧 (11664 B = 9 lane x 54 row x 6 word x 4 B),
// 按 lane-major 线性布局换算出 icnd2049_panel_fb 的 fb 写口序列:
//
//   DDR byte offset = lane*1296 + row*24 + word*4   (lane 0..8, row 0..53, word 0..5)
//   fb_wlane = lane[3:0],  fb_waddr = {row[5:0], word[2:0]},  fb_wdata = 32-bit word
//
// 取帧起始地址 = slice_base + slice_idx * 0x3000 (slice_idx 0..359).
//
// AXI 读通道套路移植自 zynq_pov hub75e_panel_seq_v6.v 的 DMA FSM:
//   - 单 outstanding: 一次只挂一个 AR, 收完整 burst (rlast) 再发下一个
//   - arvalid 置位后 hold 到 arready (AXI 规定不能撤)
//   - INCR burst, arsize=2 (4B/beat), arcache=0011, RRESP 错误容忍 (计数不卡死)
// 区别: v6 是 64KB/2^n 布局用固定 256-beat burst; 本模块 1296B/lane 非 2^n,
//   全程线性 beat 计数, 每 burst <=16 beats 且动态截断保证不跨 4KB 边界.
//
// 握手: fetch_go 1 拍脉冲启动 (busy 期间忽略), 锁存 slice_idx;
//       fetch_busy 电平, fetch_done 1 拍脉冲.
//
// 时钟 50 MHz, 时序余量大, 写法直白优先.
//==============================================================================
`timescale 1ns / 1ps

module ddr_slice_fetch #(
    parameter [31:0] SLICE_STRIDE = 32'h0000_3000,  // 每 slice 字节跨度
    parameter [11:0] TOTAL_WORDS  = 12'd2916        // 11664 B / 4
)(
    input  wire        aclk,
    input  wire        aresetn,

    //-------------------------------------------------------------------
    // AXI4 master 读通道 (32-bit data, ID 省略 = 恒 0)
    //-------------------------------------------------------------------
    output wire [31:0] m_axi_araddr,
    output reg  [7:0]  m_axi_arlen,     // 动态 0..15 (<=16 beats)
    output wire [2:0]  m_axi_arsize,    // 固定 2 = 4 byte/beat
    output wire [1:0]  m_axi_arburst,   // 固定 INCR
    output wire        m_axi_arlock,
    output wire [3:0]  m_axi_arcache,   // 4'b0011 normal non-cacheable bufferable
    output wire [2:0]  m_axi_arprot,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,

    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,

    //-------------------------------------------------------------------
    // 控制
    //-------------------------------------------------------------------
    input  wire [31:0] slice_base,      // DDR 帧集基址
    input  wire [8:0]  slice_idx,       // 0..359, fetch_go 时锁存
    input  wire        fetch_go,        // 1 拍脉冲
    output wire        fetch_busy,
    output reg         fetch_done,      // 1 拍脉冲

    //-------------------------------------------------------------------
    // BRAM 写侧 (对齐 icnd2049_panel_fb 的 fb 写口)
    //-------------------------------------------------------------------
    output reg         fb_we,
    output reg  [3:0]  fb_wlane,
    output reg  [8:0]  fb_waddr,        // {row[5:0], word[2:0]}
    output reg  [31:0] fb_wdata
);

    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;

    //-------------------------------------------------------------------
    // FSM
    //   F_IDLE : 等 fetch_go → 锁存起始地址/清计数 → F_CALC
    //   F_CALC : 由当前地址算本 burst 长度 (min(16, 剩余, 到 4KB 边界)),
    //            置 arvalid → F_AR
    //   F_AR   : hold arvalid 到 arready → F_R
    //   F_R    : 逐 beat 收数写 fb 口. rlast 时: 还有剩余 → F_CALC 续发;
    //            收完 → fetch_done 脉冲 → F_IDLE
    //-------------------------------------------------------------------
    localparam [1:0]
        F_IDLE = 2'd0,
        F_CALC = 2'd1,
        F_AR   = 2'd2,
        F_R    = 2'd3;

    reg [1:0]  state;
    reg [31:0] cur_addr;        // 每 beat +4, rlast 后即下一 burst 起始地址
    reg [11:0] words_left;      // 剩余 beat 数 (含未发 AR 的)
    reg [3:0]  lane;            // 0..8
    reg [5:0]  row;             // 0..53
    reg [2:0]  word;            // 0..5
    reg [3:0]  err_cnt;         // RRESP 非 OKAY 饱和计数 (容忍不卡死)

    assign m_axi_araddr = cur_addr;
    assign m_axi_rready = (state == F_R);
    assign fetch_busy   = (state != F_IDLE);

    wire beat = m_axi_rvalid && m_axi_rready;

    // burst 长度: min(16, words_left, 到下一 4KB 边界的 beat 数)
    // cur_addr 恒 4B 对齐, beats_to_4k 范围 1..1024
    wire [10:0] beats_to_4k = 11'd1024 - {1'b0, cur_addr[11:2]};
    wire [11:0] lim_words   = (words_left < 12'd16) ? words_left : 12'd16;
    wire [11:0] burst_beats = ({1'b0, beats_to_4k} < lim_words)
                              ? {1'b0, beats_to_4k} : lim_words;  // 1..16

    always @(posedge aclk) begin
        if (!aresetn) begin
            state         <= F_IDLE;
            cur_addr      <= 32'd0;
            words_left    <= 12'd0;
            lane          <= 4'd0;
            row           <= 6'd0;
            word          <= 3'd0;
            err_cnt       <= 4'd0;
            m_axi_arvalid <= 1'b0;
            m_axi_arlen   <= 8'd0;
            fetch_done    <= 1'b0;
            fb_we         <= 1'b0;
            fb_wlane      <= 4'd0;
            fb_waddr      <= 9'd0;
            fb_wdata      <= 32'd0;
        end else begin
            fetch_done <= 1'b0;     // 默认: done 只 1 拍
            fb_we      <= 1'b0;     // 默认: 每个 beat 单拍写

            // RRESP 错误饱和计数, 流程照走 (v6 套路)
            if (beat && m_axi_rresp[1] && err_cnt != 4'hF)
                err_cnt <= err_cnt + 4'd1;

            case (state)
                F_IDLE: begin
                    if (fetch_go) begin
                        // 锁存: 起始地址 = slice_base + slice_idx*0x3000
                        // (0x3000 = <<13 + <<12, 综合成两个加法器)
                        cur_addr   <= slice_base
                                      + {10'b0, slice_idx, 13'b0}
                                      + {11'b0, slice_idx, 12'b0};
                        words_left <= TOTAL_WORDS;
                        lane       <= 4'd0;
                        row        <= 6'd0;
                        word       <= 3'd0;
                        state      <= F_CALC;
                    end
                end

                F_CALC: begin
                    m_axi_arlen   <= burst_beats[7:0] - 8'd1;
                    m_axi_arvalid <= 1'b1;
                    state         <= F_AR;
                end

                F_AR: begin
                    // arvalid 已置位, hold 到 arready
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        state         <= F_R;
                    end
                end

                F_R: begin
                    if (beat) begin
                        // 收 1 beat → 写 fb 口 (寄存 1 拍输出)
                        fb_we    <= 1'b1;
                        fb_wlane <= lane;
                        fb_waddr <= {row, word};
                        fb_wdata <= m_axi_rdata;

                        // 线性计数换算 (lane, row, word)
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
                                state <= F_CALC;    // cur_addr 已指向下 burst
                            end
                        end
                    end
                end

                default: state <= F_IDLE;
            endcase
        end
    end

endmodule
