//-----------------------------------------------------------------------------
// icnd2049_panel_seq.v - P0.9375 COB 屏 (ICND2049 列驱 ×108 + ICND3019 行驱 ×24)
//
// Port 自 zynq_pov led_panel_seq.v (ICND1069 版, 2026-05), 2026-07-07 改 ICND2049:
//  * DCLK 门控: 1069 要求 free-running DCLK, 2049 相反 — 空闲期的 DCLK 边沿会把 0
//    移进移位链破坏未锁存数据 (xsdb 慢速逐字灌必坏)。只在移位/marker 期间出 DCLK。
//    命令从 idle 启动时立即驱第一 bit (DCLK 低), 半周期后第一个上升沿采样, 保证 setup。
//  * 老 row_out (1069 ROW/换行, 线名 GCLK) → 该线在 2049 屏上是 OE: 改为寄存器电平
//    控制 oe_out, 复位默认 1 (消隐)。marker_ROW 模式删除。
//  * ICND3019 行驱 FSM 原样保留 (同款芯片)。时序常数按 50 MHz 重算仍全部满足:
//    ADV DCLK HIGH 64 cyc = 1.28us (≥500ns), setup/hold 8 cyc = 160ns (≥20ns),
//    CFG 空白 16 cyc = 320ns (≥100ns), RCLK 高/低 8 cyc = 160ns (~100ns)。
//
// 寄存器 (s_axi, 4KB, 建议基址 0x40010000):
//   0x00 CMD (W: trigger; R: status)
//     [15:0]   data word (MSB first)
//     [22:16]  le_count — word 模式: 最后 le_count 拍 LE 高 (数据锁存用 1;
//              2049 指令 = LE 覆盖 N 个 CLK 上升沿, 用 marker_LE 或加大 le_count)
//     [25:24]  mode: 00=broadcast word (16 DCLK, 9 路同数据)
//                    01=marker_LE (le_count 拍 DCLK, LE 高, SDI=0)
//                    10=保留 (按 word 处理)
//                    11=per-chain word (9 路各自 chain_data[i])
//   0x00 read: [0]=busy [1]=cmd_pending [2]=icnd_busy [3]=oe_reg
//   0x04 BURST (W): 下一个 0x00 命令自动重发 N 次 (总 N+1 次), 快速整屏填充
//   0x08 ICND3019 (W): [31]=type 0=advance/1=config, [0]=SDI(advance), [3:0]=reg(config)
//   0x0C 杂项 (W): [31:30]=subcmd
//     00 → sdi_mask = wdata[8:0] (per-chain enable, 默认全 1; auto 模式=颜色选择)
//     01 → chain_data[wdata[19:16]] = wdata[15:0] (per-chain 模式数据预载)
//     10 → oe_reg = wdata[0] (0=显示, 1=消隐; 复位默认 1);
//          wdata[24:16] = auto 扫描行数 (非 0 时更新, 默认 384; 设成真实行数提亮度)
//          wdata[27]=cfg_we (v4): 为 1 时更新 [29]=dclk_fast (0=12.5M/1=25M),
//            [28]=overlap_en; wdata[15:8]=oe_window (DCLK 数, 非 0 更新, 默认 48)
//            — 旧脚本 [27]=0 不受影响。⚠ dclk_fast 只在 auto 停止时切
//     11 → auto 自主扫描: [0]=auto_en, [23:8]=auto_pattern (每芯片 16bit 图形),
//          [29:24]=disp 窗 (单位 1024 aclk, 0 当 3 用 → 默认 ~61us)
//          auto 模式: PL 自己循环 灌12词(LE=4/首行5)→3019走行(384位单'1')→OE低显示,
//          ~130Hz 帧率, 无需 ARM/JTAG 持续喂
//
// v4 overlap 模式 (2026-07-08): 显示行 N (OE 低 oe_window 个 DCLK) 的同时移入行
//   N+1 (2049 双缓存, latch1→reg2 转移在 OE 下降沿)。行周期 = max(192 DCLK 移位,
//   oe_window) + 3019 行推进死区; 25M+overlap ≈ 1.9 kHz @54 行。亮度 = oe_window/192。
//
// 时钟: s_axi_aclk = FCLK0 50 MHz, DCLK = dclk_fast ? 25M : 12.5M (2049 max 25M)
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module icnd2049_panel_fb #(
    parameter DCLK_DIV = 4    // 50 MHz / 4 = 12.5 MHz DCLK, 2/2 = 50% 占空比
)(
    // AXI-Lite slave
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

    // ICND2049 列驱 (经屏内 74HC245 扇出到 9 组)
    output reg         dclk_out,
    output reg         le_out,
    output reg         oe_out,    // 屏连接器 "GCLK" 线 = 2049 OE, 1=消隐
    output reg [8:0]   sdi_out,   // 9-way: [0]=R1 [1]=G1 [2]=B1 [3]=R2 [4]=G2 [5]=B2 [6]=R3 [7]=G3 [8]=B3

    // ICND3019 行驱 (A=DCLK, B=RCLK, C=SDI, 2026-05-27 老屏实测约定)
    output reg         icnd_sdi_out,
    output reg         icnd_dclk_out,
    output reg         icnd_rclk_out
);

    // v4: 运行时分频 — dclk_fast=0: aclk/4=12.5M (HALF=1), =1: aclk/2=25M (HALF=0)
    reg  dclk_fast;
    wire [7:0] HALF = dclk_fast ? 8'd0 : (DCLK_DIV/2) - 1;

    //---------- Sequencer state (声明提前, divider 要用 bits_left) ----------
    reg        busy;
    reg [15:0] data_shift;
    reg [6:0]  bits_left;
    reg [6:0]  le_count_reg;
    reg [1:0]  mode_reg;
    reg [15:0] data_latched;
    reg [1:0]  mode_latched;
    reg [6:0]  le_latched;
    reg [15:0] burst_left;

    reg        cmd_pending;
    reg [15:0] pending_data;
    reg [1:0]  pending_mode;
    reg [6:0]  pending_le;
    reg [15:0] pending_burst;

    reg [15:0] chain_data [0:8];           // ARM 预写, 9 chain 各自 16-bit
    reg [15:0] pending_chain_data [0:8];   // snapshot at start_pulse (race-free)
    reg [15:0] chain_shift [0:8];

    //---------- 门控 DCLK divider ----------
    // 只在移位/marker 期间跑 (bits_left != 0); idle 时 DCLK 保持低、计数清零。
    // 命令启动 (idle→load) 时 SDI 已就位且 dclk=0/div=0 → HALF+1 拍后第一个
    // 上升沿采样第一 bit, setup = 半个 DCLK 周期 ✓
    wire seq_active = (bits_left != 7'd0);
    reg [7:0] div_count;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            div_count <= 8'b0;
            dclk_out  <= 1'b0;
        end else if (seq_active) begin
            if (div_count == HALF) begin
                div_count <= 8'b0;
                dclk_out  <= ~dclk_out;
            end else begin
                div_count <= div_count + 1;
            end
        end else begin
            div_count <= 8'b0;
            dclk_out  <= 1'b0;
        end
    end

    // 在 dclk 即将从 1 翻 0 的那拍触发: 当拍换 SDI/LE, 与 dclk 翻 0 同步
    // → 半个 DCLK 周期 stable → 下个上升沿采样 ✓ (与老设计相同)
    wire dclk_will_fall = seq_active && (div_count == HALF) && dclk_out;

    //---------- ICND3019 sub-FSM state ----------
    reg        icnd_busy;
    reg [2:0]  icnd_state;
    reg [6:0]  icnd_div;
    reg [4:0]  icnd_pulse_count;
    reg [4:0]  icnd_pulse_target;
    reg        icnd_start_pulse;
    reg        icnd_pending_type;
    reg        icnd_pending_sdi;
    reg [3:0]  icnd_pending_reg;

    //---------- AXI-Lite WRITE FSM ----------
    reg start_pulse;
    reg [15:0] burst_reg;
    reg [8:0]  sdi_mask;
    reg        oe_set_pulse;
    reg        oe_set_val;
    reg        fb_we;
    reg [3:0]  fb_wlane;
    reg [8:0]  fb_waddr;
    reg [31:0] fb_wdata;
    reg        use_fb;
    reg        auto_en;
    reg [15:0] auto_pattern;
    reg [19:0] auto_disp_cyc;   // 显示窗 aclk 数 (非 overlap 模式)
    reg [8:0]  au_rows_max;     // 扫描行数-1
    reg        overlap_en;      // v4: 显示与下一行移位重叠
    reg [7:0]  oe_window;       // v4: overlap 模式 OE 低窗口, 单位 DCLK

    integer ci;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready     <= 1'b0;
            s_axi_wready      <= 1'b0;
            s_axi_bvalid      <= 1'b0;
            s_axi_bresp       <= 2'b00;
            start_pulse       <= 1'b0;
            burst_reg         <= 16'b0;
            icnd_start_pulse  <= 1'b0;
            icnd_pending_type <= 1'b0;
            icnd_pending_sdi  <= 1'b0;
            icnd_pending_reg  <= 4'b0;
            sdi_mask          <= 9'b111_111_111;
            oe_set_pulse      <= 1'b0;
            oe_set_val        <= 1'b1;
            fb_we             <= 1'b0;
            fb_wlane          <= 4'd0;
            fb_waddr          <= 9'd0;
            fb_wdata          <= 32'd0;
            use_fb            <= 1'b0;
            auto_en           <= 1'b0;
            auto_pattern      <= 16'h8000;   // 每芯片 1 点; 9 路全开单行 ~1.6A 安全
            auto_disp_cyc     <= 20'd3072;
            au_rows_max       <= 9'd383;
            dclk_fast         <= 1'b0;
            overlap_en        <= 1'b0;
            oe_window         <= 8'd48;      // 192 的 1/4
            for (ci = 0; ci < 9; ci = ci + 1)
                chain_data[ci] <= 16'b0;
        end else begin
            start_pulse      <= 1'b0;
            icnd_start_pulse <= 1'b0;
            oe_set_pulse     <= 1'b0;
            fb_we            <= 1'b0;
            if (!s_axi_awready && !s_axi_wready &&
                s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                if (s_axi_awaddr[15]) begin
                    // fb 窗口写: [14:11]=lane, [10:5]=row, [4:2]=pair
                    fb_we    <= 1'b1;
                    fb_wlane <= s_axi_awaddr[14:11];
                    fb_waddr <= s_axi_awaddr[10:2];
                    fb_wdata <= s_axi_wdata;
                end else if (s_axi_awaddr[3:2] == 2'b00 && !cmd_pending) begin
                    start_pulse <= 1'b1;
                end else if (s_axi_awaddr[3:2] == 2'b01) begin
                    burst_reg <= s_axi_wdata[15:0];
                end else if (s_axi_awaddr[3:2] == 2'b10 && !icnd_busy) begin
                    icnd_start_pulse  <= 1'b1;
                    icnd_pending_type <= s_axi_wdata[31];
                    icnd_pending_sdi  <= s_axi_wdata[0];
                    icnd_pending_reg  <= s_axi_wdata[3:0];
                end else if (s_axi_awaddr[3:2] == 2'b11) begin
                    if (s_axi_wdata[31:30] == 2'b00) begin
                        sdi_mask <= s_axi_wdata[8:0];
                    end else if (s_axi_wdata[31:30] == 2'b01) begin
                        if (s_axi_wdata[19:16] < 4'd9)
                            chain_data[s_axi_wdata[19:16]] <= s_axi_wdata[15:0];
                    end else if (s_axi_wdata[31:30] == 2'b10) begin
                        oe_set_pulse <= 1'b1;
                        oe_set_val   <= s_axi_wdata[0];
                        if (s_axi_wdata[24:16] != 9'd0)
                            au_rows_max <= s_axi_wdata[24:16] - 1'b1;
                        if (s_axi_wdata[27]) begin       // v4 cfg_we
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
                end
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    s_axi_bvalid <= 1'b0;
            end
            if (start_pulse) burst_reg <= 16'b0;
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
                s_axi_rdata   <= {24'b0, dclk_fast, overlap_en, use_fb, auto_en, oe_out, icnd_busy, cmd_pending, busy};
                s_axi_rresp   <= 2'b00;
                s_axi_rvalid  <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

    //---------- auto 自主扫描引擎状态 ----------
    localparam AU_IDLE  = 3'd0;
    localparam AU_FILL1 = 3'd1;
    localparam AU_FILL2 = 3'd2;
    localparam AU_WAIT  = 3'd3;
    localparam AU_ROW   = 3'd4;
    localparam AU_ROWW  = 3'd5;
    localparam AU_DISP  = 3'd6;
    localparam AU_FBRD  = 3'd7;   // fb 读延迟拍
    reg [2:0]  au_state;
    reg [3:0]  au_word;           // fb 模式当前 word 0..11
    reg [8:0]  au_row;        // 0..383 (24 颗 3019 × 16)
    reg [19:0] au_cnt;
    reg        auto_oe;
    reg        au_icnd_go;    // 1 拍脉冲 → 3019 FSM advance
    reg        au_icnd_sdi;
    reg [9:0]  oe_cnt;        // v4 overlap: OE 低窗口计数 (aclk), 独立于 FSM 跑
    reg        oe_done;       // v4: 窗口已收 (OE 已回高), AU_WAIT 的行推进门条件
    reg        adv_fired;     // v4.1: 本行 3019 推进已发 (overlap 下藏进移位消隐尾并行跑)

    //---------- framebuffer: 9 lane × 512 × 32bit (row[5:0],pair[2:0]) ----------
    reg  [8:0]  fb_raddr;
    wire [31:0] fb_dout [0:8];
    genvar gi;
    generate for (gi = 0; gi < 9; gi = gi + 1) begin: g_fb
        reg [31:0] mem [0:511];
        reg [31:0] dout_r;
        always @(posedge s_axi_aclk) begin
            if (fb_we && fb_wlane == gi[3:0]) mem[fb_waddr] <= fb_wdata;
            dout_r <= mem[fb_raddr];
        end
        assign fb_dout[gi] = dout_r;
    end endgenerate

    //---------- OE register ----------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)      oe_out <= 1'b1;   // 复位消隐
        else if (auto_en)        oe_out <= auto_oe;
        else if (oe_set_pulse)   oe_out <= oe_set_val;
    end

    //---------- 命令装载 (task 消重复) ----------
    // idle 启动与 fall 链接共用: 装载 pending 命令到工作寄存器并驱第一 bit
    task load_cmd;
        begin
            burst_left    <= pending_burst;
            data_latched  <= pending_data;
            mode_latched  <= pending_mode;
            le_latched    <= pending_le;
            mode_reg      <= pending_mode;
            le_count_reg  <= pending_le;
            data_shift    <= pending_data;
            case (pending_mode)
                2'b01: begin   // marker_LE
                    bits_left <= (pending_le == 7'd0) ? 7'd1 : pending_le;
                    le_out    <= 1'b1;
                    sdi_out   <= 9'b0;
                end
                2'b11: begin   // per-chain word
                    bits_left <= 7'd16;
                    chain_shift[0] <= pending_chain_data[0];
                    chain_shift[1] <= pending_chain_data[1];
                    chain_shift[2] <= pending_chain_data[2];
                    chain_shift[3] <= pending_chain_data[3];
                    chain_shift[4] <= pending_chain_data[4];
                    chain_shift[5] <= pending_chain_data[5];
                    chain_shift[6] <= pending_chain_data[6];
                    chain_shift[7] <= pending_chain_data[7];
                    chain_shift[8] <= pending_chain_data[8];
                    sdi_out <= {pending_chain_data[8][15], pending_chain_data[7][15], pending_chain_data[6][15],
                                pending_chain_data[5][15], pending_chain_data[4][15], pending_chain_data[3][15],
                                pending_chain_data[2][15], pending_chain_data[1][15], pending_chain_data[0][15]} & sdi_mask;
                    le_out  <= (pending_le >= 7'd16);
                end
                default: begin // broadcast word (00 / 10)
                    bits_left <= 7'd16;
                    sdi_out   <= {9{pending_data[15]}} & sdi_mask;
                    le_out    <= (pending_le >= 7'd16);
                end
            endcase
        end
    endtask

    // burst 重发装载 (用已 latch 的命令)
    task reload_cmd;
        begin
            data_shift   <= data_latched;
            mode_reg     <= mode_latched;
            le_count_reg <= le_latched;
            case (mode_latched)
                2'b01: begin
                    bits_left <= (le_latched == 7'd0) ? 7'd1 : le_latched;
                    le_out    <= 1'b1;
                    sdi_out   <= 9'b0;
                end
                default: begin
                    bits_left <= 7'd16;
                    sdi_out   <= {9{data_latched[15]}} & sdi_mask;
                    le_out    <= (le_latched >= 7'd16);
                end
            endcase
        end
    endtask

    //---------- Sequencer ----------
    integer chk;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            busy          <= 1'b0;
            le_out        <= 1'b0;
            sdi_out       <= 9'b0;
            data_shift    <= 16'b0;
            bits_left     <= 7'b0;
            le_count_reg  <= 7'b0;
            mode_reg      <= 2'b0;
            data_latched  <= 16'b0;
            mode_latched  <= 2'b0;
            le_latched    <= 7'b0;
            burst_left    <= 16'b0;
            cmd_pending   <= 1'b0;
            pending_data  <= 16'b0;
            pending_mode  <= 2'b0;
            pending_le    <= 7'b0;
            pending_burst <= 16'b0;
            for (chk = 0; chk < 9; chk = chk + 1) begin
                chain_shift[chk] <= 16'b0;
                pending_chain_data[chk] <= 16'b0;
            end
            au_state    <= AU_IDLE;
            au_word     <= 4'd0;
            au_row      <= 9'd0;
            au_cnt      <= 20'd0;
            auto_oe     <= 1'b1;
            au_icnd_go  <= 1'b0;
            au_icnd_sdi <= 1'b0;
            oe_cnt      <= 10'd0;
            oe_done     <= 1'b1;
            adv_fired   <= 1'b0;
        end else begin
            au_icnd_go <= 1'b0;
            // v4 overlap: OE 窗口独立计数, 到点回高 (在 case 之前, 状态机赋值优先)
            if (!oe_done) begin
                if (oe_cnt == 10'd0) begin
                    auto_oe <= 1'b1;
                    oe_done <= 1'b1;
                end else oe_cnt <= oe_cnt - 1'b1;
            end
            // v4.1: overlap 下 3019 行推进并行藏进移位的消隐尾 (OE 已回高即可发,
            // 与 2049 DCLK/SDI/LE 完全独立; OE 下降沿前 icnd_busy 必已收尾)
            if (auto_en && overlap_en && !adv_fired && oe_done &&
                !icnd_busy && !au_icnd_go &&
                (au_state == AU_FILL1 || au_state == AU_FBRD ||
                 au_state == AU_FILL2 || au_state == AU_WAIT)) begin
                au_icnd_sdi <= (au_row == 9'd0);
                au_icnd_go  <= 1'b1;
                adv_fired   <= 1'b1;
            end
            if (!seq_active && cmd_pending) begin
                // idle 启动: 立即装载 + 驱第一 bit (dclk=0, 半周期后首个上升沿采样)
                cmd_pending <= 1'b0;
                load_cmd;
            end else if (dclk_will_fall) begin
                if (bits_left == 7'd1) begin
                    // 最后一拍: 链接下一命令 (无 DCLK gap) 或收工
                    if (burst_left != 16'b0) begin
                        burst_left <= burst_left - 1;
                        reload_cmd;
                    end else if (cmd_pending) begin
                        cmd_pending <= 1'b0;
                        load_cmd;
                    end else begin
                        bits_left <= 7'd0;
                        le_out    <= 1'b0;
                        sdi_out   <= 9'b0;
                        busy      <= 1'b0;
                    end
                end else begin
                    // 中段: 推下一 bit
                    bits_left <= bits_left - 1;
                    if (mode_reg == 2'b01) begin
                        // marker_LE: 保持 le_out, sdi=0
                    end else if (mode_reg == 2'b11) begin
                        chain_shift[0] <= {chain_shift[0][14:0], 1'b0};
                        chain_shift[1] <= {chain_shift[1][14:0], 1'b0};
                        chain_shift[2] <= {chain_shift[2][14:0], 1'b0};
                        chain_shift[3] <= {chain_shift[3][14:0], 1'b0};
                        chain_shift[4] <= {chain_shift[4][14:0], 1'b0};
                        chain_shift[5] <= {chain_shift[5][14:0], 1'b0};
                        chain_shift[6] <= {chain_shift[6][14:0], 1'b0};
                        chain_shift[7] <= {chain_shift[7][14:0], 1'b0};
                        chain_shift[8] <= {chain_shift[8][14:0], 1'b0};
                        sdi_out <= {chain_shift[8][14], chain_shift[7][14], chain_shift[6][14],
                                    chain_shift[5][14], chain_shift[4][14], chain_shift[3][14],
                                    chain_shift[2][14], chain_shift[1][14], chain_shift[0][14]} & sdi_mask;
                        le_out  <= ((bits_left - 1) <= le_count_reg);
                    end else begin
                        data_shift <= {data_shift[14:0], 1'b0};
                        sdi_out    <= {9{data_shift[14]}} & sdi_mask;
                        le_out     <= ((bits_left - 1) <= le_count_reg);
                    end
                end
            end

            // 接受新命令 (放最后, busy<=1 win 任何 race)
            if (start_pulse) begin
                pending_data  <= s_axi_wdata[15:0];
                pending_mode  <= s_axi_wdata[25:24];
                pending_le    <= s_axi_wdata[22:16];
                pending_burst <= burst_reg;
                pending_chain_data[0] <= chain_data[0];
                pending_chain_data[1] <= chain_data[1];
                pending_chain_data[2] <= chain_data[2];
                pending_chain_data[3] <= chain_data[3];
                pending_chain_data[4] <= chain_data[4];
                pending_chain_data[5] <= chain_data[5];
                pending_chain_data[6] <= chain_data[6];
                pending_chain_data[7] <= chain_data[7];
                pending_chain_data[8] <= chain_data[8];
                cmd_pending   <= 1'b1;
                busy          <= 1'b1;
            end

            // ---- auto 引擎 (手动 start_pulse 优先, auto 只在队列空时注入) ----
            if (auto_en) begin
                case (au_state)
                    AU_IDLE: begin
                        auto_oe <= 1'b1;
                        if (!busy && !cmd_pending && !start_pulse && !icnd_busy)
                            au_state <= AU_FILL1;
                    end
                    AU_FILL1: if (use_fb) begin
                        au_word  <= 4'd0;
                        fb_raddr <= {au_row[5:0], 3'd0};
                        au_state <= AU_FBRD;
                    end else if (!cmd_pending && !start_pulse) begin
                        pending_data  <= auto_pattern;
                        pending_mode  <= 2'b00;
                        pending_le    <= 7'd0;
                        pending_burst <= 16'd10;   // 11 词无 LE
                        cmd_pending   <= 1'b1;
                        busy          <= 1'b1;
                        au_state      <= AU_FILL2;
                    end
                    AU_FBRD: au_state <= AU_FILL2;   // BRAM 1 拍延迟
                    AU_FILL2: if (use_fb) begin
                        if (!cmd_pending && !start_pulse) begin
                            // 注入 per-chain word: 9 lane 并行取 BRAM 半字
                            pending_chain_data[0] <= au_word[0] ? fb_dout[0][31:16] : fb_dout[0][15:0];
                            pending_chain_data[1] <= au_word[0] ? fb_dout[1][31:16] : fb_dout[1][15:0];
                            pending_chain_data[2] <= au_word[0] ? fb_dout[2][31:16] : fb_dout[2][15:0];
                            pending_chain_data[3] <= au_word[0] ? fb_dout[3][31:16] : fb_dout[3][15:0];
                            pending_chain_data[4] <= au_word[0] ? fb_dout[4][31:16] : fb_dout[4][15:0];
                            pending_chain_data[5] <= au_word[0] ? fb_dout[5][31:16] : fb_dout[5][15:0];
                            pending_chain_data[6] <= au_word[0] ? fb_dout[6][31:16] : fb_dout[6][15:0];
                            pending_chain_data[7] <= au_word[0] ? fb_dout[7][31:16] : fb_dout[7][15:0];
                            pending_chain_data[8] <= au_word[0] ? fb_dout[8][31:16] : fb_dout[8][15:0];
                            pending_mode  <= 2'b11;
                            pending_le    <= (au_word == 4'd11) ? ((au_row == 9'd0) ? 7'd5 : 7'd4) : 7'd0;
                            pending_burst <= 16'd0;
                            cmd_pending   <= 1'b1;
                            busy          <= 1'b1;
                            if (au_word == 4'd11) begin
                                au_state <= AU_WAIT;
                            end else begin
                                au_word  <= au_word + 1'b1;
                                fb_raddr <= {au_row[5:0], au_word[3:1] + au_word[0]}; // 下一 word 的 pair
                                au_state <= AU_FBRD;
                            end
                        end
                    end else if (!cmd_pending && !start_pulse) begin
                        pending_data  <= auto_pattern;
                        pending_mode  <= 2'b00;
                        pending_le    <= (au_row == 9'd0) ? 7'd5 : 7'd4;  // 首行5/换行4
                        pending_burst <= 16'd0;
                        cmd_pending   <= 1'b1;
                        au_state      <= AU_WAIT;
                    end
                    // overlap: 推进已并行发出, 等移位+推进+OE窗三者齐 → 直接 OE 下降
                    // 非 overlap: 老路径走 AU_ROW 串行推进
                    AU_WAIT: if (overlap_en) begin
                        if (!busy && !cmd_pending && oe_done &&
                            adv_fired && !icnd_busy && !au_icnd_go)
                            au_state <= AU_ROWW;
                    end else if (!busy && !cmd_pending)
                        au_state <= AU_ROW;
                    AU_ROW: begin
                        au_icnd_sdi <= (au_row == 9'd0);   // 单 '1' 进链
                        au_icnd_go  <= 1'b1;
                        au_state    <= AU_ROWW;
                    end
                    AU_ROWW: if (!icnd_busy && !au_icnd_go) begin
                        auto_oe  <= 1'b0;                  // OE 下降沿转移 reg2 + 显示
                        if (overlap_en) begin
                            // 显示刚锁存的 au_row, 同时立刻去移下一行 (2049 双缓存)
                            oe_cnt    <= dclk_fast ? {1'b0, oe_window, 1'b0}
                                                   : {oe_window, 2'b0};   // DCLK→aclk
                            oe_done   <= 1'b0;
                            adv_fired <= 1'b0;
                            au_row    <= (au_row >= au_rows_max) ? 9'd0 : au_row + 1'b1;
                            au_state  <= AU_FILL1;
                        end else begin
                            au_cnt   <= auto_disp_cyc;
                            au_state <= AU_DISP;
                        end
                    end
                    AU_DISP: begin
                        if (au_cnt == 20'd0) begin
                            auto_oe  <= 1'b1;
                            au_row   <= (au_row >= au_rows_max) ? 9'd0 : au_row + 1'b1;
                            // fb 模式整行重灌; 均匀模式走 FILL2 捷径 (锁存移入同 word, 内容不变)
                            au_state <= use_fb ? AU_FILL1 : AU_FILL2;
                        end else au_cnt <= au_cnt - 1'b1;
                    end
                    default: au_state <= AU_IDLE;
                endcase
            end else begin
                au_state  <= AU_IDLE;
                auto_oe   <= 1'b1;
                oe_done   <= 1'b1;
                adv_fired <= 1'b0;
            end
        end
    end

    //---------- ICND3019 FSM (原样保留, 50 MHz 下时序全满足) ----------
    localparam ICND_S_IDLE     = 3'd0;
    localparam ICND_S_ADV_PRE  = 3'd1;
    localparam ICND_S_ADV_HIGH = 3'd2;
    localparam ICND_S_ADV_HOLD = 3'd3;
    localparam ICND_S_CFG_PRE  = 3'd4;
    localparam ICND_S_CFG_HIGH = 3'd5;
    localparam ICND_S_CFG_LOW  = 3'd6;
    localparam ICND_S_CFG_POST = 3'd7;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            icnd_state        <= ICND_S_IDLE;
            icnd_busy         <= 1'b0;
            icnd_div          <= 7'b0;
            icnd_pulse_count  <= 5'b0;
            icnd_pulse_target <= 5'b0;
            icnd_sdi_out      <= 1'b0;
            icnd_dclk_out     <= 1'b0;
            icnd_rclk_out     <= 1'b0;
        end else begin
            case (icnd_state)
                ICND_S_IDLE: begin
                    icnd_dclk_out <= 1'b0;
                    icnd_rclk_out <= 1'b0;
                    if (icnd_start_pulse) begin
                        icnd_busy <= 1'b1;
                        icnd_div  <= 7'b0;
                        if (icnd_pending_type == 1'b0) begin
                            icnd_sdi_out <= icnd_pending_sdi;
                            icnd_state   <= ICND_S_ADV_PRE;
                        end else begin
                            icnd_pulse_target <= icnd_pending_reg + 5'd8;
                            icnd_pulse_count  <= 5'b0;
                            icnd_state        <= ICND_S_CFG_PRE;
                        end
                    end else if (au_icnd_go) begin
                        // auto 引擎行推进
                        icnd_busy    <= 1'b1;
                        icnd_div     <= 7'b0;
                        icnd_sdi_out <= au_icnd_sdi;
                        icnd_state   <= ICND_S_ADV_PRE;
                    end
                end
                ICND_S_ADV_PRE: begin
                    if (icnd_div == 7'd7) begin
                        icnd_div      <= 7'b0;
                        icnd_dclk_out <= 1'b1;
                        icnd_state    <= ICND_S_ADV_HIGH;
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_ADV_HIGH: begin
                    if (icnd_div == 7'd63) begin
                        icnd_div      <= 7'b0;
                        icnd_dclk_out <= 1'b0;
                        icnd_state    <= ICND_S_ADV_HOLD;
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_ADV_HOLD: begin
                    if (icnd_div == 7'd7) begin
                        icnd_busy  <= 1'b0;
                        icnd_state <= ICND_S_IDLE;
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_CFG_PRE: begin
                    if (icnd_div == 7'd15) begin
                        icnd_div      <= 7'b0;
                        icnd_rclk_out <= 1'b1;
                        icnd_state    <= ICND_S_CFG_HIGH;
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_CFG_HIGH: begin
                    if (icnd_div == 7'd7) begin
                        icnd_div      <= 7'b0;
                        icnd_rclk_out <= 1'b0;
                        icnd_state    <= ICND_S_CFG_LOW;
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_CFG_LOW: begin
                    if (icnd_div == 7'd7) begin
                        icnd_div <= 7'b0;
                        if (icnd_pulse_count == icnd_pulse_target - 1) begin
                            icnd_state <= ICND_S_CFG_POST;
                        end else begin
                            icnd_pulse_count <= icnd_pulse_count + 1;
                            icnd_rclk_out    <= 1'b1;
                            icnd_state       <= ICND_S_CFG_HIGH;
                        end
                    end else icnd_div <= icnd_div + 1;
                end
                ICND_S_CFG_POST: begin
                    if (icnd_div == 7'd15) begin
                        icnd_busy  <= 1'b0;
                        icnd_state <= ICND_S_IDLE;
                    end else icnd_div <= icnd_div + 1;
                end
                default: icnd_state <= ICND_S_IDLE;
            endcase
        end
    end

endmodule
