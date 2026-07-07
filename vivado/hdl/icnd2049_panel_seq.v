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
//     11 → auto 自主扫描: [0]=auto_en, [23:8]=auto_pattern (每芯片 16bit 图形),
//          [29:24]=disp 窗 (单位 1024 aclk, 0 当 3 用 → 默认 ~61us)
//          auto 模式: PL 自己循环 灌12词(LE=4/首行5)→3019走行(384位单'1')→OE低显示,
//          ~130Hz 帧率, 无需 ARM/JTAG 持续喂
//
// 时钟: s_axi_aclk = FCLK0 50 MHz, DCLK = 50/DCLK_DIV = 12.5 MHz (2049 max 25M)
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module icnd2049_panel_seq #(
    parameter DCLK_DIV = 4    // 50 MHz / 4 = 12.5 MHz DCLK, 2/2 = 50% 占空比
)(
    // AXI-Lite slave
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    input  wire [3:0]  s_axi_awaddr,
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
    input  wire [3:0]  s_axi_araddr,
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

    localparam [7:0] HALF = (DCLK_DIV/2) - 1;

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
    reg        auto_en;
    reg [15:0] auto_pattern;
    reg [19:0] auto_disp_cyc;   // 显示窗 aclk 数
    reg [8:0]  au_rows_max;     // 扫描行数-1

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
            auto_en           <= 1'b0;
            auto_pattern      <= 16'h8000;   // 每芯片 1 点; 9 路全开单行 ~1.6A 安全
            auto_disp_cyc     <= 20'd3072;
            au_rows_max       <= 9'd383;
            for (ci = 0; ci < 9; ci = ci + 1)
                chain_data[ci] <= 16'b0;
        end else begin
            start_pulse      <= 1'b0;
            icnd_start_pulse <= 1'b0;
            oe_set_pulse     <= 1'b0;
            if (!s_axi_awready && !s_axi_wready &&
                s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                if (s_axi_awaddr[3:2] == 2'b00 && !cmd_pending) begin
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
                    end else if (s_axi_wdata[31:30] == 2'b11) begin
                        auto_en       <= s_axi_wdata[0];
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
                s_axi_rdata   <= {27'b0, auto_en, oe_out, icnd_busy, cmd_pending, busy};
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
    reg [2:0]  au_state;
    reg [8:0]  au_row;        // 0..383 (24 颗 3019 × 16)
    reg [19:0] au_cnt;
    reg        auto_oe;
    reg        au_icnd_go;    // 1 拍脉冲 → 3019 FSM advance
    reg        au_icnd_sdi;

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
            au_row      <= 9'd0;
            au_cnt      <= 20'd0;
            auto_oe     <= 1'b1;
            au_icnd_go  <= 1'b0;
            au_icnd_sdi <= 1'b0;
        end else begin
            au_icnd_go <= 1'b0;
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
                    AU_FILL1: if (!cmd_pending && !start_pulse) begin
                        pending_data  <= auto_pattern;
                        pending_mode  <= 2'b00;
                        pending_le    <= 7'd0;
                        pending_burst <= 16'd10;   // 11 词无 LE
                        cmd_pending   <= 1'b1;
                        busy          <= 1'b1;
                        au_state      <= AU_FILL2;
                    end
                    AU_FILL2: if (!cmd_pending && !start_pulse) begin
                        pending_data  <= auto_pattern;
                        pending_mode  <= 2'b00;
                        pending_le    <= (au_row == 9'd0) ? 7'd5 : 7'd4;  // 首行5/换行4
                        pending_burst <= 16'd0;
                        cmd_pending   <= 1'b1;
                        au_state      <= AU_WAIT;
                    end
                    AU_WAIT: if (!busy && !cmd_pending) au_state <= AU_ROW;
                    AU_ROW: begin
                        au_icnd_sdi <= (au_row == 9'd0);   // 单 '1' 进链
                        au_icnd_go  <= 1'b1;
                        au_state    <= AU_ROWW;
                    end
                    AU_ROWW: if (!icnd_busy && !au_icnd_go) begin
                        auto_oe  <= 1'b0;                  // OE 下降沿转移 reg2 + 显示
                        au_cnt   <= auto_disp_cyc;
                        au_state <= AU_DISP;
                    end
                    AU_DISP: begin
                        if (au_cnt == 20'd0) begin
                            auto_oe  <= 1'b1;
                            au_row   <= (au_row >= au_rows_max) ? 9'd0 : au_row + 1'b1;
                            // ⚠ 仅均匀图形成立: FILL2 锁存必移入 16bit, 全链同 word 时内容不变
                            // (12× 省填充); 将来接 framebuffer 逐行不同数据必须回 AU_FILL1 全灌
                            au_state <= AU_FILL2;
                        end else au_cnt <= au_cnt - 1'b1;
                    end
                    default: au_state <= AU_IDLE;
                endcase
            end else begin
                au_state <= AU_IDLE;
                auto_oe  <= 1'b1;
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
