//-----------------------------------------------------------------------------
// icnd2047_panel_core.v — ICND2047 双沿列驱引擎核 (方案 C)
//
// 照 docs/design_icnd2047/01_rtl_ddr_engine.md 编码; 寄存器归属按 00_overview
// 裁决节 (行驱 cfg=0x24, frame_period=0x28; oe_window 默认 48 沿由上层寄存器给,
// 本核只做 [2,187] 箝位)。上层 pov 顶层 (AXI/angle_tracker/ddr_slice_fetch/fb
// BRAM/写仲裁) 把本核当黑盒, fb 读口与寄存器语义 = v5 icnd2049_panel_pov 现状。
//
// 数据路径 (01 文档 §1.2, 方案 C):
//   * 全逻辑 50MHz 单域: 1 aclk = 1 沿 = 1 bit (双沿 50Mbps/lane @ DCLK 25MHz)
//   * SDI×9 / LE / OE: ODDR D1=D2 (等效 SDR, 与 DCLK 共享 ODDR→pad 路径)
//   * DCLK: ODDR SAME_EDGE D1=dclk_d(上一拍) D2=dclk_r(当前拍) → 边沿推迟半拍
//     10ns, 落数据眼图正中 (setup/hold 名义 10ns/10ns, 芯片要求 5/5ns)
//   * ddr_slow=1 降级: dclk_r 每 2 拍翻 (12.5MHz), 数据每 2 拍换 → 25Mbps
//     (等效现役单沿速率, SI 逃生门, 协议仍是双沿采样)
//
// 移位 FSM (01 §2.3): EG_FETCH 1 + EG_LOAD 1 + EG_SHIFT 192 + EG_LWAIT 0..n +
//   EG_DISP 1 = 195 拍/行 (fast, LWAIT=0) → 54 行 = 10530 拍 = 4.75kHz
// fb 流水预取 (01 §2.2): sh_cnt[4:0]==29 发 raddr, ==31 换 pair — 无逐词气泡。
// fb 布局逐 bit 兼容 v5: 9 lane × 512 × 32b, addr={row[5:0],pair[2:0]},
//   上线序 word0[15..0]..word11[15..0] (pack_obs/gen_chess_obs 零改动)。
// LE (01 §2.4): 与数据尾重叠, 首行 5 沿 / 换行 4 沿; 覆盖 EG_SHIFT 最后 le_len
//   个 bit ⇒ 恰 le_len 个 DCLK 沿且必含上升沿 (0 沿 Reset 保护)。
// OE 窗口 (01 §2.5): 单位=沿, 箝位 [2,187] (187 保证 LE 必落在 OE 回高后,
//   latch1→reg2 两种转移语义等价); 行推进藏尾 P2 照 v4.1 adv_fired 结构。
// 行驱: row_drv_icnd1028 子模块 (ICND3019 同族时序克隆, 0x24 cfg 运行时参数)。
// 手动路径 (01 §2.7): word=16 bit=16 拍; le_count 单位改沿; marker_LE 带沿。
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module icnd2047_panel_core (
    input  wire         aclk,           // 50 MHz (FCLK0)
    input  wire         aresetn,

    // ---- 寄存器语义配置 (上层 AXI 已锁存, 语义=v5) ----
    input  wire         auto_en,        // 0x0C sub11 [0]
    input  wire [8:0]   rows,           // 0x0C sub10 [24:16] 扫描行数 (54); 0 当 54
    input  wire [7:0]   oe_window,      // 0x0C sub10 [15:8], 单位=沿, 内箝 [2,187]
    input  wire         ddr_slow,       // 0x0C sub10 [29] (原 dclk_fast 改义)
    input  wire         oe_set_pulse,   // 0x0C sub10 手动 OE (auto_en=0 时有效)
    input  wire         oe_set_val,
    input  wire [8:0]   sdi_mask,       // 0x0C sub00

    // ---- fb 读口 (v5 g_fb BRAM 语义: 同步读 1 拍延迟) ----
    output reg  [8:0]   fb_raddr,       // {row[5:0], pair[2:0]}
    input  wire [287:0] fb_dout_flat,   // 9 lane × 32b, lane i = [i*32 +: 32]

    // ---- 手动命令 (v5 0x00/0x04 语义, le_count 单位=沿) ----
    input  wire         cmd_start,      // 1 拍脉冲, 同拍 cmd_* 有效
    input  wire [15:0]  cmd_data,
    input  wire [6:0]   cmd_le,         // 沿数 (寄存器写 11/12)
    input  wire [1:0]   cmd_mode,       // 00/10=broadcast word, 01=marker_LE, 11=per-chain
    input  wire [15:0]  cmd_burst,      // 0x04: 自动重发 N 次 (总 N+1)
    input  wire [143:0] chain_data_flat,// 9 × 16b per-chain 数据

    // ---- 行驱手动 (v5 0x08) + 0x24 运行时时序 cfg ----
    input  wire         row_man_go,
    input  wire         row_man_type,   // 0=advance 1=config
    input  wire         row_man_sdi,
    input  wire [3:0]   row_man_reg,
    input  wire [31:0]  row_cfg,        // 0x24: [7:0]=adv_high [15:8]=pre/hold
                                        //       [16]=bk极性 [17]=dclk极性 [18]=lck极性
    // ---- status (R 0x00 扩展位 + 0x24R/0x28R 素材) ----
    output wire         busy,           // 手动 sequencer busy
    output wire         cmd_pending_o,
    output wire         row_busy_o,     // R0x00[13]
    output wire [2:0]   eg_state_o,     // 0x24R [31:29]
    output wire [8:0]   shift_row_o,
    output wire [15:0]  frame_count_o,
    output wire [31:0]  frame_period_o, // 0x28R: 最近一整屏 aclk 数 (期望 ~10530)
    output wire         oe_done_o,      // R0x00[11]
    output wire         oe_state_o,     // OE fabric 镜像 (pad 是 ODDR, 不可回采)
    output wire         adv_fired_o,    // R0x00[12]

    // ---- pads (全走 ODDR/IOB) ----
    output wire         dclk_pad,
    output wire         le_pad,
    output wire         oe_pad,         // 1=消隐
    output wire [8:0]   sdi_pad,
    output wire         row_sdi,
    output wire         row_dclk,
    output wire         row_lck,
    output wire         row_bk
);

    // ---------------- fb 展开 ----------------
    wire [31:0] fb_dout_w [0:8];
    genvar gu;
    generate for (gu = 0; gu < 9; gu = gu + 1) begin: g_unf
        assign fb_dout_w[gu] = fb_dout_flat[gu*32 +: 32];
    end endgenerate

    // ---------------- 箝位 / 派生 ----------------
    wire [7:0] win_c    = (oe_window < 8'd2)   ? 8'd2   :
                          (oe_window > 8'd187) ? 8'd187 : oe_window;
    // OE 低宽单位=沿; ddr_slow 时 1 沿 = 2 aclk (占空比守恒, LE-after-OE 关系守恒)
    wire [8:0] win_aclk = ddr_slow ? {win_c, 1'b0} : {1'b0, win_c};
    wire [8:0] row_max  = (rows == 9'd0) ? 9'd53 : (rows - 9'd1);

    // ---------------- 状态 ----------------
    localparam EG_IDLE  = 3'd0;
    localparam EG_FETCH = 3'd1;
    localparam EG_LOAD  = 3'd2;
    localparam EG_SHIFT = 3'd3;
    localparam EG_LWAIT = 3'd4;
    localparam EG_DISP  = 3'd5;
    localparam EG_MAN   = 3'd6;

    reg  [2:0]  state;
    reg  [7:0]  sh_cnt;             // 当前正在出线的 bit 0..191
    reg         slow_ph;            // ddr_slow: 每 bit 2 拍的相位
    reg  [31:0] pair_reg [0:8];     // 9 lane 当前 pair
    reg  [8:0]  sdi_r;              // → ODDR D1=D2
    reg         le_r;
    reg         dclk_r, dclk_d;     // dclk_d = dclk_r 延迟 1 拍 (ODDR D1)
    reg         oe_r;               // 1=消隐
    reg  [8:0]  oe_cnt;
    reg         oe_done;
    reg         adv_fired;
    reg  [8:0]  shift_row;
    reg         row_go_r, row_first_r;
    reg  [31:0] per_cnt, frame_period_r;
    reg  [15:0] frame_count_r;

    wire tick = ~ddr_slow | slow_ph;            // bit 推进节拍
    wire [7:0] nc = sh_cnt + 8'd1;              // 下一 bit 序号
    wire [8:0] next_row = (shift_row == row_max) ? 9'd0 : (shift_row + 9'd1);
    wire [2:0] le_len   = (shift_row == 9'd0) ? 3'd5 : 3'd4;   // 首行5/换行4
    wire [7:0] le_start = 8'd192 - {5'd0, le_len};
    // 行边界静默区 (0x24[30:25], 拍): LE尾→OE落 与 OE落→下行突发 各插死区,
    // 隔离 OE/LCK 电流阶跃与 CLK 沿 (2026-07-17 行边界串扰案)。0=关=原行为
    wire [5:0] q_gap = row_cfg[30:25];
    reg  [5:0] qg_cnt;

    // bit 提取 (01 §2.1): sel=bit[4:0]; half=sel[4], bit_sel=15-sel[3:0] (词内 MSB first)
    function pr_bit;
        input [31:0] w;
        input [4:0]  s;
        begin
            pr_bit = w[{s[4], ~s[3:0]}];
        end
    endfunction

    // ---------------- 手动命令寄存器 ----------------
    reg         cmd_pending, busy_r;
    reg  [15:0] pdata, pburst;
    reg  [6:0]  ple;
    reg  [1:0]  pmode;
    reg  [15:0] pchain [0:8];       // cmd_start 时快照 (race-free, v5 同款)
    reg  [15:0] ldata;              // burst 重发用锁存
    reg  [6:0]  lle;
    reg  [1:0]  lmode;
    reg  [15:0] lchain [0:8];
    reg  [15:0] mdata_sh;
    reg  [15:0] mchain_sh [0:8];
    reg  [6:0]  mk;                 // 当前 bit 0..mtotal-1
    reg  [6:0]  mtotal, mle;
    reg  [1:0]  mmode;
    reg  [15:0] mburst;

    wire [6:0] mle_eff  = (mle > mtotal) ? mtotal : mle;
    wire [6:0] mle_from = mtotal - mle_eff;     // bit>=mle_from 时 LE 高 (word/chain)

    // ---------------- 行驱子模块 ----------------
    wire rd_busy;
    row_drv_icnd1028 u_row (
        .clk      (aclk),
        .rst_n    (aresetn),
        .row_go   (row_go_r),
        .row_first(row_first_r),
        .man_go   (row_man_go),
        .man_type (row_man_type),
        .man_sdi  (row_man_sdi),
        .man_reg  (row_man_reg),
        .cfg      (row_cfg),
        .row_busy (rd_busy),
        .row_sdi  (row_sdi),
        .row_dclk (row_dclk),
        .row_lck  (row_lck),
        .row_bk   (row_bk)
    );

    wire disp_ready = oe_done && adv_fired && !rd_busy && !row_go_r;

    // ---------------- 主时序块 (单写者: 所有输出寄存器) ----------------
    integer li;
    always @(posedge aclk) begin
        if (!aresetn) begin
            state       <= EG_IDLE;
            sh_cnt      <= 8'd0;
            slow_ph     <= 1'b0;
            qg_cnt      <= 6'd0;
            sdi_r       <= 9'd0;
            le_r        <= 1'b0;
            dclk_r      <= 1'b0;
            oe_r        <= 1'b1;        // 复位消隐
            oe_cnt      <= 9'd0;
            oe_done     <= 1'b1;
            adv_fired   <= 1'b0;
            shift_row   <= 9'd0;
            row_go_r    <= 1'b0;
            row_first_r <= 1'b0;
            fb_raddr    <= 9'd0;
            per_cnt     <= 32'd0;
            frame_period_r <= 32'd0;
            frame_count_r  <= 16'd0;
            cmd_pending <= 1'b0;
            busy_r      <= 1'b0;
            pdata <= 16'd0; pburst <= 16'd0; ple <= 7'd0; pmode <= 2'd0;
            ldata <= 16'd0; lle <= 7'd0; lmode <= 2'd0;
            mdata_sh <= 16'd0; mk <= 7'd0; mtotal <= 7'd0; mle <= 7'd0;
            mmode <= 2'd0; mburst <= 16'd0;
            for (li = 0; li < 9; li = li + 1) begin
                pair_reg[li]  <= 32'd0;
                pchain[li]    <= 16'd0;
                lchain[li]    <= 16'd0;
                mchain_sh[li] <= 16'd0;
            end
        end else begin
            row_go_r <= 1'b0;
            per_cnt  <= per_cnt + 32'd1;

            // ---- P1: OE 窗口计数 (与 FSM 解耦; 单位=拍) ----
            if (!oe_done) begin
                if (oe_cnt == 9'd0) begin
                    oe_r    <= 1'b1;
                    oe_done <= 1'b1;
                end else
                    oe_cnt <= oe_cnt - 9'd1;
            end

            // ---- 手动 OE (auto 停时) ----
            if (!auto_en && oe_set_pulse)
                oe_r <= oe_set_val;

            // ---- P2: 行推进藏尾 (v4.1 adv_fired 结构) ----
            // OE 回高后立即在移位窗剩余时间并行推进行选; row_busy 期间 OE 恒 1
            if (auto_en && !adv_fired && oe_done && !rd_busy &&
                (state == EG_FETCH || state == EG_LOAD || state == EG_SHIFT)) begin
                row_first_r <= (shift_row == 9'd0);
                row_go_r    <= 1'b1;
                adv_fired   <= 1'b1;
            end

            // ---- 手动命令接收 (v5: latch pending) ----
            if (cmd_start && !cmd_pending) begin
                pdata  <= cmd_data;
                pmode  <= cmd_mode;
                ple    <= cmd_le;
                pburst <= cmd_burst;
                for (li = 0; li < 9; li = li + 1)
                    pchain[li] <= chain_data_flat[li*16 +: 16];
                cmd_pending <= 1'b1;
                busy_r      <= 1'b1;
            end

            // ---- 主 FSM ----
            case (state)
                //--------------------------------------------------------
                EG_IDLE: begin
                    dclk_r  <= 1'b0;       // idle DCLK 静默 (无沿)
                    le_r    <= 1'b0;
                    sdi_r   <= 9'd0;
                    slow_ph <= 1'b0;
                    if (cmd_pending && !auto_en) begin
                        // 手动装载: 进入 bit0 (dclk 起翻, 沿在 +10ns)
                        cmd_pending <= 1'b0;
                        mmode  <= pmode;
                        mle    <= pmode == 2'b01 ? ((ple == 7'd0) ? 7'd1 : ple) : ple;
                        mtotal <= (pmode == 2'b01) ? ((ple == 7'd0) ? 7'd1 : ple) : 7'd16;
                        mburst <= pburst;
                        mk     <= 7'd0;
                        ldata  <= pdata;
                        lmode  <= pmode;
                        lle    <= ple;
                        dclk_r <= 1'b1;    // 沿 1 (上升)
                        case (pmode)
                            2'b01: begin   // marker_LE: LE 高 + N 沿, SDI=0 (调试)
                                le_r  <= 1'b1;
                                sdi_r <= 9'd0;
                            end
                            2'b11: begin   // per-chain word
                                for (li = 0; li < 9; li = li + 1) begin
                                    lchain[li]    <= pchain[li];
                                    mchain_sh[li] <= {pchain[li][14:0], 1'b0};
                                    sdi_r[li]     <= pchain[li][15] & sdi_mask[li];
                                end
                                le_r <= (ple >= 7'd16);
                            end
                            default: begin // broadcast word
                                mdata_sh <= {pdata[14:0], 1'b0};
                                sdi_r    <= {9{pdata[15]}} & sdi_mask;
                                le_r     <= (ple >= 7'd16);
                            end
                        endcase
                        state <= EG_MAN;
                    end else if (auto_en && !busy_r && !rd_busy) begin
                        shift_row <= 9'd0;
                        adv_fired <= 1'b0;
                        fb_raddr  <= 9'd0;         // {row0, pair0}
                        state     <= EG_FETCH;
                    end
                end

                //--------------------------------------------------------
                EG_FETCH: begin                     // raddr 已在进入本态时置好
                    if (!auto_en) begin
                        state   <= EG_IDLE;
                        oe_r    <= 1'b1;
                        oe_done <= 1'b1;
                        oe_cnt  <= 9'd0;
                    end else if (qg_cnt >= q_gap) begin
                        qg_cnt <= 6'd0;
                        state  <= EG_LOAD;          // BRAM 读 1 拍 (静默区后)
                    end else
                        qg_cnt <= qg_cnt + 6'd1;    // OE落→突发 死区
                end

                //--------------------------------------------------------
                EG_LOAD: begin
                    if (!auto_en) begin
                        state   <= EG_IDLE;
                        oe_r    <= 1'b1;
                        oe_done <= 1'b1;
                        oe_cnt  <= 9'd0;
                    end else begin
                        // 装 pair0 + 出 bit0 + dclk 起翻 (进入 bit0)
                        for (li = 0; li < 9; li = li + 1) begin
                            pair_reg[li] <= fb_dout_w[li];
                            sdi_r[li]    <= fb_dout_w[li][15] & sdi_mask[li];
                        end
                        sh_cnt  <= 8'd0;
                        slow_ph <= 1'b0;
                        dclk_r  <= 1'b1;            // 沿 1 (上升) @ bit0 中点
                        le_r    <= 1'b0;            // 0 >= le_start 恒假
                        state   <= EG_SHIFT;
                    end
                end

                //--------------------------------------------------------
                EG_SHIFT: begin
                    // 流水预取: bit29 窗口发下一 pair 地址 (pair=5 时读 {row,6} 无害)
                    if (sh_cnt[4:0] == 5'd29)
                        fb_raddr <= {shift_row[5:0], sh_cnt[7:5] + 3'd1};
                    if (tick) begin
                        slow_ph <= 1'b0;
                        if (sh_cnt == 8'd191) begin
                            // 192 沿发完 (dclk_r 已回 0), latch1=本行数据
                            sdi_r <= 9'd0;
                            le_r  <= 1'b0;
                            if (!auto_en) begin     // 停 auto: 收尾本行回 IDLE
                                state   <= EG_IDLE;
                                oe_r    <= 1'b1;
                                oe_done <= 1'b1;
                                oe_cnt  <= 9'd0;
                            end else begin
                                qg_cnt <= 6'd0;
                                state  <= (disp_ready && q_gap == 6'd0) ? EG_DISP
                                                                        : EG_LWAIT;
                            end
                        end else begin
                            dclk_r <= ~dclk_r;      // 进入 bit nc → 沿在其中点
                            sh_cnt <= nc;
                            le_r   <= (nc >= le_start);  // 尾部 le_len 拍重叠
                            if (nc[4:0] == 5'd0) begin   // 换 pair (bit 32/64/../160)
                                for (li = 0; li < 9; li = li + 1) begin
                                    pair_reg[li] <= fb_dout_w[li];
                                    sdi_r[li]    <= fb_dout_w[li][15] & sdi_mask[li];
                                end
                            end else begin
                                for (li = 0; li < 9; li = li + 1)
                                    sdi_r[li] <= pr_bit(pair_reg[li], nc[4:0]) & sdi_mask[li];
                            end
                        end
                    end else
                        slow_ph <= 1'b1;
                end

                //--------------------------------------------------------
                EG_LWAIT: begin        // 等上一显示窗收完 + 行选推进完成 (稳态 0 拍)
                    if (!auto_en) begin
                        state   <= EG_IDLE;
                        oe_r    <= 1'b1;
                        oe_done <= 1'b1;
                        oe_cnt  <= 9'd0;
                    end else if (disp_ready) begin
                        if (qg_cnt >= q_gap) begin  // LE尾→OE落 死区
                            qg_cnt <= 6'd0;
                            state  <= EG_DISP;
                        end else
                            qg_cnt <= qg_cnt + 6'd1;
                    end
                end

                //--------------------------------------------------------
                EG_DISP: begin         // OE↓: reg2←latch1, 显示 shift_row; 1 拍
                    oe_r      <= 1'b0;
                    oe_cnt    <= win_aclk - 9'd1;
                    oe_done   <= 1'b0;
                    adv_fired <= 1'b0;
                    fb_raddr  <= {next_row[5:0], 3'd0};
                    if (shift_row == row_max) begin
                        shift_row      <= 9'd0;
                        frame_period_r <= per_cnt + 32'd1;   // 含本拍
                        per_cnt        <= 32'd0;
                        frame_count_r  <= frame_count_r + 16'd1;
                    end else
                        shift_row <= shift_row + 9'd1;
                    state <= EG_FETCH;
                end

                //--------------------------------------------------------
                EG_MAN: begin          // 手动 sequencer (双沿: 1 拍 = 1 沿)
                    if (tick) begin
                        slow_ph <= 1'b0;
                        if (mk == mtotal - 7'd1) begin
                            if (mburst != 16'd0) begin       // burst 无缝重发
                                mburst <= mburst - 16'd1;
                                mk     <= 7'd0;
                                dclk_r <= ~dclk_r;
                                case (lmode)
                                    2'b01: begin
                                        le_r  <= 1'b1;
                                        sdi_r <= 9'd0;
                                    end
                                    2'b11: begin
                                        for (li = 0; li < 9; li = li + 1) begin
                                            mchain_sh[li] <= {lchain[li][14:0], 1'b0};
                                            sdi_r[li]     <= lchain[li][15] & sdi_mask[li];
                                        end
                                        le_r <= (lle >= 7'd16);
                                    end
                                    default: begin
                                        mdata_sh <= {ldata[14:0], 1'b0};
                                        sdi_r    <= {9{ldata[15]}} & sdi_mask;
                                        le_r     <= (lle >= 7'd16);
                                    end
                                endcase
                            end else begin                    // 收工
                                sdi_r  <= 9'd0;
                                le_r   <= 1'b0;
                                busy_r <= cmd_pending;        // 有 pending 保持 busy
                                state  <= EG_IDLE;
                            end
                        end else begin
                            mk     <= mk + 7'd1;
                            dclk_r <= ~dclk_r;
                            case (mmode)
                                2'b01: ;                      // marker: 保持 LE/SDI
                                2'b11: begin
                                    for (li = 0; li < 9; li = li + 1) begin
                                        mchain_sh[li] <= {mchain_sh[li][14:0], 1'b0};
                                        sdi_r[li]     <= mchain_sh[li][15] & sdi_mask[li];
                                    end
                                    le_r <= ((mk + 7'd1) >= mle_from);
                                end
                                default: begin
                                    mdata_sh <= {mdata_sh[14:0], 1'b0};
                                    sdi_r    <= {9{mdata_sh[15]}} & sdi_mask;
                                    le_r     <= ((mk + 7'd1) >= mle_from);
                                end
                            endcase
                        end
                    end else
                        slow_ph <= 1'b1;
                end

                default: state <= EG_IDLE;
            endcase

            // 手动收工后 busy 清 (无 pending 时)
            if (state == EG_IDLE && !cmd_pending && busy_r && !(cmd_start && !cmd_pending))
                busy_r <= 1'b0;
        end
    end

    // dclk_d = dclk_r 延迟 1 拍 (ODDR D1: pad 前半拍旧值 → 沿恰在 +10ns)
    always @(posedge aclk) begin
        if (!aresetn) dclk_d <= 1'b0;
        else          dclk_d <= dclk_r;
    end

    // ---------------- 输出级: ODDR (01 §1.2 接线细目) ----------------
    wire rst_hi = ~aresetn;

    // 相位旋钮 (0x24: [20]=DCLK翻转 [22:21]=数据平移拍 [24:23]=LE平移拍)
    // 治首/尾 bit 装载错位 (板2 col_shift 同款病, 2026-07-16 共阳屏实见)
    // [30:25]=quiet_gap 行边界静默区拍数 (见 q_gap, FSM 内实现)
    wire       ph_dclk_inv = row_cfg[20];
    wire [1:0] ph_data_dly = row_cfg[22:21];
    wire [1:0] ph_le_dly   = row_cfg[24:23];
    reg  [8:0] sdi_p1, sdi_p2, sdi_p3;
    reg        le_p1, le_p2, le_p3;
    always @(posedge aclk) begin
        sdi_p1 <= sdi_r;  sdi_p2 <= sdi_p1;  sdi_p3 <= sdi_p2;
        le_p1  <= le_r;   le_p2  <= le_p1;   le_p3  <= le_p2;
    end
    wire [8:0] sdi_o = (ph_data_dly == 2'd0) ? sdi_r :
                       (ph_data_dly == 2'd1) ? sdi_p1 :
                       (ph_data_dly == 2'd2) ? sdi_p2 : sdi_p3;
    wire       le_o  = (ph_le_dly == 2'd0) ? le_r :
                       (ph_le_dly == 2'd1) ? le_p1 :
                       (ph_le_dly == 2'd2) ? le_p2 : le_p3;

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_dclk (
        .C(aclk), .CE(1'b1), .D1(dclk_d ^ ph_dclk_inv), .D2(dclk_r ^ ph_dclk_inv),
        .R(rst_hi), .S(1'b0), .Q(dclk_pad));

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_le (
        .C(aclk), .CE(1'b1), .D1(le_o), .D2(le_o),
        .R(rst_hi), .S(1'b0), .Q(le_pad));

    // OE 复位/GSR 必须=1 (消隐) → INIT=1 + S 复位
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b1), .SRTYPE("SYNC")) u_oddr_oe (
        .C(aclk), .CE(1'b1), .D1(oe_r), .D2(oe_r),
        .R(1'b0), .S(rst_hi), .Q(oe_pad));

    genvar gs;
    generate for (gs = 0; gs < 9; gs = gs + 1) begin: g_oddr_sdi
        ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_sdi (
            .C(aclk), .CE(1'b1), .D1(sdi_o[gs]), .D2(sdi_o[gs]),
            .R(rst_hi), .S(1'b0), .Q(sdi_pad[gs]));
    end endgenerate

    assign oe_state_o = oe_r;

    // ---------------- status ----------------
    assign busy           = busy_r | cmd_pending | (state == EG_MAN);
    assign cmd_pending_o  = cmd_pending;
    assign row_busy_o     = rd_busy;
    assign eg_state_o     = state;
    assign shift_row_o    = shift_row;
    assign frame_count_o  = frame_count_r;
    assign frame_period_o = frame_period_r;
    assign oe_done_o      = oe_done;
    assign adv_fired_o    = adv_fired;

endmodule
