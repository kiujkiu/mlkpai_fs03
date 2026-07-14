//==============================================================================
// tb_2047_core.v — ICND2047 双沿引擎核 (icnd2047_panel_core) xsim 自检 TB
//
// 验项 (对齐任务书 + 01 文档 §5):
//  [T1] 双沿 195 拍/行节拍: BFM 双沿采样 (negedge+posedge) 重建移位流, 与 fb
//       内容 192b/lane 逐 bit 比对 (9 lane 全查); 行周期=195 拍 / 192 沿/行 /
//       帧周期 10530 拍 (4.75kHz) / pad 数据-DCLK 沿距 10ns (setup/hold 0 违例)
//  [T2] LE 沿计数: auto 行序列 5,4,4,...,4 循环 (首行5/换行4, 含奇数沿跨周期);
//       手动路径 3 (普通锁存) / 11 (WR_REG1, BURST 级联) / 12 (WR_REG2) /
//       marker_LE 7 沿; 0 沿 LE=0 次 + LE 高必含上升沿 (Reset 保护)
//  [T3] overlap OE 窗: 宽度=oe_window 沿; 200→187 上箝 / 1→2 下箝 / 96 档;
//       160 档 LWAIT>0; OE 低期间下一行移位照跑 (overlap 证据)
//  [T4] 行切换消隐: row_busy=1 ⇒ OE=1 全程断言; row_first 仅回首行推进为 1;
//       0x24 adv_high=200 拉长 T_adv → LWAIT 出现且数据无错; 行 DCLK 宽度可调
//  [T5] ddr_slow 降级: 行周期 387 拍 / DCLK 沿距 40ns / OE 宽 96 拍 (占空守恒) /
//       数据逐 bit 仍对 / LE 5,4,4... / 帧周期 20898
//
// 跑法: sim/run_tb_2047.sh (cmd.exe + settings64.bat + xvlog/xelab/xsim,
//       ODDR 用 unisims_ver + glbl)
//==============================================================================
`timescale 1ns / 1ps

//------------------------------------------------------------------------------
// ICND2047 行为模型 (1 lane, 12 级联抽象为 192b 链):
// 双沿采样 SR + LE 沿计数/译码 + latch1 + OE↓ 转移 reg2 + 5ns setup/hold 检查
//------------------------------------------------------------------------------
module tb2047_bfm (
    input  wire         en,
    input  wire         dclk,
    input  wire         le,
    input  wire         oe,
    input  wire         sdi,
    output reg  [191:0] reg2_o,
    output reg  [191:0] latch1_o,
    output reg  [15:0]  viol_o,     // setup+hold <5ns 违例计数
    output reg  [15:0]  zle_o      // 0 沿 LE / 无上升沿 LE 计数
);
    reg [191:0] sr;
    integer  le_edges, le_re;
    integer  last_le;               // 最近一次 LE 译码沿数
    reg [7:0] le_hist [0:4095];     // LE 长度历史 (lane0 精查用)
    integer  le_hist_n;
    reg [15:0] cfg1, cfg2;          // WR_REG1/2 (11/12 沿)
    realtime t_sdi, t_le, t_edge;
    real     min_margin;

    initial begin
        sr = 192'd0; reg2_o = 192'd0; latch1_o = 192'd0;
        viol_o = 16'd0; zle_o = 16'd0;
        le_edges = 0; le_re = 0; last_le = -1; le_hist_n = 0;
        cfg1 = 16'd0; cfg2 = 16'd0;
        t_sdi = -100.0; t_le = -100.0; t_edge = -100.0;
        min_margin = 1.0e9;
    end

    // SDI/LE 翻转: 距离最近 DCLK 沿 <5ns = hold 违例
    always @(sdi) if (en) begin
        if ($realtime - t_edge < 5.0) viol_o = viol_o + 16'd1;
        t_sdi = $realtime;
    end
    always @(le) if (en) begin
        if ($realtime - t_edge < 5.0) viol_o = viol_o + 16'd1;
        t_le = $realtime;
    end

    // DCLK 任意沿 (双沿采样)
    always @(dclk) if (en && (dclk === 1'b0 || dclk === 1'b1)) begin
        if ($realtime - t_sdi < 5.0) viol_o = viol_o + 16'd1;   // setup
        if ($realtime - t_le  < 5.0) viol_o = viol_o + 16'd1;
        if ($realtime - t_sdi < min_margin) min_margin = $realtime - t_sdi;
        t_edge = $realtime;
        sr <= {sr[190:0], sdi};
        if (le) begin
            le_edges = le_edges + 1;
            if (dclk === 1'b1) le_re = le_re + 1;   // 上升沿
        end
    end

    always @(posedge le) if (en) begin
        le_edges = 0; le_re = 0;
    end

    // LE 下降沿译码
    always @(negedge le) if (en) begin
        if (le_edges == 0)              zle_o = zle_o + 16'd1;   // 0 沿 = Reset 歧义
        else if (le_re == 0)            zle_o = zle_o + 16'd1;   // 无上升沿
        last_le = le_edges;
        if (le_hist_n < 4096) le_hist[le_hist_n] = le_edges[7:0];
        le_hist_n = le_hist_n + 1;
        case (le_edges)
            3, 4, 5: latch1_o <= sr;    // DATA_LATCH (行推进由 row_drv 侧管)
            11:      cfg1 <= sr[15:0];  // WR_REG1
            12:      cfg2 <= sr[15:0];  // WR_REG2
        endcase
        le_edges = 0; le_re = 0;
    end

    // OE 下降沿: latch1 → reg2 (显示缓存转移)
    always @(negedge oe) if (en) reg2_o <= latch1_o;

endmodule

//------------------------------------------------------------------------------
module tb_2047_core;

    localparam integer ROWS = 54;

    reg clk = 1'b0;
    always #10 clk = ~clk;              // 50 MHz
    reg rstn = 1'b0;
    reg bfm_en = 1'b0;

    // ---------------- DUT 配置输入 ----------------
    reg         auto_en   = 1'b0;
    reg  [8:0]  rows      = ROWS;
    reg  [7:0]  oe_window = 8'd48;      // 00_overview 裁决默认
    reg         ddr_slow  = 1'b0;
    reg         oe_set_pulse = 1'b0;
    reg         oe_set_val   = 1'b1;
    reg  [8:0]  sdi_mask  = 9'h1FF;
    reg         cmd_start = 1'b0;
    reg  [15:0] cmd_data  = 16'd0;
    reg  [6:0]  cmd_le    = 7'd0;
    reg  [1:0]  cmd_mode  = 2'd0;
    reg  [15:0] cmd_burst = 16'd0;
    reg  [143:0] chain_data_flat = 144'd0;
    reg         row_man_go = 1'b0;
    reg         row_man_type = 1'b0;
    reg         row_man_sdi  = 1'b0;
    reg  [3:0]  row_man_reg  = 4'd0;
    reg  [31:0] row_cfg = 32'd0;

    // ---------------- fb 模型 (v5 g_fb 语义: 9×512×32, 同步读 1 拍) ----------------
    wire [8:0]  fb_raddr;
    reg  [31:0] fbmem [0:8][0:511];
    reg  [31:0] fb_dout_r [0:8];
    wire [287:0] fb_dout_flat;
    genvar gf;
    generate for (gf = 0; gf < 9; gf = gf + 1) begin: g_fbf
        assign fb_dout_flat[gf*32 +: 32] = fb_dout_r[gf];
    end endgenerate
    integer fi, fj, ftmp;
    initial begin
        for (fi = 0; fi < 9; fi = fi + 1)
            for (fj = 0; fj < 512; fj = fj + 1) begin
                ftmp = fi*512 + fj;
                fbmem[fi][fj] = (ftmp * 32'h9E37_79B1) ^ (ftmp << 13) ^ 32'h5A5A_A5A5;
            end
    end
    always @(posedge clk)
        for (fi = 0; fi < 9; fi = fi + 1)
            fb_dout_r[fi] <= fbmem[fi][fb_raddr];

    // ---------------- DUT ----------------
    wire        busy, cmd_pending_o, row_busy_o, oe_done_o, adv_fired_o;
    wire [2:0]  eg_state_o;
    wire [8:0]  shift_row_o;
    wire [15:0] frame_count_o;
    wire [31:0] frame_period_o;
    wire        dclk_pad, le_pad, oe_pad;
    wire [8:0]  sdi_pad;
    wire        row_sdi, row_dclk, row_lck, row_bk;

    icnd2047_panel_core dut (
        .aclk(clk), .aresetn(rstn),
        .auto_en(auto_en), .rows(rows), .oe_window(oe_window),
        .ddr_slow(ddr_slow), .oe_set_pulse(oe_set_pulse), .oe_set_val(oe_set_val),
        .sdi_mask(sdi_mask),
        .fb_raddr(fb_raddr), .fb_dout_flat(fb_dout_flat),
        .cmd_start(cmd_start), .cmd_data(cmd_data), .cmd_le(cmd_le),
        .cmd_mode(cmd_mode), .cmd_burst(cmd_burst),
        .chain_data_flat(chain_data_flat),
        .row_man_go(row_man_go), .row_man_type(row_man_type),
        .row_man_sdi(row_man_sdi), .row_man_reg(row_man_reg),
        .row_cfg(row_cfg),
        .busy(busy), .cmd_pending_o(cmd_pending_o), .row_busy_o(row_busy_o),
        .eg_state_o(eg_state_o), .shift_row_o(shift_row_o),
        .frame_count_o(frame_count_o), .frame_period_o(frame_period_o),
        .oe_done_o(oe_done_o), .adv_fired_o(adv_fired_o),
        .dclk_pad(dclk_pad), .le_pad(le_pad), .oe_pad(oe_pad), .sdi_pad(sdi_pad),
        .row_sdi(row_sdi), .row_dclk(row_dclk), .row_lck(row_lck), .row_bk(row_bk)
    );

    // ---------------- BFM: lane0 精查实例 + 9 lane 抽查阵 ----------------
    wire [191:0] b0_reg2, b0_latch1;
    wire [15:0]  b0_viol, b0_zle;
    tb2047_bfm u_bfm0 (.en(bfm_en), .dclk(dclk_pad), .le(le_pad), .oe(oe_pad),
                       .sdi(sdi_pad[0]),
                       .reg2_o(b0_reg2), .latch1_o(b0_latch1),
                       .viol_o(b0_viol), .zle_o(b0_zle));

    wire [1727:0] bfm_reg2_f, bfm_l1_f;
    wire [143:0]  bfm_viol_f, bfm_zle_f;
    genvar gb;
    generate for (gb = 0; gb < 9; gb = gb + 1) begin: g_bfm
        tb2047_bfm u_b (.en(bfm_en), .dclk(dclk_pad), .le(le_pad), .oe(oe_pad),
                        .sdi(sdi_pad[gb]),
                        .reg2_o(bfm_reg2_f[gb*192 +: 192]),
                        .latch1_o(bfm_l1_f[gb*192 +: 192]),
                        .viol_o(bfm_viol_f[gb*16 +: 16]),
                        .zle_o(bfm_zle_f[gb*16 +: 16]));
    end endgenerate

    // ---------------- 错误记账 ----------------
    integer errors = 0;
    task fail(input [511:0] msg);
        begin
            errors = errors + 1;
            $display("[%0t] FAIL: %0s", $time, msg);
        end
    endtask

    // ---------------- 金样: fb → 192b 行流 (与 BFM 同向拼接) ----------------
    function [191:0] golden_row(input integer lane, input integer row);
        integer p, b;
        reg [31:0] pr;
        reg [191:0] g;
        begin
            g = 192'd0;
            for (p = 0; p < 6; p = p + 1) begin
                pr = fbmem[lane][row*8 + p];
                for (b = 15; b >= 0; b = b - 1) g = {g[190:0], pr[b]};      // word 2p
                for (b = 15; b >= 0; b = b - 1) g = {g[190:0], pr[16+b]};   // word 2p+1
            end
            golden_row = g;
        end
    endfunction

    // ---------------- 监视器: OE↓ 逐行 reg2 vs 金样 (9 lane 全查) ----------------
    reg     cmp_en = 1'b0;
    integer disp_n = 0, data_err = 0, data_ok = 0;
    integer m_ln, m_row;
    reg [191:0] m_got, m_exp;
    always @(negedge oe_pad) if (cmp_en) begin
        #1;
        m_row = disp_n % ROWS;
        for (m_ln = 0; m_ln < 9; m_ln = m_ln + 1) begin
            m_got = bfm_reg2_f[m_ln*192 +: 192];
            m_exp = golden_row(m_ln, m_row);
            if (m_got !== m_exp) begin
                data_err = data_err + 1;
                if (data_err <= 5)
                    $display("[%0t] FAIL data: disp %0d row %0d lane %0d\n  got %048x\n  exp %048x",
                             $time, disp_n, m_row, m_ln, m_got, m_exp);
            end else data_ok = data_ok + 1;
        end
        disp_n = disp_n + 1;
    end

    // ---------------- 监视器: row_busy ⇒ OE 消隐 ----------------
    integer blank_err = 0;
    always @(posedge clk) if (rstn && bfm_en && row_busy_o && oe_pad !== 1'b1) begin
        blank_err = blank_err + 1;
        if (blank_err <= 3) $display("[%0t] FAIL: row_busy=1 but OE=%b", $time, oe_pad);
    end

    // ---------------- 监视器: row_first 正确性 ----------------
    integer rf_err = 0, rf_first_n = 0, rf_total_n = 0;
    always @(posedge clk) if (rstn && dut.row_go_r) begin
        rf_total_n = rf_total_n + 1;
        if (dut.row_first_r) rf_first_n = rf_first_n + 1;
        if (dut.row_first_r !== (dut.shift_row == 9'd0)) begin
            rf_err = rf_err + 1;
            $display("[%0t] FAIL: row_first=%b but shift_row=%0d",
                     $time, dut.row_first_r, dut.shift_row);
        end
    end

    // ---------------- 监视器: DCLK 沿计数 + 最小沿距 ----------------
    integer  dclk_edges = 0;
    realtime t_dedge = 0.0;
    real     min_dint = 1.0e9;
    always @(dclk_pad) if (bfm_en && (dclk_pad === 1'b0 || dclk_pad === 1'b1)) begin
        if (dclk_edges > 0 && ($realtime - t_dedge) < min_dint)
            min_dint = $realtime - t_dedge;
        t_dedge = $realtime;
        dclk_edges = dclk_edges + 1;
    end

    // ---------------- 汇总 BFM 违例 ----------------
    function integer viol_sum;
        input dummy;
        integer k;
        begin
            viol_sum = b0_viol + b0_zle;
            for (k = 0; k < 9; k = k + 1)
                viol_sum = viol_sum + bfm_viol_f[k*16 +: 16] + bfm_zle_f[k*16 +: 16];
        end
    endfunction

    // ---------------- 测量 task ----------------
    real     r_period, r_width;
    realtime t_a;
    integer  e_a;

    task measure_period(output real p);
        realtime t0;
        begin
            @(negedge oe_pad); t0 = $realtime;
            @(negedge oe_pad); p = ($realtime - t0) / 20.0;
        end
    endtask

    task measure_oe_width(output real w);
        realtime t0;
        begin
            @(negedge oe_pad); t0 = $realtime;
            @(posedge oe_pad); w = ($realtime - t0) / 20.0;
        end
    endtask

    task send_cmd(input [15:0] d, input [1:0] m, input [6:0] l, input [15:0] b);
        begin
            @(posedge clk);
            cmd_data <= d; cmd_mode <= m; cmd_le <= l; cmd_burst <= b;
            cmd_start <= 1'b1;
            @(posedge clk);
            cmd_start <= 1'b0;
            @(posedge clk);
            @(posedge clk);
            while (busy) @(posedge clk);
            repeat (4) @(posedge clk);
        end
    endtask

    task check_le_hist(input integer n_expect);
        integer k;
        reg bad;
        begin
            bad = 0;
            if (u_bfm0.le_hist_n < n_expect) begin
                fail("le_hist too short");
                $display("        got %0d entries expect >=%0d", u_bfm0.le_hist_n, n_expect);
            end else begin
                for (k = 0; k < n_expect; k = k + 1)
                    if (u_bfm0.le_hist[k] !== (((k % ROWS) == 0) ? 8'd5 : 8'd4)) begin
                        if (!bad) $display("        le_hist[%0d]=%0d exp %0d",
                                           k, u_bfm0.le_hist[k], ((k % ROWS) == 0) ? 5 : 4);
                        bad = 1;
                    end
                if (bad) fail("LE edge sequence != 5,4,4,...,4 cyclic");
            end
        end
    endtask

    // ---------------- 主流程 ----------------
    integer k0, e_row, e_ovl, base_err, idle_e;
    integer fc0;
    real    p1, p2, w1;
    reg [191:0] casc_exp, tmp192;
    reg [15:0]  tmpw;

    initial begin
        rstn = 1'b0;
        repeat (25) @(posedge clk);     // >100ns GSR
        rstn = 1'b1;
        repeat (10) @(posedge clk);
        bfm_en = 1'b1;
        repeat (5) @(posedge clk);

        //==================================================================
        // T1: 双沿主节拍 + 数据逐 bit (oe_window=48, ddr_slow=0)
        //==================================================================
        $display("---- T1: auto @ oe_window=48, ddr_slow=0 ----");
        min_dint = 1.0e9;
        disp_n = 0; cmp_en = 1'b1;
        fc0 = frame_count_o;
        auto_en = 1'b1;

        while (frame_count_o != fc0 + 2) @(posedge clk);
        // 行周期 5 连测
        for (k0 = 0; k0 < 5; k0 = k0 + 1) begin
            measure_period(r_period);
            if (r_period != 195.0) begin
                fail("T1 row period != 195 aclk");
                $display("        got %0.1f", r_period);
            end
        end
        $display("PASS T1a: row period = 195 aclk (5x)");
        // 每行 192 沿
        @(negedge oe_pad); e_a = dclk_edges;
        @(negedge oe_pad);
        if (dclk_edges - e_a !== 192) begin
            fail("T1 edges per row != 192");
            $display("        got %0d", dclk_edges - e_a);
        end else $display("PASS T1b: 192 DCLK edges per row");
        // 沿距 20ns (25MHz 双沿 = 50Mbps); ps→real 换算容差 ±0.01
        if (min_dint < 19.99 || min_dint > 20.01) begin
            fail("T1 min DCLK edge spacing != 20ns");
            $display("        got %0.6f ns", min_dint);
        end else $display("PASS T1c: DCLK edge spacing 20ns (50Mbps/lane)");
        // OE 宽度
        measure_oe_width(r_width);
        if (r_width != 48.0) begin
            fail("T1 OE low width != 48");
            $display("        got %0.1f", r_width);
        end else $display("PASS T1d: OE low = 48 aclk (oe_window=48)");
        // 帧周期寄存器
        while (frame_count_o != fc0 + 3) @(posedge clk);
        if (frame_period_o < 10525 || frame_period_o > 10535) begin
            fail("T1 frame_period != 10530 +/-5");
            $display("        got %0d", frame_period_o);
        end else
            $display("PASS T1e: frame_period = %0d aclk -> refresh %0.2f kHz",
                     frame_period_o, 50000.0/frame_period_o);
        // 数据比对已跑 >=2 帧
        if (disp_n < 2*ROWS) fail("T1 not enough OE falls compared");
        if (data_err != 0)   fail("T1 data mismatch vs fb golden");
        else $display("PASS T1f: %0d row-compares bit-exact (9 lane x %0d disp)", data_ok, disp_n);
        // setup/hold
        if (viol_sum(0) != 0) begin
            fail("T1 setup/hold or zero-LE violations");
            $display("        sum=%0d", viol_sum(0));
        end else
            $display("PASS T1g: 0 setup/hold violations, min data-edge margin %0.1f ns",
                     u_bfm0.min_margin);

        //==================================================================
        // T2(auto 部分): LE 序列 5,4,4,...,4
        //==================================================================
        $display("---- T2a: LE edge sequence (auto) ----");
        base_err = errors;
        check_le_hist(2*ROWS);
        if (errors == base_err)
            $display("PASS T2a: le_hist[0..%0d] = 5,4,4,...,4 cyclic (row0=5)", 2*ROWS-1);

        //==================================================================
        // T3: OE 窗箝位 + overlap 证据
        //==================================================================
        $display("---- T3: oe_window clamp ----");
        // overlap 证据: OE 低期间 DCLK 沿照跑 (显示 row N 同时移 row N+1)
        @(negedge oe_pad); e_ovl = dclk_edges;
        @(posedge oe_pad);
        if (dclk_edges - e_ovl < 40) begin
            fail("T3 no shifting during OE low (overlap broken)");
            $display("        edges in window = %0d", dclk_edges - e_ovl);
        end else
            $display("PASS T3a: %0d DCLK edges inside 48-aclk OE window (overlap live)",
                     dclk_edges - e_ovl);

        oe_window = 8'd96;  repeat (3) @(negedge oe_pad);
        measure_oe_width(r_width);
        if (r_width != 96.0)  begin fail("T3 width(96) != 96");  $display("        got %0.1f", r_width); end
        else $display("PASS T3b: oe_window=96 -> OE low 96 aclk");
        measure_period(r_period);
        if (r_period != 195.0) begin fail("T3 period(96) != 195"); $display("        got %0.1f", r_period); end

        oe_window = 8'd200; repeat (3) @(negedge oe_pad);
        measure_oe_width(r_width);
        if (r_width != 187.0) begin fail("T3 width(200) != 187 (upper clamp)"); $display("        got %0.1f", r_width); end
        else $display("PASS T3c: oe_window=200 clamped -> OE low 187 aclk");

        oe_window = 8'd1;   repeat (3) @(negedge oe_pad);
        measure_oe_width(r_width);
        if (r_width != 2.0)   begin fail("T3 width(1) != 2 (lower clamp)"); $display("        got %0.1f", r_width); end
        else $display("PASS T3d: oe_window=1 clamped -> OE low 2 aclk");

        oe_window = 8'd160; repeat (3) @(negedge oe_pad);
        measure_oe_width(r_width);
        measure_period(r_period);
        if (r_width != 160.0) begin fail("T3 width(160) != 160"); $display("        got %0.1f", r_width); end
        if (r_period <= 195.0 || r_period > 260.0) begin
            fail("T3 period(160): LWAIT expected (195 < period <= 260)");
            $display("        got %0.1f", r_period);
        end else
            $display("PASS T3e: oe_window=160 -> period %0.0f aclk (LWAIT=%0.0f, tail no longer hidden)",
                     r_period, r_period - 195.0);
        if (data_err != 0) fail("T3 data mismatch during clamp sweeps");

        oe_window = 8'd48;  repeat (3) @(negedge oe_pad);

        //==================================================================
        // T4: 行切换消隐 + row_first + 0x24 T_adv 拉长
        //==================================================================
        $display("---- T4: row blanking / row_first / 0x24 cfg ----");
        if (blank_err != 0) fail("T4 row_busy while OE low");
        else $display("PASS T4a: row_busy=1 => OE=1 held (%0d advances so far)", rf_total_n);
        if (rf_err != 0) fail("T4 row_first flag wrong");
        else $display("PASS T4b: row_first only on wrap-to-row0 (%0d of %0d advances)",
                      rf_first_n, rf_total_n);

        // 0x24: adv_high=200 -> T_adv=8+200+8=216 > 藏尾预算 -> LWAIT 出现
        row_cfg = 32'h0000_00C8;    // [7:0]=200, pre/hold 默认 8
        repeat (3) @(negedge oe_pad);
        measure_period(r_period);
        // T_adv = 8+200+8 = 216, 藏尾上限 ~146 → LWAIT ≈ 70+ (期望 195<p<=285)
        if (r_period <= 195.0 || r_period > 285.0) begin
            fail("T4 period(adv_high=200): expected LWAIT");
            $display("        got %0.1f", r_period);
        end else
            $display("PASS T4c: adv_high=200 -> period %0.0f aclk (LWAIT=%0.0f), still no data err",
                     r_period, r_period - 195.0);
        // 行 DCLK 宽度实测 = 200 拍
        @(posedge row_dclk); t_a = $realtime;
        @(negedge row_dclk);
        if (($realtime - t_a)/20.0 != 200.0) begin
            fail("T4 row_dclk high width != 200 aclk");
            $display("        got %0.1f", ($realtime - t_a)/20.0);
        end else $display("PASS T4d: row_dclk high = 200 aclk (0x24 runtime cfg)");
        row_cfg = 32'd0;
        repeat (3) @(negedge oe_pad);
        measure_period(r_period);
        if (r_period != 195.0) begin
            fail("T4 period not back to 195 after cfg restore");
            $display("        got %0.1f", r_period);
        end
        if (data_err != 0) fail("T4 data mismatch during adv stretch");
        if (blank_err != 0) fail("T4 blanking violated during adv stretch");

        //==================================================================
        // 停 auto -> idle 静默 + 手动路径 (T2 手动部分)
        //==================================================================
        $display("---- T2b: manual path (LE 3/11/12, marker, cascade) ----");
        auto_en = 1'b0;
        k0 = 0;
        while (eg_state_o != 3'd0 && k0 < 1000) begin @(posedge clk); k0 = k0 + 1; end
        if (eg_state_o != 3'd0) fail("T2 engine did not return to IDLE after auto_en=0");
        repeat (4) @(posedge clk);      // ODDR 输出级 1 拍延迟
        if (oe_pad !== 1'b1) fail("T2 OE not blanked after auto stop");
        cmp_en = 1'b0;
        repeat (10) @(posedge clk);

        // idle DCLK 静默
        idle_e = dclk_edges;
        repeat (1000) @(posedge clk);
        if (dclk_edges != idle_e) begin
            fail("T2 DCLK edges while idle");
            $display("        %0d extra edges", dclk_edges - idle_e);
        end else $display("PASS T2c: idle DCLK silent (0 edges / 1000 aclk)");

        // 普通锁存 LE=3 (奇数沿跨 DCLK 周期)
        send_cmd(16'hA5C3, 2'b00, 7'd3, 16'd0);
        if (u_bfm0.last_le !== 3) begin
            fail("T2 manual word LE!=3");
            $display("        got %0d", u_bfm0.last_le);
        end else $display("PASS T2d: manual word LE=3 edges decoded");
        if (b0_latch1[15:0] !== 16'hA5C3) begin
            fail("T2 manual word data not latched (LE-overlap tail shift broken)");
            $display("        latch1[15:0]=%04x exp a5c3", b0_latch1[15:0]);
        end else $display("PASS T2e: latch1[15:0]=A5C3 (data shifts during LE=H)");

        // WR_REG1 级联: BURST=10 词无 LE + 末词 LE=11 (12 颗同时锁存)
        base_err = errors;
        send_cmd(16'hBEEF, 2'b00, 7'd0, 16'd10);    // 11 词填链
        send_cmd(16'h1234, 2'b00, 7'd11, 16'd0);    // 末词 WR_REG1
        if (u_bfm0.last_le !== 11) begin
            fail("T2 WR_REG1 LE != 11");
            $display("        got %0d", u_bfm0.last_le);
        end
        if (u_bfm0.cfg1 !== 16'h1234) begin
            fail("T2 WR_REG1 value wrong");
            $display("        cfg1=%04x exp 1234", u_bfm0.cfg1);
        end
        casc_exp = {{11{16'hBEEF}}, 16'h1234};
        if (u_bfm0.sr !== casc_exp) begin
            fail("T2 cascade chain fill (11xBEEF + 1234) mismatch");
            $display("        sr=%048x", u_bfm0.sr);
        end
        if (errors == base_err && u_bfm0.cfg1 === 16'h1234)
            $display("PASS T2f: WR_REG1 cascade (BURST 10 + LE=11), chain = 11xBEEF+1234");

        // WR_REG2 LE=12
        send_cmd(16'h0F0F, 2'b00, 7'd12, 16'd0);
        if (u_bfm0.last_le !== 12 || u_bfm0.cfg2 !== 16'h0F0F) begin
            fail("T2 WR_REG2 LE=12 decode/value wrong");
            $display("        le=%0d cfg2=%04x", u_bfm0.last_le, u_bfm0.cfg2);
        end else $display("PASS T2g: WR_REG2 LE=12 -> cfg2=0F0F");

        // marker_LE: 7 沿 (奇数), 必含上升沿
        send_cmd(16'h0000, 2'b01, 7'd7, 16'd0);
        if (u_bfm0.last_le !== 7) begin
            fail("T2 marker_LE != 7 edges");
            $display("        got %0d", u_bfm0.last_le);
        end else $display("PASS T2h: marker_LE = 7 edges (odd, spans half DCLK period)");

        // per-chain word: 9 lane 各异, LE=4
        for (k0 = 0; k0 < 9; k0 = k0 + 1)
            chain_data_flat[k0*16 +: 16] = 16'h1111 * (k0 + 1);
        send_cmd(16'h0000, 2'b11, 7'd4, 16'd0);
        base_err = errors;
        for (k0 = 0; k0 < 9; k0 = k0 + 1) begin
            tmp192 = bfm_l1_f[k0*192 +: 192];
            tmpw   = 16'h1111 * (k0 + 1);
            if (tmp192[15:0] !== tmpw) begin
                fail("T2 per-chain word latch mismatch");
                $display("        lane %0d got %04x exp %04x", k0, tmp192[15:0], tmpw);
            end
        end
        if (errors == base_err) $display("PASS T2i: per-chain word x9 latched (LE=4)");

        // 0 沿 LE 保护全程复核
        if (viol_sum(0) != 0) fail("T2 zero-edge LE or setup/hold violation seen");
        else $display("PASS T2j: zero-edge-LE count = 0, all LE contain rising edge");

        // 行驱手动: config reg=3 -> 3+8=11 个 LCK 脉冲
        begin : t4_manual_row
            integer lck_n;
            lck_n = 0;
            fork
                begin : cnt_lck
                    forever @(posedge row_lck) lck_n = lck_n + 1;
                end
                begin
                    @(posedge clk);
                    row_man_type <= 1'b1; row_man_reg <= 4'd3; row_man_go <= 1'b1;
                    @(posedge clk);
                    row_man_go <= 1'b0;
                    @(posedge clk);
                    while (row_busy_o) @(posedge clk);
                    repeat (4) @(posedge clk);
                    disable cnt_lck;
                end
            join
            if (lck_n !== 11) begin
                fail("T4 manual config LCK pulses != reg+8");
                $display("        got %0d exp 11", lck_n);
            end else $display("PASS T4e: manual row config -> 11 LCK pulses (reg=3)");
        end

        //==================================================================
        // T5: ddr_slow 降级 (12.5MHz DCLK, 25Mbps, 双沿协议不变)
        //==================================================================
        $display("---- T5: ddr_slow fallback ----");
        ddr_slow = 1'b1;
        u_bfm0.le_hist_n = 0;
        min_dint = 1.0e9;
        disp_n = 0; cmp_en = 1'b1;
        fc0 = frame_count_o;
        auto_en = 1'b1;

        while (frame_count_o != fc0 + 2) @(posedge clk);
        for (k0 = 0; k0 < 3; k0 = k0 + 1) begin
            measure_period(r_period);
            if (r_period != 387.0) begin
                fail("T5 row period != 387 aclk (slow)");
                $display("        got %0.1f", r_period);
            end
        end
        $display("PASS T5a: row period = 387 aclk (3x)");
        @(negedge oe_pad); e_a = dclk_edges;
        @(negedge oe_pad);
        if (dclk_edges - e_a !== 192) begin
            fail("T5 edges per row != 192 (slow)");
            $display("        got %0d", dclk_edges - e_a);
        end else $display("PASS T5b: still 192 DCLK edges per row");
        if (min_dint < 39.99 || min_dint > 40.01) begin
            fail("T5 min edge spacing != 40ns (12.5MHz)");
            $display("        got %0.6f", min_dint);
        end else $display("PASS T5c: DCLK edge spacing 40ns (25Mbps fallback)");
        measure_oe_width(r_width);
        if (r_width != 96.0) begin
            fail("T5 OE width != 96 aclk (48 edges x2, duty preserved)");
            $display("        got %0.1f", r_width);
        end else $display("PASS T5d: OE low = 96 aclk = 48 edges (duty preserved)");
        while (frame_count_o != fc0 + 3) @(posedge clk);
        if (frame_period_o < 20893 || frame_period_o > 20903) begin
            fail("T5 frame_period != 20898 +/-5");
            $display("        got %0d", frame_period_o);
        end else
            $display("PASS T5e: frame_period = %0d aclk -> %0.2f kHz",
                     frame_period_o, 50000.0/frame_period_o);
        if (data_err != 0) fail("T5 data mismatch in slow mode");
        else $display("PASS T5f: %0d disp compares bit-exact in slow mode", disp_n);
        base_err = errors;
        check_le_hist(ROWS + 10);
        if (errors == base_err) $display("PASS T5g: LE sequence 5,4,4,... intact in slow mode");
        if (viol_sum(0) != 0) fail("T5 setup/hold violations in slow mode");

        auto_en = 1'b0;
        repeat (600) @(posedge clk);
        cmp_en = 1'b0;
        if (blank_err != 0) fail("final: row_busy/OE blanking violations");
        if (rf_err != 0)    fail("final: row_first violations");

        //==================================================================
        if (errors == 0)
            $display("=== TB RESULT: ALL CHECKS PASS (icnd2047_panel_core dual-edge engine) ===");
        else
            $display("=== TB RESULT: %0d FAILURES ===", errors);
        $finish;
    end

    // 看门狗
    initial begin
        #8_000_000;     // 8 ms
        fail("watchdog timeout");
        $display("        eg_state=%0d disp_n=%0d frame_count=%0d",
                 eg_state_o, disp_n, frame_count_o);
        $display("=== TB RESULT: TIMEOUT ===");
        $finish;
    end

endmodule
