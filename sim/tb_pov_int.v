//==============================================================================
// tb_pov_int.v — v5 icnd2049_panel_pov POV 集成自检 TB (xsim)
//
// 只测 v5 新增的集成缝 (v4.1 面板逻辑已有 tb_panel_fb 全绿, ddr_slice_fetch
// 已有 tb_ddr_fetch 全绿; AXI slave 行为模型抄 tb_ddr_fetch):
//
//  [1] fake 模式 slice_idx 经 AXI 读 0x10: 0→1→2→3→0 绕, 步进间隔 = fake_period
//  [2] 每次 fetch 首 AR 地址 = slice_base + idx*0x3000; burst 合法
//      (arlen<=15 / INCR / 4B beat / 不跨 4KB / fetch 内地址连续 / rlast 位置),
//      每 fetch 恰 2916 beats + 2916 笔 fb 写
//  [3] u_fetch fb 写口 (lane/addr/data) 逐笔 vs slave 内存 lane-major 布局 +
//      pov_en 下写口 mux 直通 + fetch 完成后 BRAM 内容抽查 (层级引用)
//  [4] R 0x00: [9]=pov_en, [10]=fetch_busy (开前 0 / 开后 1 / busy 采样 / 关后 0)
//  [5] 真传感器: 关 fake 只留 pov_en, spin_sync 3 个脉冲 (间隔 200000 / 195000,
//      脉宽 100 拍 > 1us 去抖): 第 2 脉冲后读 0x14 rev_period ≈ 200000,
//      slice_idx 回 0 并按 50000 拍/slice 自行步进 (AXI 轮询验证);
//      第 3 脉冲提前 5000 拍 (早于自然 wrap) 证明硬回零, 且 locked=1 (0x10[31])
//  [6] R 0x18 = {locked_ever,15'b0,slice_max} 锁存峰值 = 0x8000_0003
//
// 配置: 50 MHz aclk; n_slices=4, fake_period=2000, slice_base=0x0010_0034
// (故意非 4KB 对齐, 逼出 burst 截断); 面板 rows=54 + overlap + 25M DCLK +
// auto/use_fb 全程同跑 (真实 v5 工况, 扫描引擎读 BRAM 与取帧引擎写 BRAM 并存).
//
// 注意: fake_period(2000) < 单次 fetch 时长(~6000, 随机 AXI 延迟) → 引擎按设计
// 丢帧跳最新 (df_go 只在 idle 时接). 监视器按 df_go/df_last_slice 对账,
// 不假设每个 slice 值都被取到. 真传感器段 50000 拍/slice 则每片都取到,
// 窗口内 fetch 序列应恰为 idx 0,1,2,3.
//==============================================================================
`timescale 1ns / 1ps

module tb_pov_int;

    localparam [31:0] MEM_BASE      = 32'h0010_0000;
    localparam integer MEM_WORDS    = 65536;              // 256 KB
    localparam [31:0] SLICE_BASE_TB = 32'h0010_0034;      // 非 4KB 对齐
    localparam integer FRAME_WORDS  = 2916;               // 11664 B / 4
    localparam integer N_SLICES     = 4;
    localparam integer FAKE_PERIOD  = 2000;
    localparam integer REV1         = 200000;             // 脉冲 1→2 间隔
    localparam integer REV2         = 195000;             // 脉冲 2→3 间隔 (早于 wrap)
    localparam integer SLICE_STEP   = REV1 / N_SLICES;    // 50000

    // ---------------- clock / reset / 全局拍计 ----------------
    reg clk  = 1'b0;
    reg rstn = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    reg [31:0] cyc = 32'd0;
    always @(posedge clk) cyc <= cyc + 32'd1;

    // ---------------- AXI-Lite ----------------
    reg  [15:0] awaddr  = 16'b0;
    reg         awvalid = 1'b0;
    reg  [31:0] wdata_r = 32'b0;
    reg         wvalid  = 1'b0;
    reg         bready  = 1'b0;
    wire        awready, wready, bvalid;
    wire [1:0]  bresp;
    reg  [15:0] araddr  = 16'b0;
    reg         arvalid = 1'b0;
    reg         rready  = 1'b1;
    wire        arready, rvalid;
    wire [31:0] rdata;
    wire [1:0]  rresp;

    // ---------------- M_AXI (DUT 读主 ↔ TB 行为 slave) ----------------
    wire [31:0] m_araddr;
    wire [7:0]  m_arlen;
    wire [2:0]  m_arsize;
    wire [1:0]  m_arburst;
    wire        m_arlock;
    wire [3:0]  m_arcache;
    wire [2:0]  m_arprot;
    wire        m_arvalid;
    reg         m_arready;
    reg  [31:0] m_rdata;
    wire [1:0]  m_rresp = 2'b00;
    reg         m_rlast;
    reg         m_rvalid;
    wire        m_rready;

    // ---------------- 面板输出 (不检查, tb_panel_fb 已覆盖) ----------------
    wire       dclk, le, oe;
    wire [8:0] sdi;
    wire       i_sdi, i_dclk, i_rclk;

    reg spin_sync = 1'b0;

    icnd2049_panel_pov #(.DCLK_DIV(4)) dut (
        .s_axi_aclk    (clk),
        .s_axi_aresetn (rstn),
        .s_axi_awaddr  (awaddr),
        .s_axi_awprot  (3'b000),
        .s_axi_awvalid (awvalid),
        .s_axi_awready (awready),
        .s_axi_wdata   (wdata_r),
        .s_axi_wstrb   (4'hF),
        .s_axi_wvalid  (wvalid),
        .s_axi_wready  (wready),
        .s_axi_bresp   (bresp),
        .s_axi_bvalid  (bvalid),
        .s_axi_bready  (bready),
        .s_axi_araddr  (araddr),
        .s_axi_arprot  (3'b000),
        .s_axi_arvalid (arvalid),
        .s_axi_arready (arready),
        .s_axi_rdata   (rdata),
        .s_axi_rresp   (rresp),
        .s_axi_rvalid  (rvalid),
        .s_axi_rready  (rready),
        .dclk_out      (dclk),
        .le_out        (le),
        .oe_out        (oe),
        .sdi_out       (sdi),
        .icnd_sdi_out  (i_sdi),
        .icnd_dclk_out (i_dclk),
        .icnd_rclk_out (i_rclk),
        .spin_sync     (spin_sync),
        .m_axi_araddr  (m_araddr),
        .m_axi_arlen   (m_arlen),
        .m_axi_arsize  (m_arsize),
        .m_axi_arburst (m_arburst),
        .m_axi_arlock  (m_arlock),
        .m_axi_arcache (m_arcache),
        .m_axi_arprot  (m_arprot),
        .m_axi_arvalid (m_arvalid),
        .m_axi_arready (m_arready),
        .m_axi_rdata   (m_rdata),
        .m_axi_rresp   (m_rresp),
        .m_axi_rlast   (m_rlast),
        .m_axi_rvalid  (m_rvalid),
        .m_axi_rready  (m_rready)
    );

    integer errors = 0;

    task fail(input [511:0] msg);
        begin
            errors = errors + 1;
            $display("[%0t] FAIL: %0s", $time, msg);
        end
    endtask

    //-------------------------------------------------------------------
    // AXI-Lite 读写 task (抄 tb_panel_fb)
    //-------------------------------------------------------------------
    task axi_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            awaddr  <= addr;
            wdata_r <= data;
            awvalid <= 1'b1;
            wvalid  <= 1'b1;
            bready  <= 1'b1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 1'b0;
            wvalid  <= 1'b0;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
        end
    endtask

    task axi_read;
        input  [15:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            araddr  <= addr;
            arvalid <= 1'b1;
            rready  <= 1'b1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            while (!rvalid) @(posedge clk);
            data = rdata;
            @(posedge clk);
        end
    endtask

    //-------------------------------------------------------------------
    // 行为级 AXI read slave (抄 tb_ddr_fetch: 随机 ar/r 延迟, 单 outstanding)
    // mem[i] = (MEM_BASE + i*4) ^ 0xA5A5  可预测填充
    //-------------------------------------------------------------------
    reg [31:0] mem [0:MEM_WORDS-1];
    integer mi;
    initial begin
        for (mi = 0; mi < MEM_WORDS; mi = mi + 1)
            mem[mi] = (MEM_BASE + mi*4) ^ 32'h0000_A5A5;
    end

    reg        burst_active = 1'b0;
    reg [31:0] baddr;
    reg [7:0]  blen, bcnt;

    function [31:0] slv_read(input [31:0] a);
        begin
            if (a < MEM_BASE || a >= MEM_BASE + MEM_WORDS*4) begin
                errors = errors + 1;
                $display("[%0t] FAIL: slave read out of memory range %08x", $time, a);
                slv_read = 32'hDEAD_BEEF;
            end else
                slv_read = mem[(a - MEM_BASE) >> 2];
        end
    endfunction

    always @(posedge clk) begin
        if (!rstn)
            m_arready <= 1'b0;
        else if (m_arvalid && m_arready)
            m_arready <= 1'b0;
        else
            m_arready <= !burst_active && (({$random} % 3) == 0);
    end

    always @(posedge clk) begin
        if (!rstn) begin
            burst_active <= 1'b0;
            m_rvalid <= 1'b0;
            m_rlast  <= 1'b0;
            m_rdata  <= 32'd0;
        end else begin
            if (m_arvalid && m_arready) begin
                burst_active <= 1'b1;
                baddr <= m_araddr;
                blen  <= m_arlen;
                bcnt  <= 8'd0;
            end

            if (m_rvalid && m_rready) begin
                if (m_rlast) begin
                    m_rvalid <= 1'b0;
                    m_rlast  <= 1'b0;
                    burst_active <= 1'b0;
                end else begin
                    bcnt  <= bcnt + 8'd1;
                    baddr <= baddr + 32'd4;
                    if (({$random} % 3) != 0) begin
                        m_rdata <= slv_read(baddr + 32'd4);
                        m_rlast <= ((bcnt + 8'd1) == blen);
                    end else begin
                        m_rvalid <= 1'b0;
                    end
                end
            end else if (burst_active && !m_rvalid) begin
                if (({$random} % 3) != 0) begin
                    m_rvalid <= 1'b1;
                    m_rdata  <= slv_read(baddr);
                    m_rlast  <= (bcnt == blen);
                end
            end
        end
    end

    //-------------------------------------------------------------------
    // 监视器 [2][3]: fetch 记账 (df_go 层级引用触发, 每 fetch 一账)
    //-------------------------------------------------------------------
    integer    fetch_count   = 0;
    reg        have_fetch    = 1'b0;
    reg [31:0] fetch_base_cur = 32'd0;
    reg        expect_first_ar = 1'b0;
    integer    wr_cnt = 0, beats_cnt = 0;
    integer    f_idx [0:511];
    integer    f_cyc [0:511];

    task check_prev_fetch;   // 上一 fetch 收尾对账 (下一 df_go 或仿真末尾调)
        begin
            if (have_fetch) begin
                if (wr_cnt !== FRAME_WORDS) begin
                    fail("fb write count != 2916 for a fetch");
                    $display("        fetch %0d (idx=%0d): got %0d writes",
                             fetch_count-1, f_idx[fetch_count-1], wr_cnt);
                end
                if (beats_cnt !== FRAME_WORDS) begin
                    fail("AXI beats != 2916 for a fetch");
                    $display("        fetch %0d (idx=%0d): got %0d beats",
                             fetch_count-1, f_idx[fetch_count-1], beats_cnt);
                end
            end
        end
    endtask

    always @(posedge clk) if (rstn && dut.df_go) begin
        check_prev_fetch;
        if (dut.df_last_slice > 16'd3)
            fail("df_last_slice out of range (>3)");
        fetch_base_cur  = SLICE_BASE_TB + dut.df_last_slice * 32'h3000;
        expect_first_ar = 1'b1;
        wr_cnt    = 0;
        beats_cnt = 0;
        if (fetch_count < 512) begin
            f_idx[fetch_count] = dut.df_last_slice;
            f_cyc[fetch_count] = cyc;
        end
        fetch_count = fetch_count + 1;
        have_fetch  = 1'b1;
    end

    //-------------------------------------------------------------------
    // 监视器 [2]: AR 合法性 + 首地址 + fetch 内连续 + rlast 位置
    //-------------------------------------------------------------------
    reg [31:0] exp_ar_addr = 32'd0;
    reg [7:0]  cur_arlen   = 8'd0;
    integer    burst_beats = 0;

    always @(posedge clk) begin
        if (rstn && m_arvalid && m_arready) begin
            if (m_arlen > 8'd15)            fail("arlen > 15");
            if (m_arsize !== 3'b010)        fail("arsize != 4B");
            if (m_arburst !== 2'b01)        fail("arburst != INCR");
            if (m_araddr[1:0] !== 2'b00)    fail("araddr not word aligned");
            if (({20'b0, m_araddr[11:0]} + (m_arlen + 1)*4) > 32'd4096)
                                            fail("burst crosses 4KB boundary");
            if (expect_first_ar) begin
                if (m_araddr !== fetch_base_cur) begin
                    fail("first AR addr != slice_base + idx*0x3000");
                    $display("        got %08x expect %08x", m_araddr, fetch_base_cur);
                end
                expect_first_ar = 1'b0;
            end else if (m_araddr !== exp_ar_addr) begin
                fail("AR address not contiguous within fetch");
                $display("        got %08x expect %08x", m_araddr, exp_ar_addr);
            end
            exp_ar_addr = m_araddr + (m_arlen + 1)*4;
            cur_arlen   = m_arlen;
            burst_beats = 0;
        end
        if (rstn && m_rvalid && m_rready) begin
            burst_beats = burst_beats + 1;
            beats_cnt   = beats_cnt + 1;
            if (m_rlast && burst_beats !== cur_arlen + 1)
                fail("rlast beat count != arlen+1");
            if (!m_rlast && burst_beats > cur_arlen)
                fail("beats exceed arlen+1 without rlast");
        end
    end

    //-------------------------------------------------------------------
    // 监视器 [3]: u_fetch fb 写口逐笔 vs slave 内存 + pov 写口 mux 直通
    //-------------------------------------------------------------------
    integer    fbrem;
    reg [3:0]  exp_lane;
    reg [5:0]  exp_row;
    reg [2:0]  exp_word;
    reg [31:0] exp_data;

    always @(posedge clk) if (rstn && dut.df_fb_we) begin
        exp_lane = wr_cnt / 324;            // 324 word / lane
        fbrem    = wr_cnt % 324;
        exp_row  = fbrem / 6;
        exp_word = fbrem % 6;
        exp_data = mem[((fetch_base_cur - MEM_BASE) >> 2) + wr_cnt];
        if (dut.df_fb_wlane !== exp_lane) begin
            fail("df fb_wlane mismatch");
            $display("        wr %0d: got %0d expect %0d", wr_cnt, dut.df_fb_wlane, exp_lane);
        end
        if (dut.df_fb_waddr !== {exp_row, exp_word}) begin
            fail("df fb_waddr mismatch");
            $display("        wr %0d: got %03x expect %03x", wr_cnt, dut.df_fb_waddr, {exp_row, exp_word});
        end
        if (dut.df_fb_wdata !== exp_data) begin
            fail("df fb_wdata mismatch vs slave mem");
            $display("        wr %0d: got %08x expect %08x", wr_cnt, dut.df_fb_wdata, exp_data);
        end
        // pov_en 时引擎写必须直通 BRAM 写口 mux
        if (dut.pov_en !== 1'b1 || dut.fbw_we !== 1'b1 ||
            dut.fbw_data !== dut.df_fb_wdata || dut.fbw_lane !== dut.df_fb_wlane ||
            dut.fbw_addr !== dut.df_fb_waddr)
            fail("fb write mux not passing engine write while pov_en");
        wr_cnt = wr_cnt + 1;
    end

    //-------------------------------------------------------------------
    // spin_sync 脉冲 task (脉宽 100 拍 = 2us > 1us 去抖)
    //-------------------------------------------------------------------
    reg [31:0] pcyc = 32'd0;
    task do_pulse;
        begin
            @(negedge clk);
            spin_sync = 1'b1;
            pcyc = cyc;
            repeat (100) @(negedge clk);
            spin_sync = 1'b0;
        end
    endtask

    //-------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------
    reg [31:0] rd;
    integer    k, ntr, prev_i, d;
    integer    tr_cyc  [0:15];
    integer    tr2_cyc [0:7];
    integer    tr2_val [0:7];
    reg [31:0] t_en, c1, c2, c3;
    integer    nfA, cnt, base_w;
    reg        found, ord_ok;

    initial begin
        rstn = 1'b0;
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (5) @(posedge clk);

        //==== 面板基础配置: rows=54 + cfg_we|dclk_fast|overlap + oe_window=48 + 消隐
        // 0x8000_0000 | (1<<29)|(1<<28)|(1<<27) | (54<<16) | (48<<8) | 1
        axi_write(16'h000C, 32'hB836_3001);
        // auto 自主扫描 + use_fb (真实 v5 工况: 扫描引擎全程读 BRAM)
        axi_write(16'h000C, 32'hC100_0003);

        //==== [4] pov 开启前 status: [10:9]=00, [8]=locked=0; 面板配置位 sanity
        axi_read(16'h0000, rd);
        if (rd[10:9] !== 2'b00) fail("[4] 0x00 [10:9] != 00 before pov_en");
        if (rd[8]    !== 1'b0)  fail("[4] 0x00 [8]=locked != 0 before pov");
        if (rd[7:6]  !== 2'b11) fail("cfg sanity: [7]=dclk_fast/[6]=overlap != 11");
        if (rd[4]    !== 1'b1)  fail("cfg sanity: [4]=auto_en != 1");
        if (errors == 0) $display("PASS [4a] status pre-pov: [10:9]=00 [8]=0, cfg bits ok");

        //==== POV 三寄存器: fake_period, slice_base, 然后 fake+pov 开
        axi_write(16'h0014, FAKE_PERIOD);
        axi_write(16'h0018, SLICE_BASE_TB);
        axi_write(16'h0010, (N_SLICES << 16) | 32'h3);   // pov_en + fake_en
        t_en = cyc;

        //==== [1] fake slice_idx 经 AXI 轮询: 顺序 + 节奏
        prev_i = 0;
        ntr    = 0;
        while (ntr < 9 && (cyc - t_en) < 32'd40000) begin
            axi_read(16'h0010, rd);
            if (rd[15:0] != prev_i[15:0]) begin
                if (rd[15:0] !== ((prev_i + 1) % N_SLICES)) begin
                    fail("[1] slice_idx order broken");
                    $display("        prev %0d -> got %0d", prev_i, rd[15:0]);
                end
                tr_cyc[ntr] = cyc;
                prev_i = rd[15:0];
                ntr = ntr + 1;
            end
        end
        if (ntr !== 9) fail("[1] timeout: <9 slice transitions in 40000 cycles");
        k = errors;
        for (d = 1; d < 9; d = d + 1) begin
            if ((tr_cyc[d] - tr_cyc[d-1] > FAKE_PERIOD + 64) ||
                (tr_cyc[d] - tr_cyc[d-1] < FAKE_PERIOD - 64)) begin
                fail("[1] fake step interval != fake_period");
                $display("        trans %0d->%0d: %0d cycles (exp %0d +/-64)",
                         d-1, d, tr_cyc[d]-tr_cyc[d-1], FAKE_PERIOD);
            end
        end
        axi_read(16'h0010, rd);
        if (rd[31] !== 1'b1) fail("[1] locked (0x10[31]) != 1 in fake mode");
        if (errors == k && ntr == 9)
            $display("PASS [1] fake: 9 transitions 0->1->2->3 wrap, interval %0d~%0d (exp %0d), locked=1",
                     tr_cyc[1]-tr_cyc[0], tr_cyc[8]-tr_cyc[7], FAKE_PERIOD);

        //==== [4] pov 开启后: [9]=1, [8]=1(fake locked), 轮询捕捉 [10]=fetch_busy=1
        k = errors;
        found = 1'b0;
        for (d = 0; d < 300 && !found; d = d + 1) begin
            axi_read(16'h0000, rd);
            if (rd[9] !== 1'b1) fail("[4] 0x00 [9]=pov_en != 1 after enable");
            if (rd[8] !== 1'b1) fail("[4] 0x00 [8]=locked != 1 in fake");
            if (rd[10] === 1'b1) found = 1'b1;
        end
        if (!found) fail("[4] never observed 0x00 [10]=fetch_busy=1 in fake phase");
        if (errors == k) $display("PASS [4b] status during fake+pov: [9]=1 [8]=1, [10]=busy observed");

        nfA = fetch_count;
        if (nfA < 2) fail("[2] fewer than 2 fetches during fake phase");
        $display("[%0t] fake phase done: %0d fetches so far", $time, nfA);

        //==================================================================
        //==== [5] 真传感器路径: 关 fake, 只留 pov_en
        //==================================================================
        axi_write(16'h0010, (N_SLICES << 16) | 32'h1);   // fake off, pov on

        // 垫到 rev_cnt >= MIN_REV_CYC(50000) 再打第一个脉冲
        while (cyc < 32'd70000) @(posedge clk);

        do_pulse; c1 = pcyc;
        $display("[%0t] pulse 1 @ cyc %0d", $time, c1);
        while (cyc < c1 + REV1) @(posedge clk);
        do_pulse; c2 = pcyc;
        $display("[%0t] pulse 2 @ cyc %0d (interval %0d)", $time, c2, c2 - c1);

        repeat (200) @(posedge clk);
        // rev_period ≈ 200000
        axi_read(16'h0014, rd);
        k = errors;
        if (rd > c2 - c1 + 32'd16 || rd + 32'd16 < c2 - c1) begin
            fail("[5] rev_period != measured pulse interval");
            $display("        got %0d expect %0d +/-16", rd, c2 - c1);
        end
        if (rd > REV1 + 300 || rd + 300 < REV1)
            fail("[5] rev_period not ~200000");
        if (errors == k) $display("PASS [5a] rev_period = %0d (exp ~%0d)", rd, REV1);

        // 第二脉冲后 slice_idx 回 0
        axi_read(16'h0010, rd);
        if (rd[15:0] !== 16'd0) begin
            fail("[5] slice_idx not 0 right after pulse 2");
            $display("        got %0d", rd[15:0]);
        end else $display("PASS [5b] slice_idx = 0 after pulse 2");

        // [4] fetch(idx0) 进行中读 busy
        axi_read(16'h0000, rd);
        if (rd[9]  !== 1'b1) fail("[4] 0x00 [9] != 1 in sensor phase");
        if (rd[10] !== 1'b1) fail("[4] 0x00 [10]=fetch_busy != 1 during idx0 fetch");
        else $display("PASS [4c] [10]=fetch_busy=1 sampled during sensor-phase fetch");

        // 随后自行步进: 50000 拍/slice, 轮询 3 次转变 (1,2,3)
        prev_i = 0;
        ntr    = 0;
        while (ntr < 3 && (cyc - c2) < 32'd170000) begin
            axi_read(16'h0010, rd);
            if (rd[15:0] != prev_i[15:0]) begin
                tr2_val[ntr] = rd[15:0];
                tr2_cyc[ntr] = cyc;
                prev_i = rd[15:0];
                ntr = ntr + 1;
            end
        end
        k = errors;
        if (ntr !== 3) fail("[5] <3 slice transitions after pulse 2");
        else begin
            for (d = 0; d < 3; d = d + 1) begin
                if (tr2_val[d] !== d + 1) begin
                    fail("[5] sensor-phase slice order broken");
                    $display("        trans %0d: got %0d exp %0d", d, tr2_val[d], d+1);
                end
                if ((tr2_cyc[d] > c2 + (d+1)*SLICE_STEP + 200) ||
                    (tr2_cyc[d] + 200 < c2 + (d+1)*SLICE_STEP)) begin
                    fail("[5] sensor-phase step timing != rev_period/n_slices");
                    $display("        trans to %0d @ +%0d cycles (exp %0d +/-200)",
                             d+1, tr2_cyc[d]-c2, (d+1)*SLICE_STEP);
                end
            end
        end
        if (errors == k && ntr == 3)
            $display("PASS [5c] free-run steps 1,2,3 at +%0d/+%0d/+%0d cycles (exp %0d/slice)",
                     tr2_cyc[0]-c2, tr2_cyc[1]-c2, tr2_cyc[2]-c2, SLICE_STEP);

        // 第三脉冲: 提前 5000 拍 (idx=3, 自然 wrap 还没到) → 硬回零 + locked
        while (cyc < c2 + REV2) @(posedge clk);
        do_pulse; c3 = pcyc;
        $display("[%0t] pulse 3 @ cyc %0d (interval %0d)", $time, c3, c3 - c2);
        repeat (300) @(posedge clk);

        axi_read(16'h0010, rd);
        k = errors;
        if (rd[15:0] !== 16'd0) begin
            fail("[5] slice_idx not hard-reset to 0 by pulse 3");
            $display("        got %0d", rd[15:0]);
        end
        if (rd[31] !== 1'b1) fail("[5] locked (0x10[31]) != 1 after two stable revs");
        if (errors == k) $display("PASS [5d] pulse 3: slice_idx hard-reset to 0 (pre-wrap), locked=1");

        //==== [6] 0x18 峰值锁存
        axi_read(16'h0018, rd);
        if (rd !== 32'h8000_0003) begin
            fail("[6] 0x18 != {locked_ever=1, slice_max=3}");
            $display("        got %08x expect 80000003", rd);
        end else $display("PASS [6] 0x18 = 80000003 (locked_ever=1, slice_max=3)");

        //==== [3] fetch(idx0) 完成后 BRAM 内容抽查 (层级引用 generate BRAM)
        while (dut.df_busy !== 1'b1 && cyc < c3 + 32'd2000) @(posedge clk);
        if (dut.df_busy !== 1'b1) fail("[3] no fetch running after pulse 3");
        while (dut.df_busy) @(posedge clk);
        repeat (4) @(posedge clk);
        base_w = (SLICE_BASE_TB - MEM_BASE) >> 2;   // slice 0
        k = errors;
        if (dut.g_fb[0].mem[9'd0]   !== mem[base_w + 0])    fail("[3] BRAM lane0 row0 w0 mismatch");
        if (dut.g_fb[0].mem[9'd429] !== mem[base_w + 323])  fail("[3] BRAM lane0 row53 w5 mismatch");
        if (dut.g_fb[4].mem[9'd219] !== mem[base_w + 1461]) fail("[3] BRAM lane4 row27 w3 mismatch");
        if (dut.g_fb[8].mem[9'd0]   !== mem[base_w + 2592]) fail("[3] BRAM lane8 row0 w0 mismatch");
        if (dut.g_fb[8].mem[9'd429] !== mem[base_w + 2915]) fail("[3] BRAM lane8 row53 w5 mismatch");
        if (errors == k) $display("PASS [3a] BRAM spot check: slice-0 frame landed in fb (5 cells, 4 corners+mid)");

        //==== 关 pov: [10:9] 回 0
        axi_write(16'h0010, (N_SLICES << 16));   // pov off
        repeat (10) @(posedge clk);
        axi_read(16'h0000, rd);
        if (rd[10:9] !== 2'b00) fail("[4] 0x00 [10:9] != 00 after pov off");
        else $display("PASS [4d] status after pov off: [10:9]=00");

        //==== 收尾: 最后一个 fetch 对账 + 传感器窗口 fetch 序列 = 0,1,2,3
        check_prev_fetch;
        have_fetch = 1'b0;

        cnt    = 0;
        ord_ok = 1'b1;
        for (k = 0; k < fetch_count && k < 512; k = k + 1) begin
            if (f_cyc[k] >= c2 - 5 && f_cyc[k] < c3) begin
                if (cnt < 4 && f_idx[k] !== cnt) ord_ok = 1'b0;
                cnt = cnt + 1;
            end
        end
        if (cnt !== 4 || !ord_ok) begin
            fail("[2] sensor-window fetch sequence != idx 0,1,2,3");
            $display("        %0d fetches in window:", cnt);
            for (k = 0; k < fetch_count && k < 512; k = k + 1)
                if (f_cyc[k] >= c2 - 5 && f_cyc[k] < c3)
                    $display("        fetch idx=%0d @ cyc %0d (+%0d)", f_idx[k], f_cyc[k], f_cyc[k]-c2);
        end else
            $display("PASS [2/3] %0d total fetches all verified (first-AR addr, AR legal, 2916 beats+writes, data==slave mem); sensor window = idx 0,1,2,3", fetch_count);

        //==== 总结
        if (errors == 0)
            $display("=== TB RESULT: ALL CHECKS PASS (v5 POV integration: fake/real spin, DDR fetch, regs) ===");
        else
            $display("=== TB RESULT: %0d FAILURES ===", errors);
        $finish;
    end

    // 看门狗
    initial begin
        #40_000_000;   // 40 ms
        fail("watchdog timeout");
        $display("        cyc=%0d fetch_count=%0d df_busy=%b at_slice_idx=%0d",
                 cyc, fetch_count, dut.df_busy, dut.at_slice_idx);
        $display("=== TB RESULT: TIMEOUT ===");
        $finish;
    end

endmodule
