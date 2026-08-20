//==============================================================================
// tb_pov_dual.v — pov_dual_top 双屏顶层自检 TB (xsim)
//
// 必验清单 (任务书 ①..⑤):
//  ① dual_en=0 回归 v5 行为: 只发 A fetch / fb_B 零写入 / B 引擎消隐 /
//     0x00[11:12]=0 / 0x10 idx 步进 / pair_miss=0
//  ② A/B idx 相位差 = PHASE_B 且 mod n_slices 正确: n_slices=352 (非常规片数),
//     默认 PHASE_B=180 + 写 0x1C=351 强制每对都过 wrap; 0x1C idx_B 回读核相位
//  ③ pair 级翻页原子性: slice_base 写 0x18 落在 pair 各阶段 (含 A 尾部
//     words_left=5 的 A→B 切换竞态窗) — B 首 AR 基址必须 == 本 pair A 的快照;
//     写落定后的下一 pair 用新基址
//  ④ fetch 256-beat 突发对账: arlen≤255 / 观察到 255 (256-beat 真用上) /
//     不跨 4KB / fetch 内地址连续 / 每 fetch 2916 beats+2916 fb 写且 ≤16 burst;
//     带宽账: 现实延迟 slave 下 A+B pair 全程 < 10685 拍 (213.7 µs), 打印余量
//  ⑤ pair_miss 计数: 正常速率恒 0; fake_period < pair 时长 → 计数增长;
//     恢复后计数冻结
//  另: ⓪ 0x20 oe_window_B 旋钮 + fb_sel_b AXI fb 窗直灌 fb_B (02 §7 阶段1)
//
// 数据模型: AXI read slave 返回 data = addr ^ 0xA5A5A5A5 (纯地址函数, 无内存
// 上限 → 352 片 × 0x3000 大跨度直接可测). 两种延迟模式:
//   random (回归 v5 tb 套路: arready p=1/3, rvalid 2/3) / fast (AR 6 拍 +
//   首数据 12 拍 + 之后连拍, 近似 HP0 实测延迟, 用于带宽账).
//==============================================================================
`timescale 1ns / 1ps

module tb_pov_dual;

    localparam [31:0] BASE1       = 32'h1000_0034;   // 非 4KB 对齐, 逼截断
    localparam [31:0] BASE2       = 32'h1050_0034;
    localparam integer FRAME_WORDS = 2916;           // 11664 B / 4
    localparam integer BUDGET_CYC  = 10685;          // 213.7 µs @ 50 MHz

    // ---------------- clock / reset / 拍计 ----------------
    reg clk  = 1'b0;
    reg rstn = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    integer cyci = 0;
    always @(posedge clk) cyci = cyci + 1;

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

    // ---------------- M_AXI ----------------
    wire [31:0] m_araddr;
    wire [7:0]  m_arlen;
    wire [2:0]  m_arsize;
    wire [1:0]  m_arburst;
    wire        m_arlock, m_arvalid;
    wire [3:0]  m_arcache;
    wire [2:0]  m_arprot;
    reg         m_arready;
    reg  [31:0] m_rdata;
    wire [1:0]  m_rresp = 2'b00;
    reg         m_rlast;
    reg         m_rvalid;
    wire        m_rready;

    // ---------------- 屏引脚 ----------------
    wire       dclk_a, le_a, oe_a;
    wire [8:0] sdi_a;
    wire       isdi_a, idclk_a, irclk_a;
    wire       dclk_b, le_b, oe_b;
    wire [8:0] sdi_b;
    wire       isdi_b, idclk_b, irclk_b;

    reg spin_sync = 1'b0;

    pov_dual_top #(.DCLK_DIV(4)) dut (
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
        .dclk_out      (dclk_a),
        .le_out        (le_a),
        .oe_out        (oe_a),
        .sdi_out       (sdi_a),
        .icnd_sdi_out  (isdi_a),
        .icnd_dclk_out (idclk_a),
        .icnd_rclk_out (irclk_a),
        .dclk_out_2    (dclk_b),
        .le_out_2      (le_b),
        .oe_out_2      (oe_b),
        .sdi_out_2     (sdi_b),
        .icnd_sdi_out_2(isdi_b),
        .icnd_dclk_out_2(idclk_b),
        .icnd_rclk_out_2(irclk_b),
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
    // AXI-Lite 读写 task (抄 tb_pov_int)
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
    // 行为级 AXI read slave: data = f(addr), random / fast 两档延迟
    //-------------------------------------------------------------------
    function [31:0] slv_data(input [31:0] a);
        slv_data = a ^ 32'hA5A5_A5A5;
    endfunction

    reg        fast_mode = 1'b0;
    reg        burst_active = 1'b0;
    reg [31:0] baddr;
    reg [7:0]  blen, bcnt;
    reg [3:0]  ar_wait = 4'd0;
    reg [3:0]  r_wait  = 4'd0;

    // AR: fast = 6 拍固定延迟; random = p(1/3)/拍
    always @(posedge clk) begin
        if (!rstn) begin
            m_arready <= 1'b0; ar_wait <= 4'd0;
        end else if (m_arvalid && m_arready) begin
            m_arready <= 1'b0; ar_wait <= 4'd0;
        end else if (m_arvalid && !burst_active && !m_arready) begin
            if (fast_mode) begin
                if (ar_wait == 4'd5) begin m_arready <= 1'b1; ar_wait <= 4'd0; end
                else ar_wait <= ar_wait + 4'd1;
            end else begin
                m_arready <= (({$random} % 3) == 0);
            end
        end else begin
            m_arready <= 1'b0; ar_wait <= 4'd0;
        end
    end

    // R: fast = 首拍 12 拍延迟后连拍; random = 2/3 占空
    always @(posedge clk) begin
        if (!rstn) begin
            burst_active <= 1'b0;
            m_rvalid <= 1'b0;
            m_rlast  <= 1'b0;
            m_rdata  <= 32'd0;
            r_wait   <= 4'd0;
        end else begin
            if (m_arvalid && m_arready) begin
                burst_active <= 1'b1;
                baddr  <= m_araddr;
                blen   <= m_arlen;
                bcnt   <= 8'd0;
                r_wait <= 4'd0;
            end

            if (m_rvalid && m_rready) begin
                if (m_rlast) begin
                    m_rvalid <= 1'b0;
                    m_rlast  <= 1'b0;
                    burst_active <= 1'b0;
                end else begin
                    bcnt  <= bcnt + 8'd1;
                    baddr <= baddr + 32'd4;
                    if (fast_mode || ({$random} % 3) != 0) begin
                        m_rdata <= slv_data(baddr + 32'd4);
                        m_rlast <= ((bcnt + 8'd1) == blen);
                    end else begin
                        m_rvalid <= 1'b0;
                    end
                end
            end else if (burst_active && !m_rvalid) begin
                if (fast_mode) begin
                    if (r_wait == 4'd11) begin
                        r_wait  <= 4'd0;
                        m_rvalid <= 1'b1;
                        m_rdata  <= slv_data(baddr);
                        m_rlast  <= (bcnt == blen);
                    end else r_wait <= r_wait + 4'd1;
                end else if (({$random} % 3) != 0) begin
                    m_rvalid <= 1'b1;
                    m_rdata  <= slv_data(baddr);
                    m_rlast  <= (bcnt == blen);
                end
            end
        end
    end

    //-------------------------------------------------------------------
    // TB 模型变量 (激励写, 监视器读)
    //-------------------------------------------------------------------
    reg        tb_dual    = 1'b0;    // 期望 dual_en
    integer    tb_nslices = 4;
    integer    tb_phase   = 180;
    reg [31:0] tb_base_model = BASE1;
    integer    last_base_wr_cyc = -1000;
    reg        allow_b_wr = 1'b0;    // fb_B 允许被写 (dual 或 fb_sel_b 阶段)
    reg        budget_en  = 1'b0;    // fast_mode 下测 pair 时长

    //-------------------------------------------------------------------
    // 监视器: fetch / pair 记账 (df_go 白盒触发)
    //-------------------------------------------------------------------
    integer    fetch_count = 0, pairs_done = 0;
    reg        fetch_open = 1'b0;
    reg        cur_tgt;
    reg [8:0]  cur_idx;
    reg [31:0] cur_base, exp_first_ar;
    integer    wr_cnt = 0, beats_cnt = 0, bursts_cnt = 0;
    integer    fetch_start = 0;
    integer    fetchA_dur_max = 0;

    reg        pair_open = 1'b0;
    reg [8:0]  pairA_idx;
    reg [31:0] pair_base;
    integer    pair_start_cyc = 0;
    integer    pair_dur_max = 0, pair_dur_min = 1000000000;
    reg [8:0]  last_pairA_idx, last_pairB_idx;
    reg [31:0] last_pair_base;

    reg        done_d = 1'b0;
    reg        done_tgt = 1'b0;
    integer    exp_idx_b;

    // AR 监视器变量 (声明提前, pair 监视器要清 first_ar_pending)
    reg        first_ar_pending = 1'b0;
    reg [31:0] exp_next_ar = 32'd0;
    reg [7:0]  cur_arlen_m = 8'd0;
    integer    burst_beats_m = 0;
    integer    max_arlen_all = 0;

    task close_fetch;
        begin
            if (fetch_open) begin
                if (wr_cnt !== FRAME_WORDS) begin
                    fail("fb write count != 2916 for a fetch");
                    $display("        tgt=%0d idx=%0d: got %0d writes", cur_tgt, cur_idx, wr_cnt);
                end
                if (beats_cnt !== FRAME_WORDS) begin
                    fail("AXI beats != 2916 for a fetch");
                    $display("        tgt=%0d idx=%0d: got %0d beats", cur_tgt, cur_idx, beats_cnt);
                end
                if (bursts_cnt < 12 || bursts_cnt > 16) begin
                    fail("bursts per fetch out of 12..16 (256-beat not effective?)");
                    $display("        got %0d bursts", bursts_cnt);
                end
                fetch_open = 1'b0;
            end
        end
    endtask

    always @(posedge clk) begin
        if (rstn) begin
            // 上一 fetch 收尾 (done 延迟 1 拍, 让最后一笔 fb 写先入账)
            if (done_d) begin
                close_fetch;
                if (done_tgt == 1'b0 && !tb_dual) begin
                    // 单屏: A done 即 "pair" 完
                    pair_open      = 1'b0;
                    pairs_done     = pairs_done + 1;
                    last_pairA_idx = pairA_idx;
                    last_pair_base = pair_base;
                end else if (done_tgt == 1'b1) begin
                    pair_open      = 1'b0;
                    pairs_done     = pairs_done + 1;
                    last_pairA_idx = pairA_idx;
                    last_pair_base = pair_base;
                    if (budget_en) begin
                        if (cyci - pair_start_cyc > pair_dur_max) pair_dur_max = cyci - pair_start_cyc;
                        if (cyci - pair_start_cyc < pair_dur_min) pair_dur_min = cyci - pair_start_cyc;
                        if (cyci - pair_start_cyc > BUDGET_CYC) begin
                            fail("pair (A+B) exceeded 213.7us slice budget");
                            $display("        %0d cycles > %0d", cyci - pair_start_cyc, BUDGET_CYC);
                        end
                    end
                end
                if (budget_en && done_tgt == 1'b0 &&
                    (cyci - fetch_start) > fetchA_dur_max)
                    fetchA_dur_max = cyci - fetch_start;
            end

            // 新 fetch 开账
            if (dut.df_go) begin
                if (fetch_open) fail("df_go while previous fetch still open");
                cur_tgt      = dut.df_tgt;
                cur_idx      = dut.df_idx;
                cur_base     = dut.base_lat;
                exp_first_ar = cur_base + cur_idx * 32'h3000;
                wr_cnt       = 0;
                beats_cnt    = 0;
                bursts_cnt   = 0;
                fetch_start  = cyci;
                first_ar_pending = 1'b1;
                fetch_open   = 1'b1;
                fetch_count  = fetch_count + 1;
                if (cur_tgt == 1'b0) begin
                    if (pair_open) fail("new A fetch while pair still open (missing B?)");
                    pairA_idx      = cur_idx;
                    pair_base      = cur_base;
                    pair_start_cyc = cyci;
                    pair_open      = 1'b1;
                    // 写落定 (>30 拍前) 后的 pair 必须用最新 slice_base
                    if ((cyci - last_base_wr_cyc > 30) && cur_base !== tb_base_model) begin
                        fail("pair base != last settled slice_base write");
                        $display("        got %08x expect %08x", cur_base, tb_base_model);
                    end
                end else begin
                    if (!tb_dual)   fail("target-B fetch while dual_en=0 model");
                    if (!pair_open) fail("B fetch without preceding A");
                    // ③ 原子性: B 与本 pair 的 A 必须同基址 (同帧)
                    if (cur_base !== pair_base) begin
                        fail("ATOMICITY: B fetch base != A fetch base within pair");
                        $display("        A=%08x B=%08x", pair_base, cur_base);
                    end
                    // ② 相位: idx_B = (idx_A + PHASE_B) mod n_slices
                    exp_idx_b = (pairA_idx + tb_phase) % tb_nslices;
                    if (cur_idx !== exp_idx_b[8:0]) begin
                        fail("idx_B != (idx_A + PHASE_B) mod n_slices");
                        $display("        idx_A=%0d phase=%0d n=%0d: got %0d expect %0d",
                                 pairA_idx, tb_phase, tb_nslices, cur_idx, exp_idx_b);
                    end
                    last_pairB_idx = cur_idx;
                end
            end

            done_d <= dut.df_done_w;
            if (dut.df_done_w) done_tgt <= cur_tgt;
        end else begin
            done_d <= 1'b0;
        end
    end

    //-------------------------------------------------------------------
    // 监视器: AR 合法性 + 256-beat 对账 ④
    //-------------------------------------------------------------------
    always @(posedge clk) begin
        if (rstn && m_arvalid && m_arready) begin
            if (m_arsize !== 3'b010)     fail("arsize != 4B");
            if (m_arburst !== 2'b01)     fail("arburst != INCR");
            if (m_araddr[1:0] !== 2'b00) fail("araddr not word aligned");
            if (({20'b0, m_araddr[11:0]} + (m_arlen + 1)*4) > 32'd4096)
                fail("burst crosses 4KB boundary");
            if (first_ar_pending) begin
                if (m_araddr !== exp_first_ar) begin
                    fail("first AR != pair_base + idx*0x3000");
                    $display("        got %08x expect %08x", m_araddr, exp_first_ar);
                end
                first_ar_pending = 1'b0;
            end else if (m_araddr !== exp_next_ar) begin
                fail("AR address not contiguous within fetch");
                $display("        got %08x expect %08x", m_araddr, exp_next_ar);
            end
            exp_next_ar = m_araddr + (m_arlen + 1)*4;
            if (m_arlen > max_arlen_all) max_arlen_all = m_arlen;
            cur_arlen_m   = m_arlen;
            burst_beats_m = 0;
            bursts_cnt    = bursts_cnt + 1;
        end
        if (rstn && m_rvalid && m_rready) begin
            burst_beats_m = burst_beats_m + 1;
            beats_cnt     = beats_cnt + 1;
            if (m_rlast && burst_beats_m !== cur_arlen_m + 1)
                fail("rlast beat count != arlen+1");
            if (!m_rlast && burst_beats_m > cur_arlen_m)
                fail("beats exceed arlen+1 without rlast");
        end
    end

    //-------------------------------------------------------------------
    // 监视器: fetch fb 写口逐笔 (lane-major 布局 + target + 数据)
    //-------------------------------------------------------------------
    integer flane, frem, fw_row, fw_word;
    always @(posedge clk) if (rstn && dut.df_fb_we) begin
        if (!fetch_open) fail("fetch fb write with no open fetch");
        if (dut.df_fb_wtgt !== cur_tgt) fail("fb_wtarget != fetch target");
        flane   = wr_cnt / 324;
        frem    = wr_cnt % 324;
        fw_row  = frem / 6;
        fw_word = frem % 6;
        if (dut.df_fb_wlane !== flane[3:0]) begin
            fail("df fb_wlane mismatch");
            $display("        wr %0d: got %0d expect %0d", wr_cnt, dut.df_fb_wlane, flane);
        end
        if (dut.df_fb_waddr !== {fw_row[5:0], fw_word[2:0]}) begin
            fail("df fb_waddr mismatch");
            $display("        wr %0d: got %03x expect %03x",
                     wr_cnt, dut.df_fb_waddr, {fw_row[5:0], fw_word[2:0]});
        end
        if (dut.df_fb_wdata !== slv_data(exp_first_ar + wr_cnt*4)) begin
            fail("df fb_wdata mismatch vs addr-hash");
            $display("        wr %0d: got %08x expect %08x",
                     wr_cnt, dut.df_fb_wdata, slv_data(exp_first_ar + wr_cnt*4));
        end
        wr_cnt = wr_cnt + 1;
    end

    // fb_B 写守卫 ①
    always @(posedge clk)
        if (rstn && dut.fbB_we && !allow_b_wr)
            fail("fb_B written while single-screen (dual_en=0, fb_sel_b=0)");

    //-------------------------------------------------------------------
    // B 屏消隐检查 ①
    //-------------------------------------------------------------------
    task check_b_blank(input integer ncyc);
        integer i; reg ok;
        begin
            ok = 1'b1;
            for (i = 0; i < ncyc; i = i + 1) begin
                @(posedge clk);
                if (oe_b !== 1'b1 || sdi_b !== 9'd0 || dclk_b !== 1'b0) ok = 1'b0;
            end
            if (!ok) fail("engine B not blanked (oe/sdi/dclk active)");
        end
    endtask

    //-------------------------------------------------------------------
    // ③ 翻页竞态: 等 A fetch 剩余 <= thresh 时写 0x18
    //-------------------------------------------------------------------
    task base_race_write(input [31:0] nb, input integer thresh);
        integer t0;
        begin
            t0 = cyci;
            while (!(pair_open && fetch_open && cur_tgt == 1'b0 &&
                     dut.u_fetch.words_left != 12'd0 &&
                     dut.u_fetch.words_left <= thresh[11:0])) begin
                @(posedge clk);
                if (cyci - t0 > 200000) begin
                    fail("base_race_write: timeout waiting for A-fetch window");
                    disable base_race_write;
                end
            end
            axi_write(16'h0018, nb);
            tb_base_model    = nb;
            last_base_wr_cyc = cyci;
            $display("[%0t] race: wrote slice_base=%08x at A words_left<=%0d",
                     $time, nb, thresh);
        end
    endtask

    //-------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------
    reg [31:0] rd, rd2;
    integer    k, f0, pd0, m0, m1, m2, idxa, expb;
    reg [31:0] chk_a, chk_b;

    initial begin
        rstn = 1'b0;
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (5) @(posedge clk);

        //==== 面板基础配置 (同 tb_pov_int): rows=54 + cfg_we|fast|overlap +
        //     oe_window=48 + 消隐; auto + use_fb
        axi_write(16'h000C, 32'hB836_3001);
        axi_write(16'h000C, 32'hC100_0003);

        //==== ⓪a 0x20 oe_window_B 旋钮 (白盒查引擎端口)
        k = errors;
        repeat (3) @(posedge clk);
        if (dut.u_eng_b.oe_window !== 8'd48) fail("[0] oe_window_B default not following A(48)");
        axi_write(16'h0020, 32'd32);
        repeat (3) @(posedge clk);
        if (dut.u_eng_b.oe_window !== 8'd32) fail("[0] 0x20=32 not applied to engine B");
        if (dut.u_eng_a.oe_window !== 8'd48) fail("[0] engine A oe_window disturbed by 0x20");
        axi_write(16'h0020, 32'd0);
        repeat (3) @(posedge clk);
        if (dut.u_eng_b.oe_window !== 8'd48) fail("[0] 0x20=0 not falling back to follow A");
        if (errors == k) $display("PASS [0a] 0x20 oe_window_B: default follow(48) -> 32 -> follow(48), A untouched");

        //==== ⓪b fb_sel_b AXI fb 窗直灌 (pov_en=0)
        k = errors;
        axi_write(16'h9054, 32'hDEAD_0001);            // lane2, waddr 0x15 -> fb_A
        repeat (3) @(posedge clk);
        if (dut.u_eng_a.g_fb[2].mem[9'h15] !== 32'hDEAD_0001)
            fail("[0] AXI fb window write to fb_A failed");
        allow_b_wr = 1'b1;
        axi_write(16'h0010, 32'h8);                    // fb_sel_b=1 (pov/fake/dual=0)
        axi_write(16'h9054, 32'hDEAD_0002);            // -> fb_B
        repeat (3) @(posedge clk);
        if (dut.u_eng_b.g_fb[2].mem[9'h15] !== 32'hDEAD_0002)
            fail("[0] AXI fb window write to fb_B (fb_sel_b) failed");
        if (dut.u_eng_a.g_fb[2].mem[9'h15] !== 32'hDEAD_0001)
            fail("[0] fb_A clobbered by fb_sel_b write");
        axi_write(16'h0010, 32'h0);                    // fb_sel_b off
        allow_b_wr = 1'b0;
        repeat (5) @(posedge clk);
        if (errors == k) $display("PASS [0b] AXI fb window: fb_sel_b routes to fb_B, fb_A intact");

        //==================================================================
        //==== ① dual_en=0 回归 v5 (random 延迟 slave, n_slices=4)
        //==================================================================
        fast_mode = 1'b0;
        tb_dual   = 1'b0;
        tb_nslices = 4;
        tb_base_model = BASE1;
        axi_write(16'h0014, 32'd8000);                 // fake_period
        axi_write(16'h0018, BASE1);
        last_base_wr_cyc = cyci;
        axi_write(16'h0010, (4 << 16) | 32'h3);        // pov + fake, dual=0
        // status: [9]=1 [11]=0 [12]=0
        axi_read(16'h0000, rd);
        k = errors;
        if (rd[9]  !== 1'b1) fail("[1] status [9]=pov_en != 1");
        if (rd[11] !== 1'b0) fail("[1] status [11]=dual_en != 0");
        if (rd[12] !== 1'b0) fail("[1] status [12]=engine_B_busy != 0");
        if (rd[0]  !== 1'b1) fail("[1] status [0]=engine_A_busy != 1 (auto on)");
        // B 消隐 (监视器同时守 fbB_we==0)
        check_b_blank(400);
        // 等 5 个 fetch (全 A, 监视器对账)
        f0 = 0;
        while (fetch_count < 5 && cyci < 200000) @(posedge clk);
        if (fetch_count < 5) fail("[1] <5 fetches in v5-regression phase");
        // idx 步进可见
        axi_read(16'h0010, rd);
        if (rd[31] !== 1'b1) fail("[1] 0x10[31]=locked != 1 in fake");
        // pair_miss == 0 (fake_period 8000 > fetch ~4600)
        axi_read(16'h001C, rd);
        if (rd[15:0] !== 16'd0) begin
            fail("[1] pair_miss != 0 in regression phase");
            $display("        got %0d", rd[15:0]);
        end
        if (errors == k)
            $display("PASS [1] dual_en=0 regression: %0d A-only fetches, B blanked, status[11:12]=0, pair_miss=0",
                     fetch_count);

        // 停 pov, 排空
        axi_write(16'h0010, (4 << 16) | 32'h0);
        while (pair_open || dut.df_busy_w) @(posedge clk);
        pair_open = 1'b0;
        repeat (10) @(posedge clk);

        //==================================================================
        //==== ② + ④ 双屏 fake, n_slices=352, 默认 PHASE_B=180, fast slave
        //==================================================================
        fast_mode  = 1'b1;
        tb_dual    = 1'b1;
        tb_nslices = 352;
        tb_phase   = 180;      // 复位默认, 不写 0x1C, 验证 default
        allow_b_wr = 1'b1;
        budget_en  = 1'b1;
        axi_write(16'h0014, 32'd8000);
        axi_write(16'h0010, (352 << 16) | 32'h7);      // pov + fake + dual
        // status [11]/[12]
        axi_read(16'h0000, rd);
        k = errors;
        if (rd[11] !== 1'b1) fail("[2] status [11]=dual_en != 1");
        if (rd[12] !== 1'b1) fail("[2] status [12]=engine_B_busy != 1");
        pd0 = pairs_done;
        while (pairs_done < pd0 + 5 && cyci < 500000) @(posedge clk);
        if (pairs_done < pd0 + 5) fail("[2] <5 pairs completed (dual fake)");
        if (errors == k)
            $display("PASS [2a] dual n=352 PHASE_B=180(default): %0d pairs, idx_B=(idx_A+180)%%352 all verified",
                     pairs_done - pd0);

        // 冻结 (fake off, idx 保持) → 0x1C 回读核相位 + BRAM 落数抽查
        axi_write(16'h0010, (352 << 16) | 32'h5);      // pov+dual, fake off
        while (pair_open || dut.df_busy_w) @(posedge clk);
        repeat (20) @(posedge clk);
        k = errors;
        axi_read(16'h0010, rd);
        idxa = rd[15:0];
        axi_read(16'h001C, rd2);
        expb = (idxa + 180) % 352;
        if (rd2[24:16] !== expb[8:0]) begin
            fail("[2] 0x1C idx_B readback != (idx_A+180)%%352");
            $display("        idx_A=%0d got %0d expect %0d", idxa, rd2[24:16], expb);
        end
        if (rd2[31]   !== 1'b1)  fail("[2] 0x1C[31]=locked != 1");
        if (rd2[15:0] !== 16'd0) fail("[5] pair_miss != 0 at normal rate (dual)");
        if (errors == k)
            $display("PASS [2b] 0x1C readback: idx_A=%0d idx_B=%0d locked=1 pair_miss=0", idxa, rd2[24:16]);

        // BRAM 抽查: 最后一对 A/B 帧数据都落对 (同 base 不同 idx)
        k = errors;
        chk_a = last_pair_base + last_pairA_idx * 32'h3000;
        chk_b = last_pair_base + last_pairB_idx * 32'h3000;
        if (dut.u_eng_a.g_fb[0].mem[9'd0]   !== slv_data(chk_a))            fail("[2] fb_A lane0 row0 w0 mismatch");
        if (dut.u_eng_a.g_fb[8].mem[9'd429] !== slv_data(chk_a + 2915*4))   fail("[2] fb_A lane8 row53 w5 mismatch");
        if (dut.u_eng_b.g_fb[0].mem[9'd0]   !== slv_data(chk_b))            fail("[2] fb_B lane0 row0 w0 mismatch");
        if (dut.u_eng_b.g_fb[4].mem[9'd219] !== slv_data(chk_b + 1461*4))   fail("[2] fb_B lane4 row27 w3 mismatch");
        if (dut.u_eng_b.g_fb[8].mem[9'd429] !== slv_data(chk_b + 2915*4))   fail("[2] fb_B lane8 row53 w5 mismatch");
        if (errors == k)
            $display("PASS [2c] BRAM spot: A(idx=%0d)/B(idx=%0d) frames landed, 5 cells",
                     last_pairA_idx, last_pairB_idx);

        //==== ② wrap: PHASE_B=351 → idx_A>=1 每对都过 mod
        axi_write(16'h001C, 32'd351);
        tb_phase = 351;
        axi_write(16'h0010, (352 << 16) | 32'h7);      // fake resume
        pd0 = pairs_done;
        k = errors;
        while (pairs_done < pd0 + 4 && cyci < 800000) @(posedge clk);
        if (pairs_done < pd0 + 4) fail("[2] <4 pairs with PHASE_B=351");
        if (errors == k)
            $display("PASS [2d] PHASE_B=351 wrap: %0d pairs, (idx_A+351)%%352 incl. wrap verified",
                     pairs_done - pd0);

        //==================================================================
        //==== ③ pair 翻页原子性: 0x18 写扫过 pair 各阶段 (含 A→B 竞态窗)
        //==================================================================
        k = errors;
        base_race_write(BASE2, 2900);    // A 刚起
        base_race_write(BASE1, 1500);    // A 中段
        base_race_write(BASE2, 300);     // A 尾
        base_race_write(BASE1, 40);      // A 收尾, bvalid 大概率落 A→B 切换附近
        base_race_write(BASE2, 5);       // 极限竞态
        // 落定后下一 pair 用新 base (监视器 settled 检查), 再跑 2 对确认
        pd0 = pairs_done;
        while (pairs_done < pd0 + 2 && cyci < 1200000) @(posedge clk);
        if (pairs_done < pd0 + 2) fail("[3] pairs stalled after race sweep");
        if (last_pair_base !== BASE2) begin
            fail("[3] settled pair not using latest written base");
            $display("        got %08x expect %08x", last_pair_base, BASE2);
        end
        if (errors == k)
            $display("PASS [3] atomicity: 5-point 0x18 race sweep, B always same base as A; next pair takes new base");

        //==================================================================
        //==== ⑤ pair_miss: 过载计数 + 恢复冻结
        //==================================================================
        axi_read(16'h001C, rd);
        m0 = rd[15:0];
        if (m0 !== 0) fail("[5] pair_miss nonzero before overload");
        axi_write(16'h0014, 32'd1500);                 // < pair ~6500 → 过载
        repeat (40000) @(posedge clk);
        axi_write(16'h0014, 32'd9000);                 // 恢复
        repeat (20000) @(posedge clk);
        axi_read(16'h001C, rd);
        m1 = rd[15:0];
        k = errors;
        if (m1 - m0 < 5) begin
            fail("[5] pair_miss did not accumulate under overload");
            $display("        m0=%0d m1=%0d", m0, m1);
        end
        repeat (30000) @(posedge clk);
        axi_read(16'h001C, rd);
        m2 = rd[15:0];
        if (m2 !== m1) begin
            fail("[5] pair_miss still counting after recovery");
            $display("        m1=%0d m2=%0d", m1, m2);
        end
        if (errors == k)
            $display("PASS [5] pair_miss: 0 -> %0d under overload (fake 1500 < pair), frozen at %0d after recovery",
                     m1, m2);

        //==================================================================
        //==== ① 补: dual_en 关回 0 (在线降级) → 回归单屏
        //==================================================================
        // 冻结, 排空, 再关 dual
        axi_write(16'h0010, (352 << 16) | 32'h5);      // fake off
        while (pair_open || dut.df_busy_w) @(posedge clk);
        repeat (10) @(posedge clk);
        tb_dual    = 1'b0;
        allow_b_wr = 1'b0;
        budget_en  = 1'b0;
        axi_write(16'h0010, (352 << 16) | 32'h3);      // pov + fake, dual=0
        f0 = fetch_count;
        k = errors;
        while (fetch_count < f0 + 3 && cyci < 2000000) @(posedge clk);
        if (fetch_count < f0 + 3) fail("[1] no A fetches after dual off");
        axi_read(16'h0000, rd);
        if (rd[11] !== 1'b0) fail("[1] status [11] != 0 after dual off");
        if (rd[12] !== 1'b0) fail("[1] status [12] != 0 after dual off");
        check_b_blank(300);
        if (errors == k)
            $display("PASS [1b] dual off mid-flight: A-only fetches resume, B re-blanked");

        axi_write(16'h0010, (352 << 16) | 32'h0);      // pov off
        while (pair_open || dut.df_busy_w) @(posedge clk);
        pair_open = 1'b0;
        repeat (10) @(posedge clk);

        //==================================================================
        //==== ⑥ R 0x28 = frame_period (引擎 A 整屏 aclk 拍数)
        //     这是定 aclk 真实频率的钥匙: 本值是纯 PL 侧计数, 不含任何频率假设;
        //     用已验证正确的 CPU 时基测同一整屏的墙钟秒数, 两者相除 = aclk 真频率。
        //==================================================================
        k = errors;
        // 本 TB 从 [0] 起就在跑 auto (0x0C 写的是 B836_3001 = ddr_slow(dclk_fast)=1,
        // rows=54, oe=48) ⇒ 此刻 R0x28 应当已经反映**降级态**的整屏拍数
        // 54 x 387 = 20898 (slow 每 bit 2 拍 ⇒ 行周期 195+192=387)。
        axi_read(16'h0028, rd);
        if (rd !== 32'd20898) begin
            fail("[6] R0x28 frame_period != 20898 (ddr_slow, 54 x 387)");
            $display("        got %0d", rd);
        end else
            $display("PASS [6a] R0x28 = %0d aclk (ddr_slow=1: 54 rows x 387)", rd);
        // 0x0C sub10: rows=54, oe_window=48, cfg_we=1, dclk_fast=0(fast), oe_set_val=1(消隐)
        axi_write(16'h000C, 32'h8836_3001);
        // 0x0C sub11: auto_en=1
        axi_write(16'h000C, 32'hC000_0001);
        repeat (2*54*195 + 2000) @(posedge clk);
        axi_read(16'h0028, rd);
        if (rd !== 32'd10530) begin
            fail("[6] R0x28 frame_period != 10530 (1-bit fast, 54 x 195)");
            $display("        got %0d", rd);
        end else
            $display("PASS [6b] R0x28 = %0d aclk after ddr_slow->fast (1-bit: 54 x 195)", rd);

        // 0x0C sub01: oe_w1=54, oe_w2=108, bpp_mode=1 → 3-bit 行内 BCM
        axi_write(16'h000C, 32'h4001_6C36);
        repeat (2*54*3*195 + 2000) @(posedge clk);
        axi_read(16'h0028, rd);
        if (rd !== 32'd31590) begin
            fail("[6] R0x28 frame_period != 31590 (3-bit, 54 x 3 x 195)");
            $display("        got %0d", rd);
        end else
            $display("PASS [6c] R0x28 = %0d aclk (3-bit BCM: 54 rows x 3 planes x 195)", rd);

        // 运行时切回 1-bit (沿数写 0 = 保持原值)
        axi_write(16'h000C, 32'h4000_0000);
        repeat (2*54*195 + 2000) @(posedge clk);
        axi_read(16'h0028, rd);
        if (rd !== 32'd10530) begin
            fail("[6] R0x28 not back to 10530 after bpp_mode=0");
            $display("        got %0d", rd);
        end else if (errors == k)
            $display("PASS [6d] runtime 3-bit <-> 1-bit via 0x0C sub01, R0x28 tracks (31590 -> 10530)");
        axi_write(16'h000C, 32'hC000_0000);          // auto_en=0
        repeat (500) @(posedge clk);

        //==================================================================
        //==== 总结 (④ 带宽账数字)
        //==================================================================
        if (max_arlen_all !== 255) begin
            fail("[4] never observed a 256-beat burst (max arlen != 255)");
            $display("        max arlen = %0d", max_arlen_all);
        end
        $display("---- [4] fetch accounting: %0d fetches / %0d pairs, max arlen=%0d",
                 fetch_count, pairs_done, max_arlen_all);
        $display("---- [4] pair(A+B) duration: min %0d / max %0d cycles vs budget %0d (213.7us)",
                 pair_dur_min, pair_dur_max, BUDGET_CYC);
        $display("---- [4] single fetch max %0d cycles (%0d.%02d us); pair margin = %0d.%02dx",
                 fetchA_dur_max, fetchA_dur_max/50, (fetchA_dur_max%50)*2,
                 (BUDGET_CYC*100/pair_dur_max)/100, (BUDGET_CYC*100/pair_dur_max)%100);

        if (errors == 0)
            $display("=== TB RESULT: ALL CHECKS PASS (dual POV top: v5 regression / PHASE_B mod 352 / atomic pair / 256-beat budget / pair_miss) ===");
        else
            $display("=== TB RESULT: %0d FAILURES ===", errors);
        $finish;
    end

    // 看门狗
    initial begin
        #60_000_000;   // 60 ms
        fail("watchdog timeout");
        $display("        cyci=%0d fetches=%0d pairs=%0d pair_open=%b df_busy=%b idx=%0d",
                 cyci, fetch_count, pairs_done, pair_open, dut.df_busy_w, dut.at_slice_idx);
        $display("=== TB RESULT: TIMEOUT ===");
        $finish;
    end

endmodule
