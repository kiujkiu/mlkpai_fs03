//==============================================================================
// tb_ddr_fetch.v — ddr_slice_fetch 自检 TB (xsim)
//
// 行为级 AXI4 read slave: 256KB 内存数组预填 mem[i] = addr ^ 0xA5A5,
// arready / rvalid 随机延迟, 单 outstanding.
//
// 检查:
//   1. fb 写序列 (lane,row,word,data) 与 lane-major DDR 布局一一对应,
//      全 2916 写 (11664B/4)
//   2. 换 slice_idx 再 fetch, 地址偏移 = idx*0x3000 正确 (且 go 后改 slice_idx
//      不影响本轮 = 锁存验证)
//   3. AR 合法性: arlen<=15 / INCR / size=4B / 不跨 4KB / 地址连续;
//      每 burst 实收 beats == arlen+1 (rlast 位置), 总 beats == 2916
//   4. busy/done 时序: go 后 busy 起, done 恰 1 拍, done 时 busy 已落;
//      busy 期间的 fetch_go 被忽略 (不重启, 数据不偏)
//
// slice_base 故意取非 4KB 对齐 (+0x34) 逼出 burst 4KB 截断路径.
//==============================================================================
`timescale 1ns / 1ps

module tb_ddr_fetch;

    localparam [31:0] MEM_BASE      = 32'h0010_0000;
    localparam integer MEM_WORDS    = 65536;             // 256 KB
    localparam [31:0] SLICE_BASE_TB = MEM_BASE + 32'h34; // 非 4KB 对齐
    localparam integer FRAME_WORDS  = 2916;

    reg aclk = 1'b0;
    reg aresetn = 1'b0;
    always #10 aclk = ~aclk;    // 50 MHz

    //-------------------------------------------------------------------
    // DUT 连线
    //-------------------------------------------------------------------
    wire [31:0] araddr;
    wire [7:0]  arlen;
    wire [2:0]  arsize;
    wire [1:0]  arburst;
    wire        arvalid;
    reg         arready;
    reg  [31:0] rdata;
    wire [1:0]  rresp = 2'b00;
    reg         rlast;
    reg         rvalid;
    wire        rready;

    reg  [31:0] slice_base = SLICE_BASE_TB;
    reg  [8:0]  slice_idx  = 9'd0;
    reg         fetch_go   = 1'b0;
    wire        fetch_busy;
    wire        fetch_done;

    wire        fb_we;
    wire [3:0]  fb_wlane;
    wire [8:0]  fb_waddr;
    wire [31:0] fb_wdata;

    ddr_slice_fetch dut (
        .aclk          (aclk),
        .aresetn       (aresetn),
        .m_axi_araddr  (araddr),
        .m_axi_arlen   (arlen),
        .m_axi_arsize  (arsize),
        .m_axi_arburst (arburst),
        .m_axi_arlock  (),
        .m_axi_arcache (),
        .m_axi_arprot  (),
        .m_axi_arvalid (arvalid),
        .m_axi_arready (arready),
        .m_axi_rdata   (rdata),
        .m_axi_rresp   (rresp),
        .m_axi_rlast   (rlast),
        .m_axi_rvalid  (rvalid),
        .m_axi_rready  (rready),
        .slice_base    (slice_base),
        .slice_idx     (slice_idx),
        .fetch_go      (fetch_go),
        .fetch_busy    (fetch_busy),
        .fetch_done    (fetch_done),
        .fb_we         (fb_we),
        .fb_wlane      (fb_wlane),
        .fb_waddr      (fb_waddr),
        .fb_wdata      (fb_wdata)
    );

    integer errors = 0;

    task fail(input [255:0] msg);
        begin
            errors = errors + 1;
            $display("[%0t] FAIL: %0s", $time, msg);
        end
    endtask

    //-------------------------------------------------------------------
    // 行为级 AXI read slave (单 outstanding, 随机延迟)
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

    // 内存读 + 越界检查
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

    // arready: 无活动 burst 时随机拉起, 握手后至少歇 1 拍
    always @(posedge aclk) begin
        if (!aresetn)
            arready <= 1'b0;
        else if (arvalid && arready)
            arready <= 1'b0;
        else
            arready <= !burst_active && (({$random} % 3) == 0);
    end

    // R 通道: 随机 rvalid 间隙
    always @(posedge aclk) begin
        if (!aresetn) begin
            burst_active <= 1'b0;
            rvalid <= 1'b0;
            rlast  <= 1'b0;
            rdata  <= 32'd0;
        end else begin
            if (arvalid && arready) begin
                burst_active <= 1'b1;
                baddr <= araddr;
                blen  <= arlen;
                bcnt  <= 8'd0;
            end

            if (rvalid && rready) begin
                if (rlast) begin
                    rvalid <= 1'b0;
                    rlast  <= 1'b0;
                    burst_active <= 1'b0;
                end else begin
                    bcnt  <= bcnt + 8'd1;
                    baddr <= baddr + 32'd4;
                    if (({$random} % 3) != 0) begin
                        rdata <= slv_read(baddr + 32'd4);
                        rlast <= ((bcnt + 8'd1) == blen);
                    end else begin
                        rvalid <= 1'b0;     // 随机间隙, 下面分支再拉起
                    end
                end
            end else if (burst_active && !rvalid) begin
                if (({$random} % 3) != 0) begin
                    rvalid <= 1'b1;
                    rdata  <= slv_read(baddr);
                    rlast  <= (bcnt == blen);
                end
            end
        end
    end

    //-------------------------------------------------------------------
    // 监视器 ③: AR 合法性 + 地址连续 + beats/rlast
    //-------------------------------------------------------------------
    reg [31:0] exp_ar_addr;
    reg [7:0]  cur_arlen;
    integer    burst_beats;
    integer    total_beats;

    always @(posedge aclk) begin
        if (aresetn && arvalid && arready) begin
            if (arlen > 8'd15)              fail("arlen > 15");
            if (arsize !== 3'b010)          fail("arsize != 4B");
            if (arburst !== 2'b01)          fail("arburst != INCR");
            if (araddr[1:0] !== 2'b00)      fail("araddr not word aligned");
            if (({20'b0, araddr[11:0]} + (arlen + 1)*4) > 32'd4096)
                                            fail("burst crosses 4KB boundary");
            if (araddr !== exp_ar_addr) begin
                fail("AR address not contiguous");
                $display("        got %08x expect %08x", araddr, exp_ar_addr);
            end
            exp_ar_addr = exp_ar_addr + (arlen + 1)*4;
            cur_arlen   = arlen;
            burst_beats = 0;
        end
        if (aresetn && rvalid && rready) begin
            burst_beats = burst_beats + 1;
            total_beats = total_beats + 1;
            if (rlast && burst_beats !== cur_arlen + 1)
                fail("rlast beat count != arlen+1");
            if (!rlast && burst_beats > cur_arlen)
                fail("beats exceed arlen+1 without rlast");
        end
    end

    //-------------------------------------------------------------------
    // 记分板 ①②: fb 写序列 vs lane-major 布局
    //-------------------------------------------------------------------
    reg [31:0] fetch_base;      // 本轮预期起始地址
    integer    wr_cnt;
    reg [3:0]  exp_lane;
    reg [5:0]  exp_row;
    reg [2:0]  exp_word;
    reg [31:0] exp_data;
    integer    rem;

    always @(posedge aclk) begin
        if (aresetn && fb_we) begin
            exp_lane = wr_cnt / 324;            // 324 word / lane
            rem      = wr_cnt % 324;
            exp_row  = rem / 6;
            exp_word = rem % 6;
            exp_data = mem[((fetch_base + wr_cnt*4) - MEM_BASE) >> 2];
            if (fb_wlane !== exp_lane) begin
                fail("fb_wlane mismatch");
                $display("        wr %0d: got %0d expect %0d", wr_cnt, fb_wlane, exp_lane);
            end
            if (fb_waddr !== {exp_row, exp_word}) begin
                fail("fb_waddr mismatch");
                $display("        wr %0d: got %03x expect %03x", wr_cnt, fb_waddr, {exp_row, exp_word});
            end
            if (fb_wdata !== exp_data) begin
                fail("fb_wdata mismatch");
                $display("        wr %0d: got %08x expect %08x", wr_cnt, fb_wdata, exp_data);
            end
            wr_cnt = wr_cnt + 1;
        end
    end

    //-------------------------------------------------------------------
    // done 脉宽监视 ④
    //-------------------------------------------------------------------
    integer done_hi_cycles;
    always @(posedge aclk) begin
        if (aresetn && fetch_done) begin
            done_hi_cycles = done_hi_cycles + 1;
            if (fetch_busy) fail("busy still high during done pulse");
        end
    end

    //-------------------------------------------------------------------
    // 单轮 fetch 任务
    //-------------------------------------------------------------------
    task run_fetch(input [8:0] idx, input spurious_go);
        begin
            fetch_base     = SLICE_BASE_TB + idx * 32'h3000;
            exp_ar_addr    = fetch_base;
            wr_cnt         = 0;
            total_beats    = 0;
            done_hi_cycles = 0;

            @(negedge aclk);
            slice_idx = idx;
            fetch_go  = 1'b1;
            @(negedge aclk);
            fetch_go  = 1'b0;
            slice_idx = 9'h1FF;     // go 后立刻打乱, 验证锁存

            @(negedge aclk);
            if (!fetch_busy) fail("busy not asserted after go");

            if (spurious_go) begin
                repeat (300) @(negedge aclk);
                if (!fetch_busy) fail("fetch finished too early for spurious test");
                slice_idx = 9'd7;   // busy 期间伪 go, 必须被忽略
                fetch_go  = 1'b1;
                @(negedge aclk);
                fetch_go  = 1'b0;
                slice_idx = 9'h1FF;
            end

            wait (fetch_done);
            @(negedge aclk);
            @(negedge aclk);

            if (done_hi_cycles !== 1) fail("fetch_done not exactly 1 cycle");
            if (fetch_busy)           fail("busy still high after done");
            if (wr_cnt !== FRAME_WORDS) begin
                fail("fb write count != 2916");
                $display("        got %0d", wr_cnt);
            end
            if (total_beats !== FRAME_WORDS) begin
                fail("total AXI beats != 2916");
                $display("        got %0d", total_beats);
            end

            // 静默期: 伪 go 不得引发重启
            repeat (100) @(negedge aclk);
            if (fetch_busy)             fail("unexpected restart after done");
            if (wr_cnt !== FRAME_WORDS) fail("extra fb writes after done");

            $display("[%0t] fetch slice_idx=%0d done: %0d writes, %0d beats, base=%08x",
                     $time, idx, wr_cnt, total_beats, fetch_base);
        end
    endtask

    //-------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------
    initial begin
        exp_ar_addr    = 32'd0;
        fetch_base     = 32'd0;
        wr_cnt         = 0;
        total_beats    = 0;
        done_hi_cycles = 0;
        burst_beats    = 0;
        cur_arlen      = 8'd0;

        repeat (5) @(negedge aclk);
        aresetn = 1'b1;
        repeat (5) @(negedge aclk);

        if (fetch_busy) fail("busy high at idle");

        run_fetch(9'd0, 1'b0);      // ① 基本布局
        run_fetch(9'd3, 1'b1);      // ② 偏移 0x9000 + ④ 伪 go 忽略
        run_fetch(9'd1, 1'b0);      // 复用: 连续第三轮

        if (errors == 0)
            $display("=== ALL TESTS PASS (3 fetches x 2916 writes, AR legal, busy/done ok) ===");
        else
            $display("=== %0d ERRORS ===", errors);
        $finish;
    end

    // 看门狗
    initial begin
        #20_000_000;    // 20 ms
        fail("watchdog timeout");
        $display("=== %0d ERRORS ===", errors);
        $finish;
    end

endmodule
