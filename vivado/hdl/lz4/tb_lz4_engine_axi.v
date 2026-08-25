//============================================================================
// tb_lz4_engine_axi.v — lz4_engine_axi (适配壳) 的回归
//
//   iverilog -g2012 -o eng.out tb_lz4_engine_axi.v \
//            lz4_engine_axi.v lz4_axi_top.v lz4_decode_core.v
//   vvp eng.out +VEC=real_slice000 [+STALL] [+SKEW=n]
//   -DWPACK_SEL=0 编译 = 测"不打包"那一档 (逃生门, 必须也是绿的)
//
// 与 dr1v90/lz4hw/sim/tb_lz4_axi.v 的关系: 存储模型/比对逻辑照抄 (那部分已验过),
// **唯一真正新增的是 AXI-Lite 主端**:
//
// 🔴 原 tb 的 lite_w 是 `s_awvalid <= 1; s_wvalid <= 1;` 同一拍拉高一拍就撒手
//    (tb_lz4_axi.v:128), 而且不看 awready/wready/bvalid。真 AXI-Lite 主端
//    (GP0 → axi_interconnect → 协议转换器) 不保证 AW/W 同拍, 也一定会等 B。
//    ⇒ 本 tb 的 lite_w 带 **skew 参数**: skew>0 = AW 先到 skew 拍, skew<0 = W 先到,
//      并且严格按 VALID/READY 握手 + 等 BVALID。默认把 -3..+3 全跑一遍。
//    这正是 lz4_engine_axi 存在的理由 —— 直接拿 lz4_axi_top 接 GP0,
//    AW 先到那一拍就会被"恒 awready"吃掉且永不回 B ⇒ **CPU 写挂死**。
//============================================================================
`timescale 1ns / 1ps

module tb_lz4_engine_axi;
    localparam AXI_DW = 64, BPW = 8;
    localparam MEMB   = 1 << 22;
    localparam SRC    = 32'h0000_0000;
    localparam DST    = 32'h0020_0000;

    reg [7:0] mem  [0:MEMB-1];
    reg [7:0] expb [0:(1<<21)-1];
    integer comp_len, raw_len;

    reg clk = 0, rstn = 0;
    always #5 clk = ~clk;

    // ---- AXI4-Lite 主端 (合规) ----
    reg  [15:0] s_awaddr = 0; reg [2:0] s_awprot = 0; reg s_awvalid = 0; wire s_awready;
    reg  [31:0] s_wdata  = 0; reg [3:0] s_wstrb = 4'hF; reg s_wvalid = 0; wire s_wready;
    wire [1:0]  s_bresp;      wire s_bvalid;  reg s_bready = 1;
    reg  [15:0] s_araddr = 0; reg [2:0] s_arprot = 0; reg s_arvalid = 0; wire s_arready;
    wire [31:0] s_rdata;      wire [1:0] s_rresp; wire s_rvalid; reg s_rready = 1;

    // ---- AXI4 master ----
    wire [31:0] m_awaddr; wire [7:0] m_awlen; wire [2:0] m_awsize;
    wire [1:0]  m_awburst; wire m_awlock; wire [3:0] m_awcache; wire [2:0] m_awprot;
    wire m_awvalid; reg m_awready = 1;
    wire [63:0] m_wdata; wire [7:0] m_wstrb; wire m_wlast; wire m_wvalid; reg m_wready = 1;
    reg  [1:0]  m_bresp = 0; reg m_bvalid = 0; wire m_bready;
    wire [31:0] m_araddr; wire [7:0] m_arlen; wire [2:0] m_arsize;
    wire [1:0]  m_arburst; wire m_arlock; wire [3:0] m_arcache; wire [2:0] m_arprot;
    wire m_arvalid; reg m_arready = 1;
    reg  [63:0] m_rdata; reg [1:0] m_rresp = 0; reg m_rlast = 1; reg m_rvalid = 0; wire m_rready;

`ifndef WPACK_SEL
  `define WPACK_SEL 1      // iverilog -DWPACK_SEL=0 可测直通(不打包)那一档
`endif
    lz4_engine_axi #(.WPACK(`WPACK_SEL)) dut (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(s_awaddr), .s_axi_awprot(s_awprot),
        .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
        .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb),
        .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
        .s_axi_bresp(s_bresp), .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready),
        .s_axi_araddr(s_araddr), .s_axi_arprot(s_arprot),
        .s_axi_arvalid(s_arvalid), .s_axi_arready(s_arready),
        .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp),
        .s_axi_rvalid(s_rvalid), .s_axi_rready(s_rready),
        .m_axi_awaddr(m_awaddr), .m_axi_awlen(m_awlen), .m_axi_awsize(m_awsize),
        .m_axi_awburst(m_awburst), .m_axi_awlock(m_awlock), .m_axi_awcache(m_awcache),
        .m_axi_awprot(m_awprot), .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wlast(m_wlast),
        .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bresp(m_bresp), .m_axi_bvalid(m_bvalid), .m_axi_bready(m_bready),
        .m_axi_araddr(m_araddr), .m_axi_arlen(m_arlen), .m_axi_arsize(m_arsize),
        .m_axi_arburst(m_arburst), .m_axi_arlock(m_arlock), .m_axi_arcache(m_arcache),
        .m_axi_arprot(m_arprot), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
        .m_axi_rdata(m_rdata), .m_axi_rresp(m_rresp), .m_axi_rlast(m_rlast),
        .m_axi_rvalid(m_rvalid), .m_axi_rready(m_rready)
    );

    // ---- 补齐信号的常数检查: AWSIZE 补错 = 静默数据错位, 必须盯 ----
    always @(posedge clk) if (rstn) begin
        if (m_awvalid && (m_awsize !== 3'b011 || m_awburst !== 2'b01)) begin
            $display("RESULT: FAIL AWSIZE/AWBURST = %b/%b (期望 011/01)", m_awsize, m_awburst);
            $fatal(1);
        end
        if (m_arvalid && (m_arsize !== 3'b011 || m_arburst !== 2'b01)) begin
            $display("RESULT: FAIL ARSIZE/ARBURST = %b/%b (期望 011/01)", m_arsize, m_arburst);
            $fatal(1);
        end
    end

    reg stall_en = 0;
    always @(posedge clk) if (stall_en) begin
        m_arready <= ({$random} % 4) != 0;
        m_awready <= ({$random} % 3) == 0;
        m_wready  <= ({$random} % 3) == 0;
    end

    // ---- 存储模型: 读 ----
    integer k;
    reg [31:0] rlat_addr; reg rlat_pend = 0; integer rdelay = 0;
    always @(posedge clk) begin
        m_rvalid <= 1'b0;
        if (m_arvalid && m_arready) begin
            rlat_addr <= m_araddr; rlat_pend <= 1'b1;
            rdelay <= stall_en ? (1 + ({$random} % 7)) : 1;
        end else if (rlat_pend) begin
            if (rdelay > 1) rdelay <= rdelay - 1;
            else begin
                for (k = 0; k < BPW; k = k + 1) m_rdata[k*8 +: 8] <= mem[rlat_addr + k];
                m_rvalid <= 1'b1; rlat_pend <= 1'b0;
            end
        end
    end

    // ---- 存储模型: 写 (多拍 INCR 突发) ----
    // 🔴 AW 与 W 是**完全独立的两个通道**, AXI 允许 W 先于 AW 到 —— 反压档下
    //    m_awready 随机拉低, 打包器同拍拉的 AWVALID/WVALID 就会被拆开, W 先走。
    //    第一版模型假定"W 到的时候 AW 已经在队列里", 于是 76/160 条误报 FAIL。
    //    真互连(SmartConnect/HP)是收得下的, 所以**该改的是模型**: 两个通道各自
    //    入队, 消费端再配对。这条也是 feedback_always_ready_tb_hides_handshake_bugs
    //    的同类 —— 恒 ready 的模型会把通道乱序这一整类问题藏起来。
    //
    // 检查项 (每条都是打包器最可能写错的地方):
    //   * AWLEN 在 AWVALID 期间不变   * WLAST 恰好落在第 AWLEN+1 拍
    //   * 突发不跨 4KB                * 每笔突发恰好回一个 B
    reg [31:0] awq_a [0:15];  reg [7:0]  awq_l [0:15];
    reg [63:0] wq_d  [0:63];  reg [7:0]  wq_s  [0:63];  reg wq_l [0:63];
    reg [3:0]  awq_w = 0, awq_r = 0;  integer awq_n = 0;
    reg [5:0]  wq_w  = 0, wq_r  = 0;  integer wq_n  = 0;
    reg [31:0] w_addr;  integer w_left = 0;  reg w_active = 0;
    integer wr_beats = 0, wr_bursts = 0;
    reg [7:0] bsr = 0;  reg bnow;
    reg [31:0] base_a;  reg [8:0] base_l;
    reg [63:0] cd; reg [7:0] cs; reg cl;
    reg awlen_lock = 0;  reg [7:0] awlen_q;

    wire aw_fire = m_awvalid && m_awready;
    wire w_fire  = m_wvalid  && m_wready;

    always @(posedge clk) if (rstn) begin
        if (m_awvalid && !awlen_lock) begin awlen_lock <= 1'b1; awlen_q <= m_awlen; end
        else if (m_awvalid && awlen_lock && m_awlen !== awlen_q) begin
            $display("RESULT: FAIL AWVALID 期间 AWLEN 变了 %0d -> %0d", awlen_q, m_awlen);
            $fatal(1);
        end
        if (aw_fire) awlen_lock <= 1'b0;
    end

    always @(posedge clk) begin
        bnow = 1'b0;

        // ---- 入队 (blocking: 同拍推入的条目本拍就可以被消费) ----
        if (aw_fire) begin
            if ((m_awaddr & 32'hFFFFF000) !=
                ((m_awaddr + ({24'd0, m_awlen} + 1) * BPW - 1) & 32'hFFFFF000)) begin
                $display("RESULT: FAIL 突发跨 4KB: addr=%08x len=%0d", m_awaddr, m_awlen);
                $fatal(1);
            end
            awq_a[awq_w] = m_awaddr; awq_l[awq_w] = m_awlen;
            awq_w = awq_w + 1; awq_n = awq_n + 1;
        end
        if (w_fire) begin
            wq_d[wq_w] = m_wdata; wq_s[wq_w] = m_wstrb; wq_l[wq_w] = m_wlast;
            wq_w = wq_w + 1; wq_n = wq_n + 1;
            if (wq_n > 64) begin $display("RESULT: FAIL 模型 W 队列溢出"); $fatal(1); end
        end

        // ---- 消费: 先取一笔 AW, 再逐拍吃 W ----
        if (!w_active && awq_n > 0) begin
            base_a = awq_a[awq_r]; base_l = {1'b0, awq_l[awq_r]} + 9'd1;
            awq_r = awq_r + 1; awq_n = awq_n - 1;
            w_addr = base_a; w_left = base_l; w_active = 1'b1;
        end
        if (w_active && wq_n > 0) begin
            cd = wq_d[wq_r]; cs = wq_s[wq_r]; cl = wq_l[wq_r];
            wq_r = wq_r + 1; wq_n = wq_n - 1;
            for (k = 0; k < BPW; k = k + 1)
                if (cs[k]) mem[w_addr + k] = cd[k*8 +: 8];
            if (cl !== (w_left == 1)) begin
                $display("RESULT: FAIL WLAST=%b 但本笔还剩 %0d 拍", cl, w_left);
                $fatal(1);
            end
            w_addr = w_addr + BPW; w_left = w_left - 1; wr_beats = wr_beats + 1;
            if (w_left == 0) begin
                w_active = 1'b0; wr_bursts = wr_bursts + 1; bnow = 1'b1;
            end
        end

        // ---- B: 每笔突发一个, 反压档延迟 4 拍 (验 DUT 有没有等 B) ----
        bsr = {bsr[6:0], bnow};
        m_bvalid <= stall_en ? bsr[4] : bsr[0];
    end

    // ---- 合规 AXI4-Lite 写: skew>0 AW 先到, skew<0 W 先到, 0 同拍 ----
    integer bcount = 0;
    task lite_w(input [15:0] a, input [31:0] d, input integer skew);
        begin
            fork
                begin
                    if (skew > 0) repeat (skew) @(posedge clk);
                    @(posedge clk); s_awaddr <= a; s_awvalid <= 1'b1;
                    @(posedge clk); while (!s_awready) @(posedge clk);
                    s_awvalid <= 1'b0;
                end
                begin
                    if (skew < 0) repeat (-skew) @(posedge clk);
                    @(posedge clk); s_wdata <= d; s_wstrb <= 4'hF; s_wvalid <= 1'b1;
                    @(posedge clk); while (!s_wready) @(posedge clk);
                    s_wvalid <= 1'b0;
                end
            join
            // 🔴 必须等 B —— 少一个 B 响应, 真互连上就是 CPU 卡死
            @(posedge clk); while (!s_bvalid) @(posedge clk);
            bcount = bcount + 1;
            if (s_bresp !== 2'b00) begin
                $display("RESULT: FAIL BRESP=%b", s_bresp); $fatal(1); end
        end
    endtask

    task lite_r(input [15:0] a, output [31:0] d);
        begin
            @(posedge clk); s_araddr <= a; s_arvalid <= 1'b1;
            @(posedge clk); while (!s_arready) @(posedge clk);
            s_arvalid <= 1'b0;
            @(posedge clk); while (!s_rvalid) @(posedge clk);
            d = s_rdata;
            if (s_rresp !== 2'b00) begin
                $display("RESULT: FAIL RRESP=%b", s_rresp); $fatal(1); end
        end
    endtask

    reg [255:0] vec; integer fd, code, i, errs, skew, dstpad; reg [1023:0] path;
    reg [31:0] st, cyc, rb;
    reg [7:0] comp [0:(1<<20)-1];

    initial begin
        if (!$value$plusargs("VEC=%s", vec)) begin $display("需要 +VEC="); $finish; end
        if (!$value$plusargs("SKEW=%d", skew)) skew = 0;
        $sformat(path, "%0s/%0s.meta", `VECDIR, vec);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("打不开 %0s", path); $finish; end
        code = $fscanf(fd, "%d %d", comp_len, raw_len); $fclose(fd);
        $sformat(path, "%0s/%0s.in.hex",  `VECDIR, vec); $readmemh(path, comp);
        $sformat(path, "%0s/%0s.ref.hex", `VECDIR, vec); $readmemh(path, expb);

        for (i = 0; i < comp_len; i = i + 1) mem[SRC + i] = comp[i];
        for (i = 0; i < raw_len + 16; i = i + 1) mem[DST + i] = 8'hA5;
        if ($test$plusargs("STALL")) stall_en = 1;

        #20 rstn = 1;
        // 🔴 五次寄存器写用**不同的 AW/W 相对时序** —— 真互连每笔都可能不一样
        lite_w(16'h0008, SRC,      skew);
        lite_w(16'h000C, comp_len, -skew);
        lite_w(16'h0010, DST,      0);
        lite_w(16'h0014, raw_len,  skew);
        // 写完立刻回读, 验寄存器真的进去了 (顺带验读通道握手)
        lite_r(16'h0008, rb);
        if (rb !== SRC) begin $display("RESULT %0s: FAIL SRC_ADDR 回读 %08x", vec, rb); $fatal(1); end
        lite_r(16'h0014, rb);
        if (rb !== raw_len) begin $display("RESULT %0s: FAIL DST_LEN 回读 %0d", vec, rb); $fatal(1); end

        lite_w(16'h0000, 32'h1, -skew);              // start

        // 软件真实用法: 轮询 STATUS 的 done/error 位, 不看内部信号
        st = 0;
        while (!(st[0] || st[1])) begin
            lite_r(16'h0004, st);
            repeat (32) @(posedge clk);
        end
        if (st[1]) begin
            $display("RESULT %0s: FAIL STATUS 报错 err_code=%0d", vec, st[4:2]); $fatal(1); end
        if (st[5] !== 1'b0) begin
            $display("RESULT %0s: FAIL done=1 但 busy 还是 1 (STATUS=%08x)", vec, st); $fatal(1); end

        errs = 0;
        for (i = 0; i < raw_len; i = i + 1)
            if (mem[DST + i] !== expb[i]) begin
                if (errs < 8) $display("  x DDR[%0d]: 得到 %02x 期望 %02x", i, mem[DST+i], expb[i]);
                errs = errs + 1;
            end
        if (errs) begin $display("RESULT %0s: FAIL %0d/%0d 字节不符", vec, errs, raw_len); $fatal(1); end
        for (i = raw_len; i < raw_len + 16; i = i + 1)
            if (mem[DST + i] !== 8'hA5) begin
                $display("RESULT %0s: FAIL 尾拍越界写 DDR[+%0d]=%02x", vec, i, mem[DST+i]); $fatal(1); end
        if (wr_beats != (raw_len + BPW - 1) / BPW) begin
            $display("RESULT %0s: FAIL 写拍数 %0d, 期望 %0d",
                     vec, wr_beats, (raw_len + BPW - 1) / BPW); $fatal(1); end
        if (awq_n != 0 || wq_n != 0 || w_active) begin
            $display("RESULT %0s: FAIL done 报了但写通道没排空 (AW 余 %0d, W 余 %0d, 突发中=%b)",
                     vec, awq_n, wq_n, w_active); $fatal(1); end

        lite_r(16'h0018, cyc);
        // 64K 窗内每 256B 一个镜像 —— 软件万一按 0x40020100 访问也应该读到同一个寄存器
        lite_r(16'h0118, rb);
        if (rb !== cyc) begin
            $display("RESULT %0s: FAIL 0x118 镜像读回 %0d != 0x18 的 %0d", vec, rb, cyc); $fatal(1); end

        $display("RESULT %0s: PASS  %0d B 正确 (skew=%0d, cycles=%0d, %0.3f B/clk, %0d 拍 / %0d 笔突发 = %0.1f 拍/笔)",
                 vec, raw_len, skew, cyc, raw_len*1.0/cyc,
                 wr_beats, wr_bursts, wr_beats*1.0/wr_bursts);
        $finish;
    end

    initial begin #200000000; $display("RESULT %0s: FAIL 超时 (AXI-Lite 挂死?)", vec); $fatal(1); end
endmodule
