`timescale 1ns / 1ps
//=============================================================================
// tb_fb_data.v — v6 数据通路专项: AXI fb 窗灌全 1 → 断言 sdi_pad 出实心高
// 背景: 板上 LA 实测 v6 九路 SDI 恒低 (v5 金样红场 99% 高), 复现并定位。
//=============================================================================
module tb_fb_data;

    reg clk  = 1'b0;
    reg rstn = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    reg  [15:0] awaddr  = 16'b0;
    reg         awvalid = 1'b0;
    reg  [31:0] wdata_r = 32'b0;
    reg         wvalid  = 1'b0;
    reg         bready  = 1'b0;
    wire        awready, wready, bvalid;
    wire [1:0]  bresp;
    reg  [15:0] araddr  = 16'b0;
    reg         arvalid = 1'b0;
    reg         rready  = 1'b0;
    wire        arready, rvalid;
    wire [31:0] rdata;
    wire [1:0]  rresp;

    wire [31:0] m_araddr;
    wire [7:0]  m_arlen;
    wire [2:0]  m_arsize;
    wire [1:0]  m_arburst;
    wire        m_arlock, m_arvalid;
    wire [3:0]  m_arcache;
    wire [2:0]  m_arprot;
    reg         m_arready = 1'b0;
    reg  [31:0] m_rdata = 32'b0;
    wire [1:0]  m_rresp = 2'b00;
    reg         m_rlast = 1'b0;
    reg         m_rvalid = 1'b0;
    wire        m_rready;

    wire       dclk_a, le_a, oe_a;
    wire [8:0] sdi_a;
    wire       isdi_a, idclk_a, irclk_a;
    wire       dclk_b, le_b, oe_b;
    wire [8:0] sdi_b;
    wire       isdi_b, idclk_b, irclk_b;
    reg spin_sync = 1'b0;

    pov_dual_top #(.DCLK_DIV(4)) dut (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'b000), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata_r), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'b000), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .dclk_out(dclk_a), .le_out(le_a), .oe_out(oe_a), .sdi_out(sdi_a),
        .icnd_sdi_out(isdi_a), .icnd_dclk_out(idclk_a), .icnd_rclk_out(irclk_a),
        .dclk_out_2(dclk_b), .le_out_2(le_b), .oe_out_2(oe_b), .sdi_out_2(sdi_b),
        .icnd_sdi_out_2(isdi_b), .icnd_dclk_out_2(idclk_b), .icnd_rclk_out_2(irclk_b),
        .spin_sync(spin_sync),
        .m_axi_araddr(m_araddr), .m_axi_arlen(m_arlen), .m_axi_arsize(m_arsize),
        .m_axi_arburst(m_arburst), .m_axi_arlock(m_arlock), .m_axi_arcache(m_arcache),
        .m_axi_arprot(m_arprot), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
        .m_axi_rdata(m_rdata), .m_axi_rresp(m_rresp), .m_axi_rlast(m_rlast),
        .m_axi_rvalid(m_rvalid), .m_axi_rready(m_rready)
    );

    integer errors = 0;
    task fail(input [511:0] msg);
        begin errors = errors + 1; $display("FAIL %0s @%0t", msg, $time); end
    endtask

    task axi_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            awaddr <= addr; wdata_r <= data;
            awvalid <= 1'b1; wvalid <= 1'b1; bready <= 1'b1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 1'b0; wvalid <= 1'b0;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
        end
    endtask

    // 采样统计: 一段窗口内 sdi_a[0] 高电平占比 + dclk 沿数
    integer hi_cnt, tot_cnt, dclk_edges;
    reg dclk_d;
    always @(posedge clk) dclk_d <= dclk_a;

    integer i, lane;
    initial begin
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (10) @(posedge clk);

        // 全 1 灌 lane0..8 (fb_A), rows=2 加速
        for (lane = 0; lane < 9; lane = lane + 1)
            for (i = 0; i < 16; i = i + 1)          // 2 行 × 8 pair
                axi_write(16'h8000 | (lane[3:0] << 11) | (i[8:0] << 2), 32'hFFFFFFFF);

        axi_write(16'h0010, (360 << 16));            // pov off
        axi_write(16'h000C, 32'h000001FF);           // sdi_mask
        axi_write(16'h000C, 32'h98023001);           // rows=2, oe=48, 双沿
        axi_write(16'h000C, 32'hC1000003);           // auto_en + use_fb

        // 跑 4 行时间 (~200 拍/行), 统计 sdi_a[0]
        hi_cnt = 0; tot_cnt = 0; dclk_edges = 0;
        repeat (8000) begin
            @(posedge clk);
            if (sdi_a[0]) hi_cnt = hi_cnt + 1;
            tot_cnt = tot_cnt + 1;
            if (dclk_a != dclk_d) dclk_edges = dclk_edges + 1;
            if (tot_cnt > 300 && tot_cnt < 800 && tot_cnt % 40 == 0)
                $display("t=%0d eg=%0d rdst=%0d div=%0d rbusy=%b rgo=%b oedone=%b advf=%b",
                         tot_cnt, dut.u_eng_a.u_core.eg_state_o,
                         dut.u_eng_a.u_core.u_row.st,
                         dut.u_eng_a.u_core.u_row.div,
                         dut.u_eng_a.u_core.row_busy_o,
                         dut.u_eng_a.u_core.row_go_r,
                         dut.u_eng_a.u_core.oe_done,
                         dut.u_eng_a.u_core.adv_fired);
        end
        $display("sdi_a[0] 高占比 = %0d/%0d (%0d%%), dclk 沿 = %0d",
                 hi_cnt, tot_cnt, (hi_cnt*100)/tot_cnt, dclk_edges);
        // 全 1 数据 + 移位窗占行周期大半 → sdi 高占比应 >60%
        if (hi_cnt * 100 < tot_cnt * 60)
            fail("sdi_a[0] 不是实心高: fb->移位数据通路断 (板上黑屏复现)");
        else
            $display("PASS sdi_a[0] 实心高: 数据通路 OK");
        if (dclk_edges < 100) fail("dclk 没跑");

        if (errors == 0) $display("=== TB RESULT: ALL CHECKS PASS ===");
        else             $display("=== TB RESULT: %0d FAIL ===", errors);
        $finish;
    end

endmodule
