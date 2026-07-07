//-----------------------------------------------------------------------------
// tb_panel_fb.v - self-checking testbench for icnd2049_panel_fb auto/use_fb
//
// Config under test:
//   scan rows = 4  (0x0C <= 0x8004_0001)
//   sdi_mask  = 0x1FF
//   fb: lane0 row0 words 0..11 = 0xA001..0xA00C, lane1 row0 = 0x5555 x12,
//       all other lane/row (rows 0..3) = 0
//   auto: 0x0C <= 0xC100_0003  (auto_en=1, use_fb=1, disp window = 1*1024 aclk)
//
// Checks per row-slot (8 slots = 2 frames of 4 rows):
//   a. exactly 192 dclk_out rising edges
//   b. sdi_out[0] reassembled (MSB first) == injected lane0 words (row0 slots)
//   c. sdi_out[1] reassembled == 0x5555 x12 (row0 slots), 0 otherwise
//   d. le_out covers exactly the last 5 (row0) / 4 (rows 1..3) dclk rising
//      edges of the slot, 0 elsewhere
//   e. oe_out low window == auto_disp_cyc (1024) +/- 2 aclk
//   f. one icnd_dclk_out pulse per slot, icnd_sdi_out==1 only on row-0 slots
//   g. slot 4 (5th) wraps back to row0 data
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_panel_fb;

    // ---------------- clock / reset ----------------
    reg clk  = 1'b0;
    reg rstn = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    // ---------------- AXI-Lite signals ----------------
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

    // ---------------- DUT outputs ----------------
    wire       dclk, le, oe;
    wire [8:0] sdi;
    wire       i_sdi, i_dclk, i_rclk;

    icnd2049_panel_fb #(.DCLK_DIV(4)) dut (
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
        .icnd_rclk_out (i_rclk)
    );

    // ---------------- AXI-Lite write task ----------------
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

    // ---------------- per-slot collectors ----------------
    localparam integer NSLOT = 8;
    localparam integer DISP_EXP = 1024;   // [29:24]=1 -> 1*1024 aclk

    reg          collecting  = 1'b0;
    reg          frames_done = 1'b0;
    integer      slot;
    integer      dclk_cnt, le_cnt, icnd_cnt, oelow_cnt;
    integer      first_le, last_le;
    reg  [15:0]  sr0, sr1;
    reg          isdi_val;
    integer      le_glitch;   // le high while sequencer idle (should never happen)

    reg  [15:0]  w0 [0:NSLOT-1][0:11];
    reg  [15:0]  w1 [0:NSLOT-1][0:11];
    integer      r_dclk  [0:NSLOT-1];
    integer      r_le    [0:NSLOT-1];
    integer      r_fle   [0:NSLOT-1];
    integer      r_lle   [0:NSLOT-1];
    integer      r_icnd  [0:NSLOT-1];
    reg          r_isdi  [0:NSLOT-1];
    integer      r_oelow [0:NSLOT-1];

    initial begin
        slot = 0; dclk_cnt = 0; le_cnt = 0; icnd_cnt = 0; oelow_cnt = 0;
        first_le = -1; last_le = -1; sr0 = 16'b0; sr1 = 16'b0;
        isdi_val = 1'b0; le_glitch = 0;
    end

    // sample serial streams on dclk rising edge
    always @(posedge dclk) if (collecting && !frames_done) begin
        sr0 = {sr0[14:0], sdi[0]};
        sr1 = {sr1[14:0], sdi[1]};
        if (le === 1'b1) begin
            le_cnt = le_cnt + 1;
            if (first_le < 0) first_le = dclk_cnt;
            last_le = dclk_cnt;
        end
        dclk_cnt = dclk_cnt + 1;
        if ((dclk_cnt % 16) == 0 && dclk_cnt <= 192 && slot < NSLOT) begin
            w0[slot][dclk_cnt/16 - 1] = sr0;
            w1[slot][dclk_cnt/16 - 1] = sr1;
        end
    end

    // icnd row-advance pulse
    always @(posedge i_dclk) if (collecting && !frames_done) begin
        icnd_cnt = icnd_cnt + 1;
        isdi_val = i_sdi;
    end

    // oe low duration (in aclk) + le sanity outside shifting
    always @(posedge clk) if (collecting && !frames_done) begin
        if (oe === 1'b0) oelow_cnt = oelow_cnt + 1;
        if (le === 1'b1 && dut.bits_left == 7'd0) le_glitch = le_glitch + 1;
    end

    // slot boundary = oe rising edge (end of display window)
    always @(posedge oe) if (collecting && !frames_done && slot < NSLOT) begin
        r_dclk[slot]  = dclk_cnt;
        r_le[slot]    = le_cnt;
        r_fle[slot]   = first_le;
        r_lle[slot]   = last_le;
        r_icnd[slot]  = icnd_cnt;
        r_isdi[slot]  = isdi_val;
        r_oelow[slot] = oelow_cnt;
        dclk_cnt = 0; le_cnt = 0; icnd_cnt = 0; oelow_cnt = 0;
        first_le = -1; last_le = -1; sr0 = 16'b0; sr1 = 16'b0; isdi_val = 1'b0;
        slot = slot + 1;
        if (slot == NSLOT) frames_done = 1'b1;
    end

    // ---------------- stimulus ----------------
    integer lane, row, pair, s, k;
    integer errors;
    reg [15:0] exp0, exp1;
    integer exp_le;
    reg exp_isdi;

    initial begin
        errors = 0;
        rstn = 1'b0;
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (5) @(posedge clk);

        // scan rows = 4 (subcmd 10, [24:16]=4, OE=1 blank)
        axi_write(16'h000C, 32'h8004_0001);
        // sdi_mask = 0x1FF (subcmd 00)
        axi_write(16'h000C, 32'h0000_01FF);

        // clear fb: lanes 0..8, rows 0..3, pairs 0..5
        for (lane = 0; lane < 9; lane = lane + 1)
            for (row = 0; row < 4; row = row + 1)
                for (pair = 0; pair < 6; pair = pair + 1)
                    axi_write(16'h8000 | (lane<<11) | (row<<5) | (pair<<2), 32'h0);

        // lane0 row0: words 0..11 = 0xA001..0xA00C
        // pair p: low half = word(2p), high half = word(2p+1)
        for (pair = 0; pair < 6; pair = pair + 1)
            axi_write(16'h8000 | (pair<<2),
                      ((32'hA002 + pair*2) << 16) | (32'hA001 + pair*2));

        // lane1 row0: 0x5555 x12
        for (pair = 0; pair < 6; pair = pair + 1)
            axi_write(16'h8000 | (1<<11) | (pair<<2), 32'h5555_5555);

        // start collecting, then enable auto+use_fb, disp = 1*1024 aclk
        collecting = 1'b1;
        axi_write(16'h000C, 32'hC100_0003);

        wait (frames_done);
        repeat (10) @(posedge clk);

        // ================= checks =================
        // a. 192 dclk edges per slot
        for (s = 0; s < NSLOT; s = s + 1) begin
            if (r_dclk[s] !== 192) begin
                errors = errors + 1;
                $display("FAIL [a] slot %0d: dclk edges exp 192 got %0d", s, r_dclk[s]);
            end
        end
        if (errors == 0) $display("PASS [a] all 8 slots: exactly 192 dclk rising edges");

        // b/c/g. lane0 / lane1 word data (slot%4==0 -> row0 data, else zeros)
        k = errors;
        for (s = 0; s < NSLOT; s = s + 1) begin
            for (pair = 0; pair < 12; pair = pair + 1) begin
                if ((s % 4) == 0) begin
                    exp0 = 16'hA001 + pair[15:0];
                    exp1 = 16'h5555;
                end else begin
                    exp0 = 16'h0000;
                    exp1 = 16'h0000;
                end
                if (w0[s][pair] !== exp0) begin
                    errors = errors + 1;
                    $display("FAIL [b] slot %0d lane0 word %0d: exp %04x got %04x",
                             s, pair, exp0, w0[s][pair]);
                end
                if (w1[s][pair] !== exp1) begin
                    errors = errors + 1;
                    $display("FAIL [c] slot %0d lane1 word %0d: exp %04x got %04x",
                             s, pair, exp1, w1[s][pair]);
                end
            end
        end
        if (errors == k) begin
            $display("PASS [b] sdi_out[0] lane0 row0 words == 0xA001..0xA00C (slots 0,4), zeros elsewhere");
            $display("PASS [c] sdi_out[1] lane1 row0 words == 0x5555 x12 (slots 0,4), zeros elsewhere");
            $display("PASS [g] slot 4 (5th row-slot) wrapped back to row0 data");
        end

        // d. LE covers exactly last 5 (row0) / 4 dclk rising edges
        k = errors;
        for (s = 0; s < NSLOT; s = s + 1) begin
            exp_le = ((s % 4) == 0) ? 5 : 4;
            if (r_le[s] !== exp_le || r_fle[s] !== (192 - exp_le) || r_lle[s] !== 191) begin
                errors = errors + 1;
                $display("FAIL [d] slot %0d: le edges exp %0d@[%0d..191] got %0d@[%0d..%0d]",
                         s, exp_le, 192-exp_le, r_le[s], r_fle[s], r_lle[s]);
            end
        end
        if (le_glitch != 0) begin
            errors = errors + 1;
            $display("FAIL [d] le_out high outside shifting: %0d aclk cycles", le_glitch);
        end
        if (errors == k) $display("PASS [d] LE covers exactly last 5 (row0) / 4 (row1..3) dclk edges, 0 elsewhere");

        // e. oe low window == 1024 +/- 2 aclk
        k = errors;
        for (s = 0; s < NSLOT; s = s + 1) begin
            if (r_oelow[s] < DISP_EXP-2 || r_oelow[s] > DISP_EXP+2) begin
                errors = errors + 1;
                $display("FAIL [e] slot %0d: oe low exp %0d+/-2 aclk got %0d", s, DISP_EXP, r_oelow[s]);
            end
        end
        if (errors == k) $display("PASS [e] oe_out low window = %0d aclk (exp %0d +/- 2) per slot",
                                  r_oelow[0], DISP_EXP);

        // f. one icnd_dclk pulse per slot, icnd_sdi=1 only on row-0 slots
        k = errors;
        for (s = 0; s < NSLOT; s = s + 1) begin
            exp_isdi = ((s % 4) == 0);
            if (r_icnd[s] !== 1) begin
                errors = errors + 1;
                $display("FAIL [f] slot %0d: icnd_dclk pulses exp 1 got %0d", s, r_icnd[s]);
            end
            if (r_isdi[s] !== exp_isdi) begin
                errors = errors + 1;
                $display("FAIL [f] slot %0d: icnd_sdi exp %0d got %0d", s, exp_isdi, r_isdi[s]);
            end
        end
        if (errors == k) $display("PASS [f] 1 icnd_dclk pulse per slot, icnd_sdi=1 only on row-0 slots (0,4)");

        // ================= summary =================
        if (errors == 0)
            $display("=== TB RESULT: ALL CHECKS PASS (8 row-slots = 2 frames) ===");
        else
            $display("=== TB RESULT: %0d FAILURES ===", errors);
        $finish;
    end

    // watchdog
    initial begin
        #3_000_000;   // 3 ms
        $display("FAIL: TIMEOUT - frames_done not reached. slot=%0d dclk_cnt=%0d oe=%b auto_state=%0d",
                 slot, dclk_cnt, oe, dut.au_state);
        $display("=== TB RESULT: TIMEOUT ===");
        $finish;
    end

endmodule
