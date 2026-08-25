//============================================================================
// lz4_axi_top.v — lz4_decode_core 的 AXI 包装
//
//   AXI-Lite slave  : 控制/状态寄存器
//   AXI4 master     : 读压缩流 + 写解压结果 (**共用一个口**)
//
// 🔴 为什么读写共用一个 AXI 主口: DR1 的 PS 只有 **2 个 HP slave**(Zynq 有 4 个,
//    见 reference_anlogic_dr1_fs03_eval §3)。panel core 的切片取数已占 HP0,
//    解码器只能用 HP1。好在 AXI4 的读写通道本来就是独立的, 一个主口够用。
//
// 带宽账 (目标 16.1 fps 满速):
//    panel 取数 ~142 MB/s + 解压写出 ~143 MB/s + 压缩流读入 ~4 MB/s ≈ 290 MB/s
//    (压缩比 33× ⇒ 读入几乎不占带宽, 这是 PL 解压相对"PS 解压再搬运"的额外好处)
//    DDR3L 16bit@1333 理论 2666 MB/s, 按 55% 效率 ~1.4 GB/s ⇒ 余量约 5×。
//    ⚠ 但 [[feedback_pov_4x_ip_breaks_hdmi]] 有前科(多 IP 同时打 DDR 把 HDMI 打成噪点),
//      集成后必须实测, 别只看算术。
//
// 结构:
//    AXI 读 → rd_fifo → (64b 拆成字节) → core.s_*
//    core.m_* → (字节攒成 64b) → wr_fifo → AXI 写
// 两级 FIFO 的作用是把**字节串行的核**与**突发的 AXI**解耦:
// 核每拍吞吐 1 字节, AXI 每拍 8 字节, 不解耦的话核会被 AXI 的地址/响应握手拖停。
//============================================================================

`timescale 1ns / 1ps

module lz4_axi_top #(
    parameter AXI_AW    = 32,
    parameter AXI_DW    = 64,          // 与 DR1 HP slave 位宽一致
    parameter BURST_LEN = 16           // INCR 突发拍数 (16×8B = 128B)
)(
    input  wire                  clk,
    input  wire                  rstn,

    // ---- AXI-Lite slave: 控制 ----
    input  wire [7:0]            s_awaddr,
    input  wire                  s_awvalid,
    output wire                  s_awready,
    input  wire [31:0]           s_wdata,
    input  wire                  s_wvalid,
    output wire                  s_wready,
    output wire                  s_bvalid,
    input  wire                  s_bready,
    input  wire [7:0]            s_araddr,
    input  wire                  s_arvalid,
    output wire                  s_arready,
    output reg  [31:0]           s_rdata,
    output wire                  s_rvalid,
    input  wire                  s_rready,

    // ---- AXI4 master: 读 ----
    output reg  [AXI_AW-1:0]     m_araddr,
    output reg  [7:0]            m_arlen,
    output reg                   m_arvalid,
    input  wire                  m_arready,
    input  wire [AXI_DW-1:0]     m_rdata,
    input  wire                  m_rlast,
    input  wire                  m_rvalid,
    output wire                  m_rready,

    // ---- AXI4 master: 写 ----
    output reg  [AXI_AW-1:0]     m_awaddr,
    output reg  [7:0]            m_awlen,
    output reg                   m_awvalid,
    input  wire                  m_awready,
    output wire [AXI_DW-1:0]     m_wdata,
    output wire [AXI_DW/8-1:0]   m_wstrb,     // 🔴 尾拍不足 8 字节时必须按字节屏蔽
    output wire                  m_wlast,
    output reg                   m_wvalid,
    input  wire                  m_wready,
    input  wire                  m_bvalid,
    output wire                  m_bready
);
    localparam BPW = AXI_DW/8;         // 每拍字节数 = 8

    // ---------------- 控制寄存器 ----------------
    // 0x00 CTRL   [0]=start(自清)
    // 0x04 STATUS [0]=done [1]=error [4:2]=err_code [5]=busy
    // 0x08 SRC_ADDR   0x0C SRC_LEN
    // 0x10 DST_ADDR   0x14 DST_LEN(=raw_len)
    // 0x18 CYCLES  (本次解码耗时, 用于实测 B/clk —— 别再靠估)
    reg [31:0] src_addr, src_len, dst_addr, dst_len, cycles;
    reg        start_p, busy;
    // 🔴 done_r 不是 core_done —— core 报完时最后几个字节可能还在 wr_acc / AXI 写通道里。
    //    软件必须等 done_r 才能读结果, 否则会读到尾巴写回前的旧数据。
    reg        done_r;

    assign s_awready = 1'b1;
    assign s_wready  = 1'b1;
    assign s_arready = 1'b1;
    reg bvalid_r, rvalid_r;
    assign s_bvalid = bvalid_r;
    assign s_rvalid = rvalid_r;

    wire core_done, core_err;
    wire [2:0] core_errcode;

    always @(posedge clk) begin
        if (!rstn) begin
            src_addr <= 0; src_len <= 0; dst_addr <= 0; dst_len <= 0;
            start_p <= 0; bvalid_r <= 0; rvalid_r <= 0;
        end else begin
            start_p <= 1'b0;                       // start 是单拍脉冲
            if (s_awvalid && s_wvalid) begin
                case (s_awaddr[7:2])
                    6'h0: start_p  <= s_wdata[0];
                    6'h2: src_addr <= s_wdata;
                    6'h3: src_len  <= s_wdata;
                    6'h4: dst_addr <= s_wdata;
                    6'h5: dst_len  <= s_wdata;
                    default: ;
                endcase
                bvalid_r <= 1'b1;
            end else if (s_bready) bvalid_r <= 1'b0;

            if (s_arvalid) begin
                case (s_araddr[7:2])
                    6'h1: s_rdata <= {26'd0, busy, core_errcode, core_err, done_r};
                    6'h2: s_rdata <= src_addr;
                    6'h3: s_rdata <= src_len;
                    6'h4: s_rdata <= dst_addr;
                    6'h5: s_rdata <= dst_len;
                    6'h6: s_rdata <= cycles;
                    default: s_rdata <= 32'hDEAD_0000;
                endcase
                rvalid_r <= 1'b1;
            end else if (s_rready) rvalid_r <= 1'b0;
        end
    end

    // ---------------- 读侧: AXI → 字节流 ----------------
    // rd_buf 是"正在按字节移出"的那一拍, nxt_buf 是**预取的下一拍**。
    // 🔴 为什么需要预取: 只有一级缓冲时, rd_buf 移空后才发 AR, 而 AR→RVALID
    //    在本仿真模型里就有 4 拍延迟 ⇒ 每 8 个压缩字节白等 4 拍。真 DDR 只会更长。
    //    双缓冲让 AR 与"上一拍的 8 字节正在被消费"重叠, 延迟被完全藏住。
    reg [AXI_DW-1:0] rd_buf;
    reg [3:0]        rd_cnt;           // rd_buf 里还剩几个字节
    reg [AXI_DW-1:0] nxt_buf;
    reg [3:0]        nxt_cnt;
    reg              nxt_val;
    reg [3:0]        req_cnt;          // 在途那一拍的有效字节数
    reg [31:0]       rd_left;          // 还有多少压缩字节**没发过读请求**
    reg [AXI_AW-1:0] rd_ptr;
    reg              rd_inflight;

    assign m_rready = 1'b1;            // 最多一笔在途且总有落脚处, 恒接收

    wire        s_valid = (rd_cnt != 0);
    wire [7:0]  s_data  = rd_buf[7:0];
    wire        s_ready;

    wire beat_in  = rd_inflight && m_rvalid;         // 本拍有新的一拍数据落地
    wire rd_fire  = s_valid && s_ready;              // 本拍消费掉一个字节
    // 本拍结束后 rd_buf 会不会空
    wire buf_empty_next = (rd_cnt == 4'd0) || (rd_fire && (rd_cnt == 4'd1));
    wire take_nxt = buf_empty_next && nxt_val;                  // 预取拍顶上来
    wire bypass   = buf_empty_next && !nxt_val && beat_in;      // 直通(预取级是空的)

    always @(posedge clk) begin
        if (!rstn) begin
            rd_cnt <= 0; nxt_cnt <= 0; nxt_val <= 0; req_cnt <= 0;
            rd_left <= 0; rd_ptr <= 0; m_arvalid <= 0; rd_inflight <= 0;
        end else if (start_p) begin
            rd_left <= src_len; rd_ptr <= src_addr;
            rd_cnt  <= 0; nxt_val <= 0; rd_inflight <= 0; m_arvalid <= 0;
        end else begin
            // ---- 发读请求: 预取级空着就可以再取一拍 ----
            if (!m_arvalid && !rd_inflight && !nxt_val && rd_left != 0) begin
                m_araddr  <= rd_ptr;
                m_arlen   <= 8'd0;      // 单拍读: 简单优先, 压缩流只占 1/33 带宽
                m_arvalid <= 1'b1;
                req_cnt   <= (rd_left >= BPW) ? BPW[3:0] : rd_left[3:0];
                rd_ptr    <= rd_ptr + BPW;
                rd_left   <= (rd_left >= BPW) ? rd_left - BPW : 32'd0;
            end
            if (m_arvalid && m_arready) begin
                m_arvalid <= 1'b0; rd_inflight <= 1'b1;
            end
            if (beat_in) rd_inflight <= 1'b0;

            // ---- 活动缓冲 ----
            if (take_nxt) begin
                rd_buf <= nxt_buf; rd_cnt <= nxt_cnt;
            end else if (bypass) begin
                rd_buf <= m_rdata;  rd_cnt <= req_cnt;
            end else if (rd_fire) begin
                rd_buf <= {8'd0, rd_buf[AXI_DW-1:8]};   // 右移一个字节
                rd_cnt <= rd_cnt - 1'b1;
            end

            // ---- 预取级 ----
            if (beat_in && !bypass) begin
                nxt_buf <= m_rdata; nxt_cnt <= req_cnt; nxt_val <= 1'b1;
            end else if (take_nxt) begin
                nxt_val <= 1'b0;
            end
        end
    end

    // ---------------- 解码核 ----------------
    wire       m_valid_c, m_last_c;
    wire [7:0] m_data_c;
    wire       m_ready_c;

    lz4_decode_core #(.HIST_AW(16)) u_core (
        .clk(clk), .rstn(rstn), .start(start_p), .raw_len(dst_len),
        .done(core_done), .error(core_err), .err_code(core_errcode),
        .s_valid(s_valid), .s_data(s_data), .s_ready(s_ready),
        .m_valid(m_valid_c), .m_data(m_data_c), .m_ready(m_ready_c), .m_last(m_last_c)
    );

    // ---------------- 写侧: 字节流 → AXI ----------------
    // 攒满 8 字节发一拍。⚠ 末尾不足 8 字节的尾巴要发出去,
    //   否则最后几个字节永远留在 wr_acc 里 —— 这是这类桥最常见的漏洞。
    //
    // 🔴 两级: wr_acc(正在攒) + wr_word(已攒满、等 AXI 发走)。
    //    只有一级的话, 每攒满 8 字节就得停核等写事务做完(约 2 拍) ⇒ 上限 0.8 B/clk。
    //    两级后核只在"下一拍又攒满而上一拍还没发走"时才停 ⇒ 实测不停。
    //
    // 🔴 m_ready_c 必须是**组合**的。写成寄存器 (`m_ready_c <= !wr_pend`) 时,
    //    wr_pend 拉高那一拍 m_ready_c 还是旧值 1, 核据此认为字节已被接收并前进,
    //    而包装里 `&& !wr_pend` 又不肯收 ⇒ **每 8 字节静默丢 1 字节**。
    //    (这就是第一次仿真 12288 字节错 11600 的根因。)
    reg [AXI_DW-1:0] wr_acc;           // 正在攒的字节 (byte i 放在第 i 个字节位)
    reg [3:0]        wr_cnt;           // wr_acc 里已有几个字节
    reg [AXI_DW-1:0] wr_word;          // 待发出的一拍
    reg [BPW-1:0]    wr_strb;
    reg [AXI_AW-1:0] wr_ptr;
    reg              wr_pend;          // 有一拍待写
    reg              flush;            // 收到 m_last, 需要把尾巴冲出去

    // 按字节位插入: 天然对齐, 尾拍直接配 strb 即可, 不需要移位补零
    wire [AXI_DW-1:0] byte_mask = {{(AXI_DW-8){1'b0}}, 8'hFF} << (8*wr_cnt);
    wire [AXI_DW-1:0] acc_next  = (wr_acc & ~byte_mask) |
                                  (({{(AXI_DW-8){1'b0}}, m_data_c}) << (8*wr_cnt));

    // 只有"本拍会攒满 && 上一拍还没发走"才需要反压核
    assign m_ready_c = !(wr_pend && (wr_cnt == BPW-1));

    assign m_wdata  = wr_word;
    assign m_wstrb  = wr_strb;
    assign m_wlast  = 1'b1;            // 单拍突发
    assign m_bready = 1'b1;

    // 🔴 还要等 **B 响应** 全部回来才算写完。只看 WVALID/WREADY 握手只能说明数据
    //    交给了互连, 不代表已经在 DDR 里可见 —— done_r 一旦早报, PS 读结果就可能
    //    读到旧数据, 而且这种错在板上是偶发的、极难查。bcnt = 未回 B 的写事务数。
    reg [3:0] bcnt;
    wire w_fire_i = m_wvalid && m_wready;
    wire b_fire_i = m_bvalid && m_bready;

    wire wr_drained = !wr_pend && !flush && !m_awvalid && !m_wvalid && (bcnt == 0);
    // 🔴 core_done 只高**一拍**(S_DONE 后立刻回 S_IDLE, S_IDLE 又把 done 清 0),
    //    而那一拍尾巴还压在写通道里 ⇒ 必须锁存, 否则 done_r 永远等不到。
    reg core_done_l;

    always @(posedge clk) begin
        if (!rstn) begin
            wr_cnt <= 0; wr_ptr <= 0; wr_pend <= 0; m_awvalid <= 0; m_wvalid <= 0;
            flush <= 0; wr_strb <= 0; busy <= 0; cycles <= 0; done_r <= 0;
            core_done_l <= 0; bcnt <= 0;
        end else begin
            // 未回 B 的写事务计数 (发出一拍 +1, 回一个 B -1)
            if (w_fire_i && !b_fire_i)      bcnt <= bcnt + 1'b1;
            else if (b_fire_i && !w_fire_i) bcnt <= bcnt - 1'b1;

            if (start_p) begin
                wr_cnt <= 0; wr_ptr <= dst_addr; wr_pend <= 0; flush <= 0;
                wr_acc <= {AXI_DW{1'b0}};
                busy <= 1'b1; cycles <= 0; done_r <= 1'b0; core_done_l <= 1'b0;
            end else begin
                if (core_done) core_done_l <= 1'b1;
                if (busy) cycles <= cycles + 1'b1;
                // 🔴 core_done 不代表写完: 还要等尾拍真的落到 DDR
                if (busy && (core_done || core_done_l) && wr_drained) begin
                    busy <= 1'b0; done_r <= 1'b1;
                end
                if (busy && core_err) busy <= 1'b0;
            end

            // ---- 核有输出就接收 ----
            if (m_valid_c && m_ready_c) begin
                wr_acc <= acc_next;
                if (wr_cnt == BPW-1) begin
                    wr_word <= acc_next; wr_strb <= {BPW{1'b1}};
                    wr_pend <= 1'b1; wr_cnt <= 0;
                end else begin
                    wr_cnt <= wr_cnt + 1'b1;
                    if (m_last_c) flush <= 1'b1;   // 尾巴不足一拍
                end
            end

            // ---- 尾拍: 用 wstrb 屏蔽无效字节, 不越界写 dst 缓冲 ----
            if (flush && !wr_pend) begin
                wr_word <= wr_acc;
                wr_strb <= ({{(BPW-1){1'b0}}, 1'b1} << wr_cnt) - 1'b1;
                wr_pend <= 1'b1; flush <= 1'b0; wr_cnt <= 0;
            end

            // ---- 发写请求 ----
            if (wr_pend && !m_awvalid && !m_wvalid) begin
                m_awaddr <= wr_ptr; m_awlen <= 8'd0;
                m_awvalid <= 1'b1;  m_wvalid <= 1'b1;
            end
            if (m_awvalid && m_awready) m_awvalid <= 1'b0;
            if (m_wvalid  && m_wready ) begin
                m_wvalid <= 1'b0; wr_pend <= 1'b0; wr_ptr <= wr_ptr + BPW;
            end
        end
    end

endmodule
