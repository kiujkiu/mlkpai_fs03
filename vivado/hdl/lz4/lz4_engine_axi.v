//==============================================================================
// lz4_engine_axi.v — dr1v90/lz4hw 的 lz4_axi_top 在本工程 BD 里的**适配壳**
//                    (2026-08-24, feature/3bit-color)
//
// 🔴 这一层只做协议适配, **解码逻辑一行没动** —— lz4_axi_top.v / lz4_decode_core.v
//    是从 dr1v90/lz4hw/rtl 原样复制过来的 (16/16 真 liblz4 向量已过)。
//    改它们等于把已验证的东西重新变成未验证的, 不干。
//
// 为什么必须有这一层 —— lz4_axi_top 的两个口都**不是**标准 AXI:
//
//  (1) 它的 "AXI-Lite slave" 其实是个简化寄存器口:
//        - s_awready / s_wready / s_arready 恒 1
//        - **写只在 `s_awvalid && s_wvalid` 同拍时才生效**
//        - 没有 awprot / wstrb / bresp / rresp
//      dr1v90 那边的 tb_lz4_axi.v 正是同拍拉一拍两个 valid (sim/tb_lz4_axi.v:128),
//      所以仿真里从来没暴露过问题。但真 AXI-Lite 主端 (GP0 → axi_interconnect
//      → 协议转换器) **不保证** AW 和 W 同拍到:
//        - AW 先到: awready 恒 1 ⇒ AW 被"吃掉"且丢弃, 且 bvalid 永不拉高
//          ⇒ CPU 的这次写永远等不到 B 响应 ⇒ **AXI 挂死**(不是写错, 是死锁)。
//        - 背靠背两次写: bvalid_r 可能一直是 1 没落过 ⇒ 互连收到的 B 少一个 ⇒ 同样挂。
//      ⇒ 本层实现一个**完整合规的 AXI4-Lite 从口**, 把 AW/W 攒齐后再一拍脉冲喂
//        给 lz4_axi_top, B/R 响应自己发。互连侧看到的永远是合规时序。
//
//  (2) 它的 AXI4 master 缺 awsize/awburst/awcache/awprot/arsize/... 一堆信号。
//      BD 里 module_ref 推断接口时这些是"可选"的, 缺了会被 IPI 用默认值补 ——
//      **AWSIZE 补错就是 64bit 口当 1B/beat 用**, wstrb 全对不上, 板上表现是
//      "解出来的图有周期性花条"这种最难查的错。⇒ 本层把整套 AXI4 信号补齐并
//      显式常数化 (arcache/arprot 抄 ddr_slice_fetch256 已验证的 4'b0011 / 3'b000)。
//
// 🔴 地址对齐契约 (软件必须遵守, 硬件不做兜底):
//      AXI_DW=64 ⇒ 读写都是 8 字节/拍, lz4_axi_top 假定 rd_buf[7:0] 就是
//      src_addr 指向的那个字节、wstrb 的 bit0 就是 dst_addr 那个字节。
//      ⇒ **SRC_ADDR / DST_ADDR 必须 8 字节对齐**。
//      DST 天然满足 (bank_base + Σn_slices*0x9000, 0x9000 是 8 的倍数);
//      SRC 是流表里 Σcomp_len_j 累加出来的**任意**偏移 ⇒ 软件要么把每条流
//      memcpy 到 8B 对齐的暂存区 (317KB/帧, ~0.3ms, 可忽略), 要么让发送端
//      把 comp_len_i 补齐到 8 的倍数 (尾部多余字节无害: 核解够 raw_len 就停)。
//
// 寄存器 (窗口 64KB, 但只译码 awaddr[7:0] ⇒ 窗内每 256B 一个镜像):
//   0x00 W CTRL   [0]=start (自清脉冲)
//   0x04 R STATUS [0]=done [1]=error [4:2]=err_code [5]=busy
//   0x08 RW SRC_ADDR   0x0C RW SRC_LEN
//   0x10 RW DST_ADDR   0x14 RW DST_LEN (=raw_len)
//   0x18 R  CYCLES  (本次解码的 aclk 拍数 —— 顺带是个**独立的 aclk 频率探针**,
//                    见 pov_dual_top.v 里那桩 "50MHz 还是 25MHz" 的悬案)
//==============================================================================
`timescale 1ns / 1ps

module lz4_engine_axi #(
    parameter AXI_AW  = 32,
    parameter AXI_DW  = 64,
    // 写突发打包器: 1 = 把核发的 N 笔单拍写攒成一笔 INCR 突发, 0 = 直通(原样单拍)
    parameter WPACK   = 1,
    parameter WPACK_N = 16          // 最大突发拍数 (16 拍 × 8 B = 128 B)
                                    // 🔴 16 不是随便取的: PS 的 HP slave 是 **AXI3**,
                                    //    AXI3 的 AWLEN 只有 4 bit ⇒ 一笔最多 16 拍。
                                    //    axi_smc 做 AXI4→AXI3 转换时会把更长的突发
                                    //    拆开, 拆了就白攒。16 正好是"不被拆的最大值"。
)(
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axi" *)
    input  wire                  s_axi_aclk,
    input  wire                  s_axi_aresetn,

    // ---------------- AXI4-Lite slave (合规) ----------------
    input  wire [15:0]           s_axi_awaddr,
    input  wire [2:0]            s_axi_awprot,
    input  wire                  s_axi_awvalid,
    output reg                   s_axi_awready,
    input  wire [31:0]           s_axi_wdata,
    input  wire [3:0]            s_axi_wstrb,
    input  wire                  s_axi_wvalid,
    output reg                   s_axi_wready,
    output wire [1:0]            s_axi_bresp,
    output reg                   s_axi_bvalid,
    input  wire                  s_axi_bready,
    input  wire [15:0]           s_axi_araddr,
    input  wire [2:0]            s_axi_arprot,
    input  wire                  s_axi_arvalid,
    output reg                   s_axi_arready,
    output reg  [31:0]           s_axi_rdata,
    output wire [1:0]            s_axi_rresp,
    output reg                   s_axi_rvalid,
    input  wire                  s_axi_rready,

    // ---------------- AXI4 master (读压缩流 + 写解压结果, 共用一口) ----------
    output wire [AXI_AW-1:0]     m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire                  m_axi_awlock,
    output wire [3:0]            m_axi_awcache,
    output wire [2:0]            m_axi_awprot,
    output wire                  m_axi_awvalid,
    input  wire                  m_axi_awready,
    output wire [AXI_DW-1:0]     m_axi_wdata,
    output wire [AXI_DW/8-1:0]   m_axi_wstrb,
    output wire                  m_axi_wlast,
    output wire                  m_axi_wvalid,
    input  wire                  m_axi_wready,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output wire                  m_axi_bready,

    output wire [AXI_AW-1:0]     m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arlock,
    output wire [3:0]            m_axi_arcache,
    output wire [2:0]            m_axi_arprot,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,
    input  wire [AXI_DW-1:0]     m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready
);
    wire clk  = s_axi_aclk;
    wire rstn = s_axi_aresetn;

    // AXI-Lite 响应恒 OKAY: 寄存器口不会产生 SLVERR (未定义地址读回 0xDEAD0000,
    // 那是数据不是错误响应 —— 让互连挂在 DECERR 上更难查)
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    // ---------------- 喂给 lz4_axi_top 的简化寄存器口 ----------------
    reg  [7:0]  l_awaddr;
    reg  [31:0] l_wdata;
    reg         l_awvalid, l_wvalid;
    reg  [7:0]  l_araddr;
    reg         l_arvalid;
    wire [31:0] l_rdata;
    wire        l_rvalid, l_bvalid;

    // ---------------- 写通道 FSM ----------------
    // W_IDLE: awready/wready 都拉高, AW 与 W 各自到齐后落 ready (每次只收一笔)
    // W_FIRE: 同拍拉一拍 l_awvalid & l_wvalid → lz4_axi_top 的 case 生效
    // W_RESP: 发 BVALID, 等 BREADY
    localparam W_IDLE = 2'd0, W_FIRE = 2'd1, W_RESP = 2'd2;
    reg [1:0] wst;
    reg       aw_ok, w_ok;

    always @(posedge clk) begin
        if (!rstn) begin
            wst <= W_IDLE;
            s_axi_awready <= 1'b1; s_axi_wready <= 1'b1; s_axi_bvalid <= 1'b0;
            aw_ok <= 1'b0; w_ok <= 1'b0; l_awvalid <= 1'b0; l_wvalid <= 1'b0;
            l_awaddr <= 8'd0; l_wdata <= 32'd0;
        end else begin
            l_awvalid <= 1'b0;
            l_wvalid  <= 1'b0;
            case (wst)
            W_IDLE: begin
                if (s_axi_awvalid && s_axi_awready) begin
                    l_awaddr      <= s_axi_awaddr[7:0];
                    aw_ok         <= 1'b1;
                    s_axi_awready <= 1'b0;
                end
                if (s_axi_wvalid && s_axi_wready) begin
                    l_wdata      <= s_axi_wdata;
                    w_ok         <= 1'b1;
                    s_axi_wready <= 1'b0;
                end
                // 两边都到齐 (含"本拍才到"的那一笔) ⇒ 下一拍开火
                if ((aw_ok || (s_axi_awvalid && s_axi_awready)) &&
                    (w_ok  || (s_axi_wvalid  && s_axi_wready ))) wst <= W_FIRE;
            end
            W_FIRE: begin
                l_awvalid <= 1'b1;      // 只高一拍 —— start 位靠这个才是单拍脉冲
                l_wvalid  <= 1'b1;
                wst       <= W_RESP;
            end
            W_RESP: begin
                s_axi_bvalid <= 1'b1;
                if (s_axi_bvalid && s_axi_bready) begin
                    s_axi_bvalid  <= 1'b0;
                    aw_ok <= 1'b0; w_ok <= 1'b0;
                    s_axi_awready <= 1'b1; s_axi_wready <= 1'b1;
                    wst           <= W_IDLE;
                end
            end
            default: wst <= W_IDLE;
            endcase
        end
    end

    // ---------------- 读通道 FSM ----------------
    // lz4_axi_top 在 s_arvalid 那拍把 s_rdata 打进寄存器 ⇒ 下一拍 l_rvalid=1 时取走
    localparam R_IDLE = 2'd0, R_FIRE = 2'd1, R_WAIT = 2'd2, R_RESP = 2'd3;
    reg [1:0] rst_st;

    always @(posedge clk) begin
        if (!rstn) begin
            rst_st <= R_IDLE;
            s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
            l_arvalid <= 1'b0; l_araddr <= 8'd0; s_axi_rdata <= 32'd0;
        end else begin
            l_arvalid <= 1'b0;
            case (rst_st)
            R_IDLE: if (s_axi_arvalid && s_axi_arready) begin
                        l_araddr      <= s_axi_araddr[7:0];
                        s_axi_arready <= 1'b0;
                        rst_st        <= R_FIRE;
                    end
            R_FIRE: begin l_arvalid <= 1'b1; rst_st <= R_WAIT; end
            R_WAIT: if (l_rvalid) begin
                        // 🔴 STATUS(0x04) 要被打包器压一手:
                        //    核的 done 只代表"它自己的 B 都回了", 而打包器里可能
                        //    还压着没发出去的拍 / 在途的外侧突发。软件读到 done=1
                        //    就会去读结果 ⇒ 必须等 pack_busy 落下来才放行。
                        //    这就是原 lz4_axi_top 用 bcnt 做的那件事, 只是记账
                        //    位置从核挪到了壳 (理由见 g_pack 上方 "B 响应记账")。
                        if (l_araddr[7:2] == 6'h1)
                            s_axi_rdata <= {l_rdata[31:6],
                                            (l_rdata[5] | pack_busy),   // busy
                                            l_rdata[4:1],               // err_code/error
                                            (l_rdata[0] & ~pack_busy)}; // done
                        else
                            s_axi_rdata <= l_rdata;
                        s_axi_rvalid <= 1'b1;
                        rst_st       <= R_RESP;
                    end
            R_RESP: if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid  <= 1'b0;
                        s_axi_arready <= 1'b1;
                        rst_st        <= R_IDLE;
                    end
            endcase
        end
    end

    //=========================================================================
    // 写突发打包器 (WPACK)
    //
    // 为什么要它: lz4_axi_top 里的 `BURST_LEN` 参数是**死参数** —— 全文件只在声明
    //   处出现一次, 从没被引用; 实际硬编码 m_awlen=0 / m_arlen=0 / m_wlast=1,
    //   即**每 8 字节一笔独立 AXI 突发**。10.47 MB/帧 × 11.1 帧/s ÷ 8 B
    //   ≈ 3 个引擎合计 1450 万笔写事务/秒。
    //   更要命的是 DDR 侧: 16-bit DDR3 的 BL8 = 一次最少 16 B, 8 B 的写要靠 DM
    //   屏蔽, **占满一整个 BL8 槽** ⇒ 写效率上限 50%, 116 MB/s 有效写要吃掉
    //   232 MB/s 的 DDR 总线时间。攒成 128 B 一笔后是 8 个满 BL8 ⇒ 100%。
    //   在一个有 feedback_pov_4x_ip_breaks_hdmi 前科的板子上, 把 DDR 写占用减半
    //   是最划算的保险。
    //
    // 🔴 为什么放在这一层而不是改 lz4_axi_top: 那两个文件 16/16 真 liblz4 向量已过,
    //    改它们等于把已验证的东西重新变成未验证的。打包器在壳里 = 核零改动。
    //
    // 🔴 最容易踩的坑 (B 响应记账):
    //    lz4_axi_top 里 `reg [3:0] bcnt` 记"发出去还没回 B 的写事务数", 而
    //    `wr_drained` 要求 bcnt==0 才允许报 done。**4 bit 到 16 会回绕成 0** ——
    //    如果打包器攒满 16 拍才回 B, 核的 bcnt 正好可能撞上 16 ⇒ wr_drained 假成立
    //    ⇒ done 早报 ⇒ 软件读到尾巴写回前的旧数据, 板上偶发、极难查。
    //    ⇒ 本打包器**收下一拍就立刻给核回一个 B**(纯流控, 不代表已落 DDR),
    //      核的 bcnt 因此永远 ≤1; "数据真的落了 DDR 才算完" 这条保证挪到本层:
    //      对外的 STATUS.done 被 pack_busy 压住 (见下面读通道), 只有外侧突发的
    //      B 全部回来、FIFO 全空, done 才放出去。语义没变, 记账位置变了。
    //=========================================================================
    localparam BPW = AXI_DW/8;

    // 核侧(inner)写通道
    wire [AXI_AW-1:0] i_awaddr;  wire [7:0] i_awlen;  wire i_awvalid;  wire i_awready;
    wire [AXI_DW-1:0] i_wdata;   wire [BPW-1:0] i_wstrb; wire i_wlast;
    wire i_wvalid;  wire i_wready;
    wire i_bvalid;  wire i_bready;

    wire pack_busy;

    generate
    if (WPACK == 0) begin : g_nopack
        assign m_axi_awaddr  = i_awaddr;
        assign m_axi_awlen   = i_awlen;
        assign m_axi_awvalid = i_awvalid;
        assign i_awready     = m_axi_awready;
        assign m_axi_wdata   = i_wdata;
        assign m_axi_wstrb   = i_wstrb;
        assign m_axi_wlast   = i_wlast;
        assign m_axi_wvalid  = i_wvalid;
        assign i_wready      = m_axi_wready;
        assign i_bvalid      = m_axi_bvalid;
        assign m_axi_bready  = i_bready;
        assign pack_busy     = 1'b0;
    end else begin : g_pack
        localparam DDEPTH = 32;             // 数据 FIFO 深度 (>= 2 个满突发)
        localparam GDEPTH = 4;              // 已封口突发的描述符 FIFO 深度
        localparam IDLE_N = 8'd64;          // 空闲多少拍就把没攒满的组封口
                                            // (满速时核每 8 拍来一拍, 64 不会误触发)

        reg [AXI_DW-1:0] pd [0:DDEPTH-1];
        reg [BPW-1:0]    ps [0:DDEPTH-1];
        reg [4:0]        dw_ptr, dr_ptr;
        reg [5:0]        dcount;

        reg [AXI_AW-1:0] ga [0:GDEPTH-1];
        reg [4:0]        gl [0:GDEPTH-1];
        reg [1:0]        gw_ptr, gr_ptr;
        reg [2:0]        gcount;

        // 正在攒的那一组
        reg              og_v;
        reg [AXI_AW-1:0] og_addr, og_next;
        reg [4:0]        og_len;
        reg [7:0]        og_idle;

        // AW/W 配对级 (AXI 允许两个通道分开到达, 不能假定同拍)
        reg              aw_v;  reg [AXI_AW-1:0] aw_a;
        reg              w_v;   reg [AXI_DW-1:0] w_d;  reg [BPW-1:0] w_s;

        reg [3:0] bcred;                    // 回给核的 B 信用
        reg [3:0] obcnt;                    // 外侧在途突发数 (HP 口上限 8)

        // ---- 发送 FSM (先声明, 上面的入队逻辑要用 snd_take/snd_pop) ----
        localparam S_IDLE = 1'b0, S_RUN = 1'b1;
        reg              snd_st;
        reg [AXI_AW-1:0] snd_addr;
        reg [4:0]        snd_left;          // 还剩几拍没发
        reg [4:0]        snd_len_r;         // 🔴 本笔突发的总拍数, **锁存不变** ——
                                            //    AWLEN 在 AWVALID 期间不许变 (AXI 规矩),
                                            //    直接用 snd_left 算就会在首拍 W 先于
                                            //    AWREADY 时把 AWLEN 变掉 = 协议违例
        reg              awv_r, wv_r;

        wire snd_take = (snd_st == S_IDLE) && (gcount != 0) && (obcnt != 4'd8);
        wire snd_pop  = wv_r && m_axi_wready;

        // ---- 入队 ----
        wire room     = (dcount <= DDEPTH-2) && (gcount <= GDEPTH-2);
        assign i_awready = !aw_v;
        assign i_wready  = !w_v && room;
        wire   pair_rdy  = aw_v && w_v;

        wire same_run  = og_v && (aw_a == og_next);
        wire full_strb = (w_s == {BPW{1'b1}});
        wire [AXI_AW-1:0] nxt_addr = aw_a + BPW;

        // 🔴 地址不连续时**先花一拍把旧组封口**, 这一拍不吃新数据。
        //    这样每拍最多推一个描述符, 描述符 FIFO 不需要双写口, 也没有
        //    "一拍推两个" 那种最容易算错边界的情况。
        wire need_close = pair_rdy && og_v && !same_run;
        wire do_commit  = pair_rdy && !need_close;
        wire do_flush   = og_v && (og_idle == IDLE_N) && !pair_rdy;

        // 吃下这一拍之后是否必须封口:
        //   1) 攒满 WPACK_N 拍
        //   2) strb 不满 = 整次解码的尾拍, 后面不会再有连续地址
        //   3) 下一拍就跨 4KB 页 —— 🔴 AXI 突发禁止跨 4KB
        wire [4:0] len_after   = og_v ? (og_len + 5'd1) : 5'd1;
        wire       close_after = (len_after >= WPACK_N[4:0]) || !full_strb ||
                                 (nxt_addr[11:0] == 12'd0);

        wire gpush = need_close || (do_commit && close_after) || do_flush;
        wire dpush = do_commit;

        always @(posedge clk) begin
            if (!rstn) begin
                dw_ptr <= 0; dr_ptr <= 0; dcount <= 0;
                gw_ptr <= 0; gr_ptr <= 0; gcount <= 0;
                og_v <= 1'b0; og_len <= 0; og_idle <= 0;
                aw_v <= 1'b0; w_v <= 1'b0; bcred <= 0;
            end else begin
                if (i_awvalid && i_awready) begin aw_a <= i_awaddr; aw_v <= 1'b1; end
                if (i_wvalid  && i_wready ) begin w_d <= i_wdata; w_s <= i_wstrb; w_v <= 1'b1; end

                // B 信用: 收下就回 (纯流控), 见本模块上方 "B 响应记账"
                if (do_commit && !(i_bvalid && i_bready))      bcred <= bcred + 1'b1;
                else if (i_bvalid && i_bready && !do_commit)   bcred <= bcred - 1'b1;

                if (pair_rdy) og_idle <= 8'd0;
                else if (og_v && og_idle != IDLE_N) og_idle <= og_idle + 1'b1;

                // ---- 数据 FIFO ----
                if (dpush) begin
                    pd[dw_ptr] <= w_d;  ps[dw_ptr] <= w_s;
                    dw_ptr <= dw_ptr + 1'b1;
                end
                if (snd_pop) dr_ptr <= dr_ptr + 1'b1;
                dcount <= dcount + (dpush ? 6'd1 : 6'd0) - (snd_pop ? 6'd1 : 6'd0);

                // ---- 描述符 FIFO ----
                if (gpush) begin
                    // need_close / do_flush 推的是**旧组**; do_commit 推的是含本拍的组
                    ga[gw_ptr] <= (do_commit && close_after) ?
                                  (og_v ? og_addr : aw_a) : og_addr;
                    gl[gw_ptr] <= (do_commit && close_after) ? len_after : og_len;
                    gw_ptr <= gw_ptr + 1'b1;
                end
                if (snd_take) gr_ptr <= gr_ptr + 1'b1;
                gcount <= gcount + (gpush ? 3'd1 : 3'd0) - (snd_take ? 3'd1 : 3'd0);

                // ---- 开着的那一组 ----
                if (need_close || do_flush) begin
                    og_v <= 1'b0; og_len <= 0; og_idle <= 0;
                end else if (do_commit) begin
                    aw_v <= 1'b0; w_v <= 1'b0;
                    if (close_after) begin
                        og_v <= 1'b0; og_len <= 0;
                    end else begin
                        og_v    <= 1'b1;
                        og_addr <= og_v ? og_addr : aw_a;
                        og_next <= nxt_addr;
                        og_len  <= len_after;
                    end
                end
            end
        end

        assign i_bvalid = (bcred != 0);

        always @(posedge clk) begin
            if (!rstn) begin
                snd_st <= S_IDLE; awv_r <= 1'b0; wv_r <= 1'b0;
                snd_left <= 0; snd_len_r <= 0; obcnt <= 0;
            end else begin
                if ((awv_r && m_axi_awready) && !(m_axi_bvalid && m_axi_bready))
                    obcnt <= obcnt + 1'b1;
                else if ((m_axi_bvalid && m_axi_bready) && !(awv_r && m_axi_awready))
                    obcnt <= obcnt - 1'b1;

                if (awv_r && m_axi_awready) awv_r <= 1'b0;

                case (snd_st)
                S_IDLE: if (snd_take) begin
                            snd_addr  <= ga[gr_ptr];
                            snd_left  <= gl[gr_ptr];
                            snd_len_r <= gl[gr_ptr];
                            awv_r     <= 1'b1;
                            wv_r      <= 1'b1;
                            snd_st    <= S_RUN;
                        end
                S_RUN:  if (snd_pop) begin
                            if (snd_left == 5'd1) begin
                                wv_r   <= 1'b0;
                                snd_st <= S_IDLE;
                            end else snd_left <= snd_left - 1'b1;
                        end
                endcase
            end
        end

        assign m_axi_awaddr  = snd_addr;
        assign m_axi_awlen   = {3'd0, (snd_len_r - 5'd1)};
        assign m_axi_awvalid = awv_r;
        assign m_axi_wdata   = pd[dr_ptr];
        assign m_axi_wstrb   = ps[dr_ptr];
        assign m_axi_wlast   = (snd_left == 5'd1);
        assign m_axi_wvalid  = wv_r;
        assign m_axi_bready  = 1'b1;

        // 🔴 "写真的落了 DDR" 的判据挪到这里: FIFO 空 + 没在攒 + 没在发 + B 全回
        assign pack_busy = (dcount != 0) || og_v || (gcount != 0) ||
                           (snd_st != S_IDLE) || (obcnt != 0) || aw_v || w_v;
    end
    endgenerate

    // ---------------- AXI4 master 缺失信号补齐 ----------------
    // 🔴 AXI_DW=64 ⇒ SIZE=3 (8B/beat)。补错这一条 = 静默数据错位。
    localparam [2:0] AXSIZE = (AXI_DW == 64) ? 3'b011 :
                              (AXI_DW == 32) ? 3'b010 : 3'b100;
    assign m_axi_awsize  = AXSIZE;
    assign m_axi_arsize  = AXSIZE;
    assign m_axi_awburst = 2'b01;      // INCR
    assign m_axi_arburst = 2'b01;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;    // 抄 ddr_slice_fetch256 (HP 口已验证)
    assign m_axi_arcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_arprot  = 3'b000;
    // m_axi_bresp / m_axi_rresp 不用: lz4_axi_top 没有错误上报路径。
    // 反正 SLVERR 只会出现在地址映射写错的情况, 那种错第一次上板就会全黑, 不会漏。

    lz4_axi_top #(
        .AXI_AW(AXI_AW),
        .AXI_DW(AXI_DW)
    ) u_lz4 (
        .clk(clk), .rstn(rstn),

        .s_awaddr (l_awaddr ), .s_awvalid(l_awvalid), .s_awready(),
        .s_wdata  (l_wdata  ), .s_wvalid (l_wvalid ), .s_wready (),
        .s_bvalid (l_bvalid ), .s_bready (1'b1),
        .s_araddr (l_araddr ), .s_arvalid(l_arvalid), .s_arready(),
        .s_rdata  (l_rdata  ), .s_rvalid (l_rvalid ), .s_rready (1'b1),

        .m_araddr (m_axi_araddr ), .m_arlen (m_axi_arlen ),
        .m_arvalid(m_axi_arvalid), .m_arready(m_axi_arready),
        .m_rdata  (m_axi_rdata  ), .m_rlast (m_axi_rlast ),
        .m_rvalid (m_axi_rvalid ), .m_rready(m_axi_rready),

        // 🔴 写通道不直连外口, 走 WPACK 打包器 (见上面 g_pack)
        .m_awaddr (i_awaddr ), .m_awlen (i_awlen ),
        .m_awvalid(i_awvalid), .m_awready(i_awready),
        .m_wdata  (i_wdata  ), .m_wstrb (i_wstrb ),
        .m_wlast  (i_wlast  ), .m_wvalid(i_wvalid),
        .m_wready (i_wready ),
        .m_bvalid (i_bvalid ), .m_bready(i_bready)
    );

endmodule
