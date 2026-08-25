//============================================================================
// lz4_decode_core.v — LZ4 raw block 解码器 (字节串行)
//
// 设计依据见 ../DESIGN.md。要点:
//   * 输入是 **LZ4 raw block**(pov_rxd.c 用 LZ4_decompress_safe 的那种):
//     无魔数/无块头/无 checksum, 原长由外部 raw_len 给。
//   * 64 KB 片上历史窗口 —— LZ4 的 offset 是 uint16 (≤65535), 所以片上窗口
//     覆盖**全部** match ⇒ 解码全程不回读 DDR, 输出是纯顺序流。
//   * **字节串行**: 每拍读历史 1 字节 / 写历史 1 字节。
//     ⇒ 重叠拷贝 (offset < match_len, 例如 offset=1 复制 100 次) **天然正确**,
//       语义与软件 `while(n--) *d++ = d[-off];` 逐字节等价, 不需要模式复制单元。
//       这是选字节串行最大的收益: 消掉了 LZ4 硬件实现里最易错的部分。
//   * 吞吐不够就**多例化几个**(载荷本来就是 PVS_FLAG_MSTREAM 多条独立流)。
//
// 接口: 纯流式, 不含 AXI。这样核可以在 iverilog 里全速仿真, AXI 包装单独一层。
//
// 🔴 历史 RAM 的读时序: ERAM 是**同步读**(读地址打一拍才出数据)。所以 match
//    拷贝做成 2 级流水: 第 N 拍发下一字节的读地址, 第 N+1 拍消费。
//    当 offset==1 时要读的正是**上一拍刚写进去的**字节, 同步 RAM 读不回来
//    (且 read-during-write 行为跨厂商不一致, 不能依赖) ⇒ 用 `last_wr_byte`
//    寄存器做旁路。offset==2 也要旁路(写与读同沿, read-before-write), 用
//    last_wr_byte2。offset>=3 才可以直接吃 RAM 输出。
//============================================================================

`timescale 1ns / 1ps

module lz4_decode_core #(
    parameter HIST_AW = 16                  // 64 KB, 覆盖 LZ4 offset 上限 65535
)(
    input  wire        clk,
    input  wire        rstn,

    // ---- 控制 ----
    input  wire        start,               // 单拍脉冲
    input  wire [31:0] raw_len,             // 期望输出字节数 (解到这么多就算完)
    output reg         done,
    output reg         error,               // 流损坏 (offset=0 / 输出超长)
    output reg  [2:0]  err_code,

    // ---- 压缩流入 (字节串行) ----
    input  wire        s_valid,
    input  wire [7:0]  s_data,
    output wire        s_ready,      // 🔴 组合, 见下面 s_ready_r 处的说明

    // ---- 解压流出 (字节串行) ----
    output wire        m_valid,     // 由输出 skid FIFO 驱动, 不再是 FSM 寄存器
    output wire [7:0]  m_data,
    input  wire        m_ready,
    output wire        m_last
);

    localparam HIST_SIZE = (1 << HIST_AW);

    localparam [3:0] S_IDLE    = 4'd0,
                     S_TOKEN   = 4'd1,
                     S_LITLEN  = 4'd2,
                     S_LITERAL = 4'd3,
                     S_OFF0    = 4'd4,
                     S_OFF1    = 4'd5,
                     S_MLEN    = 4'd6,
                     S_MPREP   = 4'd7,   // 发出 match 首字节的读地址, 等 RAM
                     S_MATCH   = 4'd8,
                     S_DONE    = 4'd9,
                     S_ERR     = 4'd10;

    localparam [2:0] E_NONE = 3'd0, E_OFF0 = 3'd1, E_OVERRUN = 3'd2, E_SRC = 3'd3;

    reg [3:0]  state;
    reg [31:0] lit_len, match_len, out_cnt;
    reg [15:0] offset;
    reg [7:0]  token;

    // ---- 历史窗口 (推断成 ERAM) ----
    reg [7:0] hist [0:HIST_SIZE-1];
    reg [HIST_AW-1:0] wr_ptr;               // 下一个要写的位置
    reg [HIST_AW-1:0] rd_addr;              // 读地址计数器 (**不直接接 RAM**)
    // 🔴 rd_addr_q: 专门给 ERAM 地址口的**流水寄存器**。TD 表征实测, 64KB 窗口
    //    被推断成 28 块 ERAM20K, 地址网扇出 28、跨半个 die, **单这一根线就 3.973ns**
    //    (整条关键路径 9.443ns 的 42%), 而且是所有通往历史 RAM 的路径的公共后缀。
    //    把它单独打一拍后: 组合逻辑→rd_addr 是一段, rd_addr_q→28 块 ERAM 是另一段,
    //    两段各自独立满足 10ns。这是唯一能真正搬动这个数的办法 —— 前面再怎么削
    //    逻辑级数都没用, 因为这根线在谁后面都一样长。
    reg [HIST_AW-1:0] rd_addr_q;
    reg [7:0] rd_data;                      // 同步读数据 (打一拍)
    reg [7:0] last_wr_byte;                 // 上一个写入的字节 (offset==1 旁路)
    reg [7:0] last_wr_byte2;                // 再上一个     (offset==2 旁路)

    reg       hist_we;
    reg [7:0] hist_wd;
    reg [HIST_AW-1:0] hist_wa;              // 🔴 必须单独存写地址, 见下

    // 输出 skid FIFO 的寄存器 (逻辑在下面"输出 skid FIFO"一节;
    // 声明提到这里是因为 hist_re 要用 ob_room)
    localparam OB_AW = 3, OB_N = 8;
    reg [8:0]       obuf [0:OB_N-1];        // {last, data}
    reg [OB_AW-1:0] ob_wp, ob_rp;
    reg [OB_AW:0]   ob_cnt;                 // 0..OB_N
    reg             ob_room;                // 寄存器: 还能再收 (门限见下)

    // 🔴 hist_we/hist_wd 是**寄存器**, 打一拍才生效; 而 wr_ptr 在同一拍就自增了。
    //    直接写 `hist[wr_ptr]` 会把字节写进 W+1 而不是 W —— 全表错位一格。
    //    (症状: offset==1 因走旁路不读 RAM 而"碰巧对", offset>=2 全错。)
    //    所以把写地址和数据一起打拍, 保证地址与数据同源同龄。
    //
    // 🔴 读**必须带使能**, 不能每拍无条件读。rd_addr 恒领先一拍, 所以 rd_data 里
    //    装的是"下一个要用的字节"。如果下游反压(m_ready=0)时还继续读, rd_addr 已经
    //    指向再下一个字节 ⇒ 这个预取值被覆盖 ⇒ **match 的首字节被第二字节顶掉**。
    //    读使能与 rd_addr 的推进条件严格同步: 只有推进了才重新取。
    //    ⚠ 这个 bug 只在 S_MPREP→S_MATCH 的第一拍就被反压时出现。核的独立 testbench
    //      里 m_ready 恒为 1, 结构上**不可能**测出来, 是接上 AXI 包装并给写通道加
    //      随机反压后才暴露的 —— 恒 ready 的 testbench 是这类 bug 的盲区。
    wire hist_re = (state == S_MPREP) || ((state == S_MATCH) && ob_room);

    // 🔴 RAM 的地址口吃的是 rd_addr_q(流水一级), 不是 rd_addr。
    //    延迟没有变: rd_addr_q 在 S_OFF1/S_MLEN 里被**直接预装**成 match 首字节
    //    地址(同时 rd_addr 装成 +1), 所以 S_MPREP 仍然只要 1 拍, B/clk 不变。
    always @(posedge clk) begin
        if (hist_we) hist[hist_wa] <= hist_wd;
        if (hist_re) rd_data <= hist[rd_addr_q];
    end

    // 🔴 match 源字节需要**两级**旁路, 不是一级:
    //   offset==1: 要的是上一拍才写的字节, 那时 RAM 读早已发出 → last_wr_byte
    //   offset==2: 要的字节的写入与本次 RAM 读**落在同一个时钟沿**, 非阻塞语义下
    //              读到的是旧值(read-before-write) → last_wr_byte2
    //   offset>=3: 写已落地至少一拍, RAM 读正常
    // (实测确实只有 off2 单独挂掉, off1 与 off4/off7 都对 —— 正好卡在这个边界。)
    wire [7:0] match_byte = (offset == 16'd1) ? last_wr_byte  :
                            (offset == 16'd2) ? last_wr_byte2 : rd_data;

    // match 源地址每拍 +1 即可: 写指针也每拍 +1, 二者差值恒为 offset。
    // 🔴 读地址必须**领先一拍** —— ERAM 同步读, 本拍发地址下拍才出数据。
    //    (曾写成 `wr_ptr + 1 - offset`, 但那时 wr_ptr 尚未自增, 算出来正是
    //     当前字节的地址 ⇒ 同一字节被读两次、且首字节读在写入之前 ⇒ 全 x。
    //     offset==1 因为走旁路不看 rd_data, 反而是唯一"碰巧对"的用例。)

    // ======================================================================
    // 输出 skid FIFO —— 把外部 m_ready 彻底挡在核的数据路径之外
    // ======================================================================
    // 🔴 起因 (TD 表征实测, 不是推测): 修好反压 bug 之后关键路径变成
    //      m_ready_q → (组合 s_ready) → 状态/使能 → rd_addr → ERAM.addrb
    //    7 级逻辑, 9.447ns 里 8.028ns(85%) 是走线, 最后一跳 rd_addr 扇出 28 块
    //    ERAM 就吃掉 3.085ns ⇒ SWNS +0.043ns, Fmax 100.4MHz, 余量 0.4%。
    //
    // 🔴 关键认识: **光加 FIFO 是不够的**。把 m_ready 换成一个本地寄存器并不会
    //    缩短逻辑级数 —— 静态时序不知道"FIFO 没满时 ready 恒 1"这件事。
    //    真正省掉前面 4 级的是: 有了 FIFO 保证有位置, s_ready 就可以**变回纯
    //    寄存器**。原来那个组合 s_ready 会把 m_ready 的依赖散播到**所有**用到
    //    s_ready 的地方 —— 包括 S_OFF1/S_MLEN 里对 rd_addr 的加载。哪怕
    //    `state != S_LITERAL` 时它逻辑上无关, 综合器也只建一张 s_ready 网。
    //    ⇒ FIFO 的作用是"让纯寄存器 s_ready 重新变成正确的", 时序收益来自后者。
    //
    // 深度 8 的理由:
    //   * 下限是 3。s_ready_r 是寄存器, 而它是拿**上一拍**的 ob_room 算的,
    //     ob_room 本身又是寄存器 ⇒ literal 通路的 ready 有 **2 拍陈旧度**。
    //     取 room 门限 = N-3 可证明永不溢出:
    //       - match 通路(1 拍陈旧): 推入要求 ob_room_t=(cnt_{t-1}<=N-3),
    //         而 cnt_t <= cnt_{t-1}+1 <= N-2 ⇒ 推完 <= N-1。
    //       - literal 通路(2 拍陈旧): 要求 cnt_{t-2}<=N-3,
    //         cnt_t <= cnt_{t-2}+2 <= N-1 ⇒ 推完 <= N。刚好不溢。
    //   * 取 8 是为了吸收 AXI 单拍写事务的往返(反压档实测 2-6 拍), 让核在写
    //     通道打嗝时继续解码; 门限 N-3=5 ⇒ 可连续吸收 6 拍停顿才反压核。
    //   * 代价 8×9=72 个 FF, 不占 ERAM。
    // (ob_* 的声明在上面历史窗口那一节, 因为 hist_re 要用 ob_room。)
    wire ob_pop = m_valid && m_ready;
    assign m_valid = (ob_cnt != {(OB_AW+1){1'b0}});
    assign m_data  = obuf[ob_rp][7:0];
    assign m_last  = obuf[ob_rp][8];

    // 这一拍 FSM 会不会吐一个字节 —— FSM 与 FIFO 共用同一个判据
    wire lit_take = (state == S_LITERAL) && s_valid && s_ready;
    wire mat_take = (state == S_MATCH)   && ob_room;
    wire ob_push  = lit_take || mat_take;
    wire [7:0] ob_pd = (state == S_LITERAL) ? s_data : match_byte;
    wire ob_pl = (state == S_LITERAL)
                 ? ((lit_len   == 32'd1) && ((out_cnt + 32'd1) >= raw_len))
                 : ((match_len == 32'd1) && ((out_cnt + 32'd1) == raw_len));

    always @(posedge clk) begin
        if (!rstn) begin
            ob_wp <= 0; ob_rp <= 0; ob_cnt <= 0; ob_room <= 1'b1;
        end else if (start || error) begin
            ob_wp <= 0; ob_rp <= 0; ob_cnt <= 0; ob_room <= 1'b1;
        end else begin
            if (ob_push) begin obuf[ob_wp] <= {ob_pl, ob_pd}; ob_wp <= ob_wp + 1'b1; end
            if (ob_pop) ob_rp <= ob_rp + 1'b1;
            if (ob_push && !ob_pop)      ob_cnt <= ob_cnt + 1'b1;
            else if (ob_pop && !ob_push) ob_cnt <= ob_cnt - 1'b1;
            ob_room <= (ob_cnt <= (OB_N-3));
        end
    end

    // 🔴 s_ready 回到**纯寄存器**。它仍然如实表示"这一拍我真的会收下这个字节":
    //    进入/停留在 S_LITERAL 时 s_ready_r 一律用 ob_room 赋值, 而 ob_room 的
    //    门限保证了"承诺过就一定收得下"。这条是上面那个卡死 bug 的正解 ——
    //    早先那版组合门虽然也对, 但把 m_ready 拖进了 ERAM 地址路径。
    reg  s_ready_r;
    assign s_ready = s_ready_r;

    integer i;
    always @(posedge clk) begin
        if (!rstn) begin
            state    <= S_IDLE;
            s_ready_r <= 1'b0;
            done     <= 1'b0;
            error    <= 1'b0;
            err_code <= E_NONE;
            wr_ptr   <= {HIST_AW{1'b0}};
            out_cnt  <= 32'd0;
            hist_we  <= 1'b0;
        end else begin
            hist_we <= 1'b0;

            case (state)
            // ------------------------------------------------------------
            S_IDLE: begin
                done <= 1'b0; error <= 1'b0; err_code <= E_NONE;
                if (start) begin
                    out_cnt <= 32'd0;
                    wr_ptr  <= {HIST_AW{1'b0}};
                    s_ready_r <= 1'b1;
                    state   <= S_TOKEN;
                end
            end

            // ---- token: 高 4 位 literal 长度, 低 4 位 match 长度-4 --------
            S_TOKEN: if (s_valid && s_ready) begin
                token     <= s_data;
                lit_len   <= {28'd0, s_data[7:4]};
                match_len <= {28'd0, s_data[3:0]} + 32'd4;
                if (s_data[7:4] == 4'hF) begin
                    state <= S_LITLEN;                 // literal 长度要跟扩展字节
                end else if (s_data[7:4] == 4'h0) begin
                    s_ready_r <= 1'b1; state <= S_OFF0;  // 没有 literal, 直奔 offset
                end else begin
                    // 🔴 进 S_LITERAL 的每条边都必须用 ob_room 赋 s_ready_r,
                    //    否则"承诺收下"就失去了 FIFO 有位置的保证 (会溢出丢字节)。
                    s_ready_r <= ob_room; state <= S_LITERAL;
                end
            end

            // ---- 长度扩展: 连加 255, 遇到非 255 结束 ----------------------
            S_LITLEN: if (s_valid && s_ready) begin
                lit_len <= lit_len + {24'd0, s_data};
                if (s_data != 8'hFF) begin
                    s_ready_r <= ob_room; state <= S_LITERAL;
                end
            end

            // ---- literal: 输入字节直接透传到输出, 同时写历史 --------------
            // 接收条件就是 lit_take = s_valid && s_ready, 不挂任何额外条件
            // (挂额外条件正是原来丢字节/卡死的写法)。输出去向是 skid FIFO,
            // 由 s_ready_r <= ob_room 保证一定收得下。
            S_LITERAL: begin
                if (lit_take) begin
                    hist_wd <= s_data;      hist_wa <= wr_ptr; hist_we <= 1'b1;
                    last_wr_byte <= s_data;     last_wr_byte2 <= last_wr_byte;
                    wr_ptr  <= wr_ptr + 1'b1;
                    out_cnt <= out_cnt + 1'b1;
                    lit_len <= lit_len - 1'b1;

                    if (lit_len == 32'd1) begin
                        // literal 段结束。若已凑满 raw_len, 说明这是收尾
                        // sequence (最后一段只有 literals, 没有 offset/match)。
                        if (out_cnt + 1 >= raw_len) begin
                            s_ready_r <= 1'b0; state <= S_DONE;
                        end else begin
                            s_ready_r <= 1'b1;   // S_OFF0 只吃不吐, 不用看 ob_room
                            state <= S_OFF0;
                        end
                    end else begin
                        s_ready_r <= ob_room;
                    end
                end else begin
                    s_ready_r <= ob_room;
                end
            end

            // ---- offset: 2 字节小端, 0 非法 -------------------------------
            S_OFF0: if (s_valid && s_ready) begin
                offset[7:0] <= s_data;
                state <= S_OFF1;
            end
            S_OFF1: if (s_valid && s_ready) begin
                offset[15:8] <= s_data;
                if ({s_data, offset[7:0]} == 16'd0) begin
                    err_code <= E_OFF0; state <= S_ERR;
                end else if (token[3:0] == 4'hF) begin
                    state <= S_MLEN;
                end else begin
                    s_ready_r <= 1'b0;
                    // 预装两级: rd_addr_q = 首字节地址(这拍就送进 RAM 地址口),
                    //           rd_addr   = 首字节+1
                    rd_addr_q <= wr_ptr - {s_data, offset[7:0]};
                    rd_addr   <= wr_ptr - {s_data, offset[7:0]} + 1'b1;
                    state     <= S_MPREP;
                end
            end

            S_MLEN: if (s_valid && s_ready) begin
                match_len <= match_len + {24'd0, s_data};
                if (s_data != 8'hFF) begin
                    s_ready_r <= 1'b0;
                    rd_addr_q <= wr_ptr - offset;
                    rd_addr   <= wr_ptr - offset + 1'b1;
                    state     <= S_MPREP;
                end
            end

            // ---- 等同步 RAM 把首字节读出来, 同时把读地址推到第 2 个字节 ----
            S_MPREP: begin
                rd_addr_q <= rd_addr;
                rd_addr   <= rd_addr + 1'b1;
                state     <= S_MATCH;
            end

            // ---- match: 每拍搬 1 字节, 重叠拷贝天然正确 -------------------
            // 🔴 使能是 mat_take = (state==S_MATCH) && ob_room, ob_room 是**本地
            //    寄存器** ⇒ 从这里到 rd_addr/ERAM 只剩 2 级左右的逻辑。
            //    这正是把 Fmax 抢回来的地方。
            S_MATCH: begin
                if (mat_take) begin
                    hist_wd <= match_byte;  hist_wa <= wr_ptr; hist_we <= 1'b1;
                    last_wr_byte <= match_byte;  last_wr_byte2 <= last_wr_byte;
                    wr_ptr  <= wr_ptr + 1'b1;
                    out_cnt <= out_cnt + 1'b1;
                    match_len <= match_len - 1'b1;
                    rd_addr_q <= rd_addr;             // 地址流水级, 与数据同步推进
                    rd_addr   <= rd_addr + 1'b1;      // 预取下一字节

                    if (out_cnt + 1 > raw_len) begin
                        err_code <= E_OVERRUN; state <= S_ERR;
                    end else if (match_len == 32'd1) begin
                        if (out_cnt + 1 == raw_len) begin
                            state <= S_DONE;
                        end else begin
                            s_ready_r <= 1'b1; state <= S_TOKEN;
                        end
                    end
                end
            end

            // ------------------------------------------------------------
            // done 要等 FIFO 也排空 —— 它的含义是"所有输出字节都已交付"
            S_DONE: if (ob_cnt == {(OB_AW+1){1'b0}}) begin
                done <= 1'b1; s_ready_r <= 1'b0; state <= S_IDLE;
            end

            S_ERR: begin
                error <= 1'b1; s_ready_r <= 1'b0; state <= S_IDLE;
            end

            default: state <= S_ERR;
            endcase
        end
    end

endmodule
