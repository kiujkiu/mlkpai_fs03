/*
 * protocol.h — POV volumetric display streaming protocol (PVS1).
 * Shared between PC sender (stream/pc/povstream.py) and board C receiver.
 * Authoritative spec: stream/pc/protocol.md. Keep both in sync.
 *
 * Transport: TCP, PC connects to board, default port 9500.
 * Stream = sequence of frames, each: 16-byte header | payload.
 * Receiver replies 1 ACK byte after each fully processed frame.
 * Sender closes the connection to end the stream.
 *
 * All multi-byte fields little-endian (both sides are LE, send struct as-is).
 */
#ifndef POV_STREAM_PROTOCOL_H
#define POV_STREAM_PROTOCOL_H

#include <stdint.h>

#define PVS_MAGIC        "PVS1"            /* 4 bytes, no NUL */
#define PVS_MAGIC_U32    0x31535650u       /* 'P','V','S','1' read as LE u32 */
#define PVS_PORT         9500

/* Frame geometry (FS03 160x180 panel, 360 angular slices).
 * Slice = 11664 data bytes padded to 0x3000; layout is the pack_obs.py
 * DDR mirror contract (lane*1296 + row*24 + word*4). Hardware-verified,
 * do not change. */
#define PVS_N_SLICES     360               /* 传统单面帧的片数 = 默认值 */
#define PVS_SLICE_DATA   11664
#define PVS_SLICE_STRIDE 0x3000            /* 12288 */
#define PVS_FRAME_RAW    (PVS_N_SLICES * PVS_SLICE_STRIDE)  /* 4423680 */

/* ---- 2026-08-20 v3.4 3-bit 色深 (docs/design_icnd2047/05_3bit_bcm.md) -----
 * 每通道 1 bit -> 3 bit (每像素 9 bit = 512 色), 行内 BCM, 权重 27/54/108 沿。
 * 一片数据 = **3 个位平面顺序排列**, plane p 落在 slice_base + p*0x3000,
 * 每个 plane 内部布局与今天的 1-bit **逐字节相同** (lane*1296 + row*24 + word*4)。
 *   plane 0 = 最低位 (权重 1, oe_w0 = 现有 oe_window)
 *   plane 1 = 中位   (权重 2, oe_w1)
 *   plane 2 = 最高位 (权重 4, oe_w2)
 * 于是片距从 0x3000 变成 0x9000, **其它一切都不变** —— 面拆分、MSTREAM 流表、
 * DELTA、压缩位全部与 1-bit 一模一样, 只是「一片有多大」换了个数。
 *
 * 🔴 片距不再是编译期常量: 收发双方都必须**从 flags 推**, 不能再直接用
 *    PVS_SLICE_STRIDE。用 PVS_STRIDE(flags)。 */
#define PVS_SLICE_STRIDE_3BIT 0x9000       /* 36864 = 3 * 0x3000 */
#define PVS_STRIDE(flags)  (((flags) & PVS_FLAG_3BIT) ? \
                            (uint32_t)PVS_SLICE_STRIDE_3BIT : \
                            (uint32_t)PVS_SLICE_STRIDE)

/* ---- 2026-07-31 v3.1 偏心屏: 帧长度不再是编译期常量 ----------------------
 * 机械 v3.1 把屏模组整体偏 6.7mm, 两 LED 面落在 X=0 / X=+13.4 → 两面不再关于
 * 转轴对称, 「屏B@θ ≡ 屏A@(θ+180)」作废 ⇒ 单份 360 片喂两面的前提没了。
 * 于是帧里可能装 1 面或 2 面, 面A 还可能折叠成 180 片 (穿心面 slice_i 与
 * slice_{i+180} 互为镜像)。
 *
 * **头里的 n_slices 是权威**, 接收方按它算长度, 不再硬校验 == PVS_N_SLICES:
 *     raw_len MUST == n_slices * PVS_STRIDE(flags) (<= PVS_FRAME_RAW_MAX)
 *     (v3.4: 片距随 PVS_FLAG_3BIT 变 0x3000/0x9000, 见下)
 * 老帧 (n_slices=360, 无新 flag) 语义逐字节不变, 老 frames_* 继续可用。
 *
 * 载荷排布 (解压后):
 *   单面:            [面 0 .. n_slices-1]
 *   PVS_FLAG_DUAL_FACE: [面A 0..nA-1][面B 0..nB-1], nA+nB = n_slices,
 *                       nA = PVS_FLAG_FOLD_A ? nB/2 : nB
 *                       ⇒ nA = FOLD_A ? n_slices/3 : n_slices/2
 *                       (360 槽: 720 -> 360+360, fold 540 -> 180+360, 与老帧同)
 *   PVS_FLAG_FOLD_A:  面A 只送前半圈; 后半圈由 PL 取 idx-n_eng/2 再做镜像置换
 *                     (n_eng = 引擎每圈片数 0x10[31:16]; 360 槽时就是老的 180)
 *                     (仅穿心面 axis_off==0 成立, 发送端必须自检过)
 */
#define PVS_N_SLICES_MAX 720
/* 🔴 2026-08-24 半屏扫描 (RTL half_scan) 把整屏拍数砍半 ⇒ 每圈画得出 283 槽,
 * 而旧上限 720*0x3000 = 8847360 只够 3-bit 240 片。抬到 0xA00000 (10.49 MB):
 *   3-bit: 0xA00000 / 0x9000 = 291 片  (283 槽单面装得下)
 *   1-bit: 0xA00000 / 0x3000 = 853 片  (实际仍只用 720, 几何约定没变)
 * 连带 pov_rxd 的 BANK_BYTES/FRAME_MAP_LEN 自动跟着涨 (它们由本常量推),
 * ⚠ 但 pov_boot.sh 的 povmem size 是**手写常量**, 必须同步改到 >= 0x29F3000,
 *   否则 mmap 覆盖不到 bank C 的尾部 —— 那是静默的越界写。 */
#define PVS_FRAME_RAW_MAX 0x1500000u                             /* 22020096 (21 MB) */
/* 🔴 2026-08-24 第二次抬: 半屏每圈画得出 283 槽, 双面 282 槽 = 564 片 x 0x9000
 * = 20.8 MB。连带板端 BANK_STRIDE 必须从 16MB 抬到 32MB (否则 bank 会踩到下一个
 * bank 头上), FRAME_MAP_LEN -> 87.9MB, pov_boot.sh 的 povmem size 同步。
 *   3-bit: 0x1500000 / 0x9000 = 611 片   1-bit: / 0x3000 = 1834 片 */
#define PVS_N_SLICES_FOLD 180              /* 折叠后的面A 片数 (=360 槽时的值) */
/* 🔴 帧**字节数**上限是硬的 (= 板端一个 DDR bank / staging 缓冲的大小),
 * 片数上限随片距变: 3-bit 一片大 3 倍, 所以片数上限小 3 倍。
 *   1-bit: 720 片 * 0x3000 = 8847360 B  (= PVS_FRAME_RAW_MAX)
 *   3-bit: 240 片 * 0x9000 = 8847360 B  (整除, 一个字节都不浪费)
 * 240 片对 3-bit 是**两倍余量**: 方案定的是每面 60 槽 (双面 120 片 = 4.42 MB),
 * 见 05_3bit_bcm.md §4 —— 3-bit 反而比今天在跑的 1-bit 720 片省一半载荷。
 * 要支持 3-bit 720 片 (26.5 MB/帧) 就必须同时加大板端 bank 间距/povmem 窗口,
 * 算式见 pov_rxd.c 文件头的帧区地址表。 */
#define PVS_N_SLICES_MAX_3BIT (PVS_FRAME_RAW_MAX / PVS_SLICE_STRIDE_3BIT)  /* 291 */
#define PVS_N_SLICES_MAX_F(flags) (((flags) & PVS_FLAG_3BIT) ? \
                                   (uint32_t)PVS_N_SLICES_MAX_3BIT : \
                                   (uint32_t)PVS_N_SLICES_MAX)

/* flags bits */
#define PVS_FLAG_RLE     (1u << 0)  /* payload = zero-run RLE (see protocol.md) */
#define PVS_FLAG_ZLIB    (1u << 1)  /* payload = zlib stream (RFC1950). CHOSEN default. */
#define PVS_FLAG_DELTA   (1u << 2)  /* payload (after RLE/zlib decode) = XOR delta
                                     * against the PREVIOUS successfully-ACKed raw
                                     * frame: raw = prev_raw ^ decoded. First frame
                                     * of a connection MUST NOT set DELTA (keyframe);
                                     * receiver NAKs a DELTA frame with no prior
                                     * frame. Sender policy: keyframe every N frames
                                     * (default 26) and on (re)connect. Composes
                                     * with ZLIB: DELTA|ZLIB = zlib(prev^cur). */
#define PVS_FLAG_DUAL_FACE (1u << 3)  /* 载荷含两面。v3.1 偏心屏两面几何不同
                                       * (垂距 0 / 13.4mm), 必须各渲一份; 板端分别
                                       * 写两个 DDR 基址, PL 侧 slice_base(0x18) /
                                       * slice_base_b(0x28)。
                                       *
                                       * 载荷排布 = **两条独立压缩流**, 不是一条:
                                       *   [u32 LE comp_len_A][面A 压缩流][面B 压缩流]
                                       *   comp_len_A = 面A 流字节数
                                       *   面B 流长度  = comp_len - 4 - comp_len_A
                                       * 之所以拆两条 (实测代价仅 33.7x→33.6x, 0.3%):
                                       * **两面可以并行解压到两个 CPU 核**, 单帧解码
                                       * 时间直接减半, 且不需要奇偶帧交错那套额外缓冲
                                       * 与一帧延迟。DELTA 组合时各面各自 XOR 自己上一
                                       * 帧的同面数据 (参考帧也按面分开存)。*/
#define PVS_FLAG_FOLD_A    (1u << 4)  /* 面A 折叠: 只送前半圈 (θ=0..179°)。
                                       * 仅当面A 穿心 (垂距 0) 时合法 —— 此时
                                       * slice_i ≡ mirror(slice_{i+h}) 严格成立
                                       * (h = 半圈片数), PL 用 idx>=h → 取 idx-h
                                       * + 镜像置换补齐。发送端必须先跑打包域自检
                                       * 再置位。
                                       * ⚠ 片数**不是**写死的 180: h = 引擎每圈
                                       * 片数/2, 360 槽下才等于 180。3-bit 走 60
                                       * 槽时折叠面就是 30 片。 */
#define PVS_FLAG_LZ4       (1u << 5)  /* 载荷 = LZ4 **raw block**。v3.3 起的首选
                                       * 压缩位, 用来顶掉 ZLIB。
                                       *
                                       * 为什么换: A9 上用真内容 anime_dual720.bin
                                       * (720 片偏心双面, 8847360 B) 实测 ——
                                       *   zlib-6  376780 B 23.5x  解压 163.5 ms  51.6 MB/s
                                       *   lz4-HC9 388166 B 22.8x  解压  41.2 ms 204.6 MB/s
                                       * 压缩比只差 3%, 单核解压快 4 倍。DUAL_FACE
                                       * 两条流双核并行 ⇒ 单帧 ~20.6 ms ⇒ 48 fps
                                       * (zlib 只到 12 fps)。链路侧多出的 11 KB/帧
                                       * 在 2.4G WiFi 上可以忽略。
                                       *
                                       * 🔴 必须是 **raw block**, 不是 CLI 的 .lz4
                                       * 帧格式。两者不是一个东西:
                                       *   raw block   = LZ4_compress_HC() 的直接输出,
                                       *                 纯 token 流, 无头无尾无校验;
                                       *                 只能用 LZ4_decompress_safe()
                                       *                 解, 且**必须由调用方给出
                                       *                 dstCapacity** (流里没有原长)。
                                       *   .lz4 帧格式 = `lz4` 命令行 / lz4frame.h /
                                       *                 python-lz4.frame 的输出, 带
                                       *                 魔数 0x184D2204 + 帧描述符 +
                                       *                 分块头 + 可选 xxhash 校验。
                                       *                 LZ4_decompress_safe() **吃不了**
                                       *                 它 —— 会把魔数当 token 解, 返回
                                       *                 负数或一堆垃圾。
                                       * 所以: 发送端只能调 LZ4_compress_HC/
                                       * LZ4_compress_default (liblz4 的 lz4.h/lz4hc.h),
                                       * 不能 `lz4 -12 f.bin` 也不能 lz4.frame.compress。
                                       *
                                       * HC 级别: 默认 **12**。同一份 anime_dual720.bin
                                       * 整帧单流 (x86 liblz4 1.10.0) 实测:
                                       *   HC9  388166 B 22.79x  enc 135 ms
                                       *   HC10 413178 B 21.41x  enc 138 ms  ← **比 HC9 差 6.4%**
                                       *   HC11 381889 B 23.17x  enc 497 ms
                                       *   HC12 370699 B 23.87x  enc 977 ms
                                       * HC10 比 HC9 还大是可复现的 (不是测量噪声), **跳过 10**。
                                       * HC12 比 HC9 小 4.5%, 甚至比 zlib-6 (377009 B) 还小
                                       * —— 于是 LZ4 在这份内容上压缩比和解压速度**双赢**。
                                       * 代价全在 PC 编码侧 (~1 s/帧) ⇒ **必须离线预压缩**
                                       * (povstream 的 --dir 预压缩缓存), 现渲直推别想。
                                       * 解压速度与级别无关 (raw block 格式一样)。
                                       * dstCapacity 从哪来: 单面 = hdr.raw_len;
                                       * DUAL_FACE = 各面自己的 nX*0x3000 (下面的排布)。
                                       *
                                       * 互斥性: LZ4 与 ZLIB **互斥**, 同时置位是非法帧,
                                       * 接收方 NAK (与 RLE 也互斥, 同理)。一帧只有一个
                                       * 压缩位。
                                       *
                                       * 载荷排布与 ZLIB 完全一致, 一个字节都不差:
                                       *   单面:       [LZ4 raw block]
                                       *   DUAL_FACE:  [u32 LE comp_len_A][A 流][B 流]
                                       *               —— 仍是**两条独立的 raw block**,
                                       *               各自可单独 LZ4_decompress_safe,
                                       *               所以照样能摊到两个核上。
                                       * 与 DELTA 组合同 ZLIB: DELTA|LZ4 = lz4(prev^cur)。*/
#define PVS_FLAG_MSTREAM   (1u << 6)  /* 载荷 = **可变条数**的独立压缩流 + 一张流表。
                                       * 取代 DUAL_FACE 那个写死两条的 4B 前缀。
                                       *
                                       * 为什么要它: 双核并行解码的 makespan 由**最
                                       * 慢那条流**决定, 而「按面切」切出来的两条流
                                       * 工作量根本不等。fold540 (面A 折 180 片 +
                                       * 面B 360 片) 实测:
                                       *   按面切 A180/B360 : makespan 20.6 ms (被面B
                                       *                      独占的 360 片封顶)
                                       *   朴素三分 180/180/180: 还是 20.6 ms —— 3 条
                                       *                      流放 2 个核 = 2+1, 零收益
                                       *   均衡三分 180/90/270 : **15.47 ms** (两核各
                                       *                      270 片, 完美平衡)
                                       * ⇒ 切点必须落在面B 的第 90 片, 代价 +460 B
                                       *   (0.17%)。**FOLD_A 只省链路 (−31%), 一分钱
                                       *   解码时间都不省** —— 按面切时 makespan 由面B
                                       *   封顶, 折不折叠都一样。这条曾经被说反过。
                                       *
                                       * 载荷 (小端, 紧凑排列, 无对齐填充):
                                       *   [u32 n_streams]
                                       *   [n_streams × { u32 comp_len_i, u32 n_slices_i }]
                                       *   [流 0][流 1] … [流 n_streams-1]
                                       * 接收方**必须**校验这两条 (不校验就会静默错位):
                                       *   comp_len == 4 + 8*n_streams + Σ comp_len_i
                                       *   Σ n_slices_i == hdr.n_slices
                                       * 流 i 的解压输出落在
                                       *   buf + (Σ_{j<i} n_slices_j) * PVS_SLICE_STRIDE,
                                       * 长度 n_slices_i * PVS_SLICE_STRIDE。
                                       * n_streams ∈ [1, PVS_MAX_STREAMS]。
                                       *
                                       * 为什么表里要带 n_slices_i (只给 comp_len_i 不
                                       * 够): 压缩流里读不出原长, 而 LZ4_decompress_safe
                                       * 必须先知道 dstCapacity 和落点。若只给压缩长度,
                                       * 流 i 的落点就得等流 i-1 解完才知道 = 退化成串行,
                                       * 并行的意义直接没了。
                                       * 为什么表里两个字段都全给 (末条不省那 4 字节):
                                       * 388 KB 的载荷里省 4 字节 = 0.001%, 换来的是两个
                                       * 独立的求和自校验 —— 载荷截断/错位会当场 NAK,
                                       * 而不是解出半帧垃圾还照样 ACK。
                                       *
                                       * 🔴 边界必须落在**片边界**上 (长度都是
                                       * PVS_SLICE_STRIDE 的整数倍) —— 板端是按片给两个
                                       * 核派活的。流之间不能有任何跨流引用 (各压各的)。
                                       *
                                       * 与 DUAL_FACE 的关系 = **正交**:
                                       *   DUAL_FACE 说的是「显示怎么分」(面A/面B 两个
                                       *     DDR 基址, 分界 = nA*0x3000, 与流表无关);
                                       *   MSTREAM  说的是「解码怎么分」(几条流)。
                                       *   MSTREAM 置位时**没有** DUAL_FACE 那个 4B
                                       *   comp_len_A 前缀 —— 流表把它取代了。
                                       * MSTREAM 也可以用在单面帧上 (360 片切两条流 =
                                       * 单面也能吃满双核)。
                                       *
                                       * 向后兼容 (两个方向都要成立):
                                       *   老固件 + 新发送端: 只要不置 MSTREAM 就还是老
                                       *     的 [u32 comp_len_A][A][B], 逐字节不变; 且
                                       *     「切分结果恰好等于按面切」时发送端**必须**
                                       *     退回老格式 (720 片双面就是这种情况)。真置了
                                       *     MSTREAM, 老固件的未知 flag 掩码会直接 NAK
                                       *     —— 响亮地失败, 不会静默解错。
                                       *   新固件 + 老发送端: 没有 MSTREAM 位就走老的
                                       *     两流分支, 逐字节兼容。 */
#define PVS_FLAG_3BIT      (1u << 7)  /* 每通道 3 bit (行内 BCM), 一片 = 3 个位平面
                                       * 顺序排列, plane p @ slice_base + p*0x3000,
                                       * 片距 0x3000 -> 0x9000。详见上面
                                       * PVS_SLICE_STRIDE_3BIT 那段与
                                       * docs/design_icnd2047/05_3bit_bcm.md。
                                       *
                                       * 🔴 **不是压缩位**, 不进 PVS_FLAGS_CODEC。
                                       * 板端 face_job_t.codec 存的是「压缩 flag 位
                                       * 原样」(见 pov_rxd.c 那条注释: 曾经按序号填
                                       * codec=1 正好命中 PVS_FLAG_RLE, 拿 zlib 流
                                       * 跑 RLE 解码, 静默失败)。本位落在 bit7 =
                                       * 压缩位集合 {bit0,bit1,bit5} 之外, 也在
                                       * 几何位 {bit3,bit4} 之外, 谁都不撞。
                                       *
                                       * 与所有现有 flag **正交**:
                                       *   DUAL_FACE / FOLD_A: 面怎么拆不变, 只是
                                       *     nA*stride 里的 stride 换成 0x9000;
                                       *   MSTREAM: 流表里的 n_slices_i 仍是**片数**,
                                       *     每条流的解压长度 = n_slices_i * stride;
                                       *   DELTA: 逐字节 XOR, 与片距无关 (但参考帧
                                       *     必须同色深 —— 长度/面边界校验已覆盖:
                                       *     色深一变 raw_len 必变);
                                       *   RLE/ZLIB/LZ4: 压的是字节流, 完全不关心。
                                       *
                                       * 🔴 硬件侧: 板端收到 3-bit 帧要把 PL 的
                                       * bpp_mode 切到 1 (0x0C subcmd=01 [16]), 收到
                                       * 1-bit 帧切回 0。两种内容可以逐帧交替, 因为
                                       * 空闲动画和上电默认内容都还是 1-bit。 */
#define PVS_MAX_STREAMS  16           /* 流表条数上限 (表最大 4+16*8 = 132 B) */
/* flags == 0 -> payload is raw (comp_len == raw_len) */

/* per-frame ACK byte, sent by receiver after decompress+verify(+commit) */
#define PVS_ACK          0x06       /* frame OK, send next */
#define PVS_NAK          0x15       /* error; sender aborts stream */

#pragma pack(push, 1)
typedef struct {
    char     magic[4];    /* "PVS1" */
    uint32_t comp_len;    /* payload length in bytes as transmitted */
    uint32_t raw_len;     /* decompressed length; MUST == n_slices * PVS_STRIDE(flags)
                           * 且 <= PVS_FRAME_RAW_MAX (老帧里就是 PVS_FRAME_RAW)。
                           * 🔴 片距是从 flags 推的, 不是常量: 3-bit 帧 0x9000。 */
    uint16_t n_slices;    /* 权威片数, 1..PVS_N_SLICES_MAX_F(flags) (1-bit 720 /
                           * 3-bit 240)。单面老帧 = 360; DUAL_FACE 时 = nA+nB;
                           * FOLD_A 时 nA = nB/2 (360 槽下就是老的 180)。
                           * 🔴 nA/nB 由 n_slices 推 (DUAL_FACE: nA = FOLD_A ?
                           * n_slices/3 : n_slices/2), **不是**写死的 360/180 ——
                           * 3-bit 走的是每面 60 槽。 */
    uint16_t flags;       /* PVS_FLAG_* */
} pvs_hdr_t;              /* 16 bytes */
#pragma pack(pop)

/*
 * Receiver skeleton:
 *   read 16B hdr; check magic/raw_len/n_slices; read comp_len bytes;
 *   if (flags & PVS_FLAG_LZ4)   LZ4_decompress_safe() from <lz4.h> (raw block!):
 *       int n = LZ4_decompress_safe(payload, frame_buf, comp_len, raw_len);
 *       // n == raw_len 才算成功; n < 0 = 流损坏/给错了 .lz4 帧格式
 *   else if (flags & PVS_FLAG_ZLIB)  uncompress() from <zlib.h> (libz, Debian: zlib1g-dev):
 *       uLongf dst = PVS_FRAME_RAW;
 *       uncompress(frame_buf, &dst, payload, comp_len);  // Z_OK && dst == PVS_FRAME_RAW
 *   else if (flags & PVS_FLAG_RLE)  zero-run decode:
 *       0x00 escape: [0x00][run_u16_le] emits `run` zero bytes; any other
 *       byte is a literal. (Fallback codec only; zlib measured 2x better.)
 *   else memcpy raw.
 *   write ACK byte.
 */

#endif /* POV_STREAM_PROTOCOL_H */
