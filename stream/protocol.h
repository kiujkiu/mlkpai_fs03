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

/* ---- 2026-07-31 v3.1 偏心屏: 帧长度不再是编译期常量 ----------------------
 * 机械 v3.1 把屏模组整体偏 6.7mm, 两 LED 面落在 X=0 / X=+13.4 → 两面不再关于
 * 转轴对称, 「屏B@θ ≡ 屏A@(θ+180)」作废 ⇒ 单份 360 片喂两面的前提没了。
 * 于是帧里可能装 1 面或 2 面, 面A 还可能折叠成 180 片 (穿心面 slice_i 与
 * slice_{i+180} 互为镜像)。
 *
 * **头里的 n_slices 是权威**, 接收方按它算长度, 不再硬校验 == PVS_N_SLICES:
 *     raw_len MUST == n_slices * PVS_SLICE_STRIDE   (<= PVS_FRAME_RAW_MAX)
 * 老帧 (n_slices=360, 无新 flag) 语义逐字节不变, 老 frames_* 继续可用。
 *
 * 载荷排布 (解压后):
 *   单面:            [面 0 .. n_slices-1]
 *   PVS_FLAG_DUAL_FACE: [面A 0..nA-1][面B 0..nB-1], nA+nB = n_slices,
 *                       nA = PVS_FLAG_FOLD_A ? 180 : 360, nB = 360
 *   PVS_FLAG_FOLD_A:  面A 只送 0..179; 180..359 由 PL 取 idx-180 再做镜像置换
 *                     (仅穿心面 axis_off==0 成立, 发送端必须自检过)
 */
#define PVS_N_SLICES_MAX 720
#define PVS_FRAME_RAW_MAX (PVS_N_SLICES_MAX * PVS_SLICE_STRIDE)  /* 8847360 */
#define PVS_N_SLICES_FOLD 180              /* 折叠后的面A 片数 */

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
#define PVS_FLAG_FOLD_A    (1u << 4)  /* 面A 折叠: 只送 180 片 (θ=0..179°)。
                                       * 仅当面A 穿心 (垂距 0) 时合法 —— 此时
                                       * slice_i ≡ mirror(slice_{i+180}) 严格成立,
                                       * PL 用 idx>=180 → 取 idx-180 + 镜像置换补齐。
                                       * 发送端必须先跑打包域自检再置位。*/
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
#define PVS_MAX_STREAMS  16           /* 流表条数上限 (表最大 4+16*8 = 132 B) */
/* flags == 0 -> payload is raw (comp_len == raw_len) */

/* per-frame ACK byte, sent by receiver after decompress+verify(+commit) */
#define PVS_ACK          0x06       /* frame OK, send next */
#define PVS_NAK          0x15       /* error; sender aborts stream */

#pragma pack(push, 1)
typedef struct {
    char     magic[4];    /* "PVS1" */
    uint32_t comp_len;    /* payload length in bytes as transmitted */
    uint32_t raw_len;     /* decompressed length; MUST == n_slices * PVS_SLICE_STRIDE
                           * 且 <= PVS_FRAME_RAW_MAX (老帧里就是 PVS_FRAME_RAW) */
    uint16_t n_slices;    /* 权威片数, 1..PVS_N_SLICES_MAX。单面老帧 = 360;
                           * DUAL_FACE 时 = nA+nB; FOLD_A 时 nA=180 */
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
