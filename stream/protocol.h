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
 *   if (flags & PVS_FLAG_ZLIB)  uncompress() from <zlib.h> (libz, Debian: zlib1g-dev):
 *       uLongf dst = PVS_FRAME_RAW;
 *       uncompress(frame_buf, &dst, payload, comp_len);  // Z_OK && dst == PVS_FRAME_RAW
 *   else if (flags & PVS_FLAG_RLE)  zero-run decode:
 *       0x00 escape: [0x00][run_u16_le] emits `run` zero bytes; any other
 *       byte is a literal. (Fallback codec only; zlib measured 2x better.)
 *   else memcpy raw.
 *   write ACK byte.
 */

#endif /* POV_STREAM_PROTOCOL_H */
