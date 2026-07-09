/*
 * protocol.h — POV volumetric display streaming protocol (PVS1 + v2 delta).
 * Shared between PC sender (stream/pc/povstream.py) and board C receiver.
 * Authoritative spec: stream/pc/protocol.md. Keep both in sync.
 *
 * Transport: TCP, PC connects to board, default port 9500.
 * Stream = sequence of frames, each: 16-byte header | payload.
 * Receiver replies 1 ACK byte after each fully processed frame.
 * Sender closes the connection to end the stream.
 *
 * v2 (2026-07): same 'PVS1' magic + 16B header, new flags bit2 = delta.
 * A delta frame's payload decodes to an XOR mask against the PREVIOUS raw
 * frame; composable with bit0 RLE (normal form: DELTA|RLE, XOR of adjacent
 * 1-bit frames is overwhelmingly zeros). First frame of every connection
 * must be a full frame (keyframe); a receiver that cannot anchor a delta
 * (no previous frame this connection) replies NAK but KEEPS the connection
 * open — the sender recovers by resending as a keyframe. Old PVS1 senders
 * never set bit2 and are unaffected.
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
#define PVS_N_SLICES     360
#define PVS_SLICE_DATA   11664
#define PVS_SLICE_STRIDE 0x3000            /* 12288 */
#define PVS_FRAME_RAW    (PVS_N_SLICES * PVS_SLICE_STRIDE)  /* 4423680 */

/* flags bits */
#define PVS_FLAG_RLE     (1u << 0)  /* payload = zero-run RLE (see protocol.md) */
#define PVS_FLAG_ZLIB    (1u << 1)  /* payload = zlib stream (RFC1950). CHOSEN default. */
#define PVS_FLAG_DELTA   (1u << 2)  /* v2: payload decodes to XOR mask vs previous
                                     * raw frame. Wire forms: DELTA|RLE|ZLIB =
                                     * zlib-wrapped RLE stream (normal: sender
                                     * picks the smaller of RLE / zlib(RLE));
                                     * DELTA|RLE = bare RLE stream; DELTA|ZLIB /
                                     * bare DELTA = full 4.4MB mask (zlib'd/raw).
                                     * RLE stream must decode to <= raw_len. */
/* flags == 0 -> payload is raw (comp_len == raw_len) */

/* per-frame ACK byte, sent by receiver after decompress+verify(+commit) */
#define PVS_ACK          0x06       /* frame OK, send next */
#define PVS_NAK          0x15       /* error; sender aborts stream. EXCEPTION
                                     * (v2): NAK on an unanchorable delta frame
                                     * keeps the connection open; sender must
                                     * resend as keyframe (full frame). */

#pragma pack(push, 1)
typedef struct {
    char     magic[4];    /* "PVS1" */
    uint32_t comp_len;    /* payload length in bytes as transmitted */
    uint32_t raw_len;     /* decompressed length, MUST be PVS_FRAME_RAW */
    uint16_t n_slices;    /* MUST be PVS_N_SLICES (360) */
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
