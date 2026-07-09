/*
 * pvs_codec.h - PVS RLE / delta codec, shared by pov_rxd.c, test_local.c
 * and bench_delta.c (header-only, static inline; keeps the daemon a
 * single-file build).
 *
 * Wire codecs (see stream/pc/protocol.md):
 *   zero-run RLE   0x00 escape + run:u16le zero bytes; any other byte is a
 *                  literal (bare 0x00 never appears as a literal). run == 0
 *                  is legal and emits nothing.
 *   delta (v2)     payload decodes to a FRAME_RAW-sized XOR mask against the
 *                  receiver's current frame (shadow). Composable with RLE
 *                  (PVS_FLAG_DELTA|PVS_FLAG_RLE is the normal form): the RLE
 *                  stream is applied directly - zero runs just advance the
 *                  cursor, literals XOR into the shadow - so cost scales with
 *                  changed bytes, not frame size.
 *
 * Dirty-page tracking: the frame is viewed as 4 KiB pages
 * (PVS_FRAME_RAW = 4423680 = 1080 * 4096 exactly). Decoders mark
 * dirty[page] = 1 for every page they touch; the daemon accumulates these
 * per DDR bank and copies only dirty spans into the uncached mapping.
 */
#ifndef PVS_CODEC_H
#define PVS_CODEC_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "../protocol.h"

#define PVS_PAGE_SHIFT 12
#define PVS_PAGE_SIZE  (1u << PVS_PAGE_SHIFT)
#define PVS_N_PAGES    (PVS_FRAME_RAW >> PVS_PAGE_SHIFT)   /* 1080, exact */

#if (PVS_N_PAGES << PVS_PAGE_SHIFT) != PVS_FRAME_RAW
#error "PVS_FRAME_RAW must be a multiple of the dirty-page size"
#endif

/* zero-run RLE decode into dst (full frame). 0 = ok. */
static inline int pvs_rle_decode(const uint8_t *src, size_t slen,
                          uint8_t *dst, size_t dlen)
{
    size_t si = 0, di = 0;
    while (si < slen) {
        uint8_t b = src[si++];
        if (b == 0x00) {
            if (si + 2 > slen) return -1;
            uint32_t run = (uint32_t)src[si] | ((uint32_t)src[si + 1] << 8);
            si += 2;
            if (di + run > dlen) return -1;
            memset(dst + di, 0, run);
            di += run;
        } else {
            if (di >= dlen) return -1;
            dst[di++] = b;
        }
    }
    return di == dlen ? 0 : -1;
}

/* zero-run RLE encode (worst case ~2.5x n; size dst accordingly). */
static inline size_t pvs_rle_encode(const uint8_t *src, size_t n, uint8_t *dst)
{
    size_t di = 0, i = 0;
    while (i < n) {
        if (src[i] == 0) {
            size_t run = 0;
            while (i < n && src[i] == 0 && run < 65535) { run++; i++; }
            dst[di++] = 0x00;
            dst[di++] = (uint8_t)(run & 0xff);
            dst[di++] = (uint8_t)(run >> 8);
        } else {
            dst[di++] = src[i++];
        }
    }
    return di;
}

/*
 * Streaming RLE-delta apply: decode the RLE stream and XOR literals into
 * shadow in one pass. Zero runs only advance the cursor (XOR 0 = no-op), so
 * untouched bytes are never read or written - this is the 30 fps fast path.
 * Marks dirty[page] for every literal. Returns changed-byte count via
 * *n_lit (may be NULL), 0 = ok, -1 = malformed (shadow may be partially
 * updated - caller must invalidate it).
 */
static inline int pvs_delta_rle_apply(const uint8_t *src, size_t slen,
                               uint8_t *shadow, size_t dlen,
                               uint8_t *dirty, uint32_t *n_lit)
{
    size_t si = 0, di = 0;
    uint32_t lit = 0;
    while (si < slen) {
        uint8_t b = src[si++];
        if (b == 0x00) {
            if (si + 2 > slen) return -1;
            uint32_t run = (uint32_t)src[si] | ((uint32_t)src[si + 1] << 8);
            si += 2;
            if (di + run > dlen) return -1;
            di += run;
        } else {
            if (di >= dlen) return -1;
            shadow[di] ^= b;
            dirty[di >> PVS_PAGE_SHIFT] = 1;
            di++;
            lit++;
        }
    }
    if (n_lit) *n_lit = lit;
    return di == dlen ? 0 : -1;
}

/*
 * Full-buffer XOR apply (for PVS_FLAG_DELTA with raw or zlib payload):
 * XOR delta into shadow 64 bits at a time, marking pages whose delta is
 * non-zero. len must be a multiple of PVS_PAGE_SIZE (PVS_FRAME_RAW is).
 */
static inline void pvs_xor_apply_pages(const uint8_t *delta, uint8_t *shadow,
                                size_t len, uint8_t *dirty)
{
    size_t np = len >> PVS_PAGE_SHIFT;
    for (size_t p = 0; p < np; p++) {
        const uint64_t *d = (const uint64_t *)(delta + (p << PVS_PAGE_SHIFT));
        uint64_t *s = (uint64_t *)(shadow + (p << PVS_PAGE_SHIFT));
        uint64_t acc = 0;
        for (unsigned i = 0; i < PVS_PAGE_SIZE / 8; i++) {
            acc |= d[i];
            s[i] ^= d[i];
        }
        if (acc) dirty[p] = 1;
    }
}

#endif /* PVS_CODEC_H */
