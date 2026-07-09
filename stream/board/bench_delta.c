/*
 * bench_delta.c - x86 microbench of the v2 board-side decode path.
 *
 * usage: ./bench_delta frameA.bin frameB.bin [iters]
 *
 * Loads two adjacent raw frames (FRAME_RAW each, e.g. from
 * stream/pc/frames_palace/), builds the XOR mask + RLE payload exactly like
 * the sender, then times the receiver-side work over `iters` (default 200)
 * alternating applications (XOR is its own inverse, so shadow toggles A/B):
 *
 *   1. pvs_delta_rle_apply   (cached shadow update, the 30fps fast path)
 *   2. sync_bank simulation  (memcpy dirty spans shadow -> bank buffer)
 *   3. reference: zlib-6 inflate of the full frame + full 4.4MB memcpy
 *      (the PVS1 path this replaces)
 *
 * Prints per-frame µs and effective MB/s. A9 numbers ~5-10x these (run the
 * committed ARM binary with --bench on the board for ground truth).
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <zlib.h>

#include "../protocol.h"
#include "pvs_codec.h"

static uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000u + (uint64_t)(t.tv_nsec / 1000);
}

static uint8_t *load_frame(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(1); }
    uint8_t *buf = malloc(PVS_FRAME_RAW);
    if (!buf || fread(buf, 1, PVS_FRAME_RAW, f) != PVS_FRAME_RAW) {
        fprintf(stderr, "%s: need exactly %u bytes\n", path, PVS_FRAME_RAW);
        exit(1);
    }
    fclose(f);
    return buf;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s frameA.bin frameB.bin [iters]\n", argv[0]);
        return 2;
    }
    int iters = argc > 3 ? atoi(argv[3]) : 200;
    if (iters < 2) iters = 2;
    iters &= ~1;                                   /* even: shadow ends == A */

    uint8_t *fa = load_frame(argv[1]);
    uint8_t *fb = load_frame(argv[2]);
    uint8_t *mask    = malloc(PVS_FRAME_RAW);
    uint8_t *payload = malloc(3 * PVS_FRAME_RAW);
    uint8_t *shadow  = malloc(PVS_FRAME_RAW);
    uint8_t *bank    = malloc(PVS_FRAME_RAW);
    uint8_t *zbuf    = malloc(compressBound(PVS_FRAME_RAW));
    uint8_t dirty[PVS_N_PAGES];
    if (!mask || !payload || !shadow || !bank || !zbuf) { perror("malloc"); return 1; }

    uint32_t n_changed = 0;
    for (size_t i = 0; i < PVS_FRAME_RAW; i++) {
        mask[i] = fa[i] ^ fb[i];
        if (mask[i]) n_changed++;
    }
    size_t plen = pvs_rle_encode(mask, PVS_FRAME_RAW, payload);

    uint8_t *wire = malloc(compressBound(plen));      /* 0x7: zlib(RLE) */
    uLongf wlen = compressBound(plen);
    if (!wire || compress2(wire, &wlen, payload, plen, 6) != Z_OK) return 1;

    uLongf zlen = compressBound(PVS_FRAME_RAW);
    if (compress2(zbuf, &zlen, fb, PVS_FRAME_RAW, 6) != Z_OK) return 1;

    printf("frames: %s -> %s\n", argv[1], argv[2]);
    printf("raw %u B | changed bytes %u (%.2f%%) | delta+RLE %zu B (%.1fx) | "
           "0x7 zlib(RLE) %lu B (%.1fx) | zlib-6 full %lu B (%.1fx)\n",
           PVS_FRAME_RAW, n_changed, 100.0 * n_changed / PVS_FRAME_RAW,
           plen, (double)PVS_FRAME_RAW / plen,
           (unsigned long)wlen, (double)PVS_FRAME_RAW / wlen,
           (unsigned long)zlen, (double)PVS_FRAME_RAW / zlen);

    /* --- 1. delta RLE apply on cached shadow --- */
    memcpy(shadow, fa, PVS_FRAME_RAW);
    memset(dirty, 0, sizeof dirty);
    uint64_t t0 = now_us();
    for (int i = 0; i < iters; i++) {
        uint32_t lit;
        if (pvs_delta_rle_apply(payload, plen, shadow, PVS_FRAME_RAW,
                                dirty, &lit) != 0) {
            fprintf(stderr, "apply failed\n");
            return 1;
        }
    }
    uint64_t dt_apply = (now_us() - t0) / iters;
    if (memcmp(shadow, fa, PVS_FRAME_RAW) != 0) {   /* even iters -> back to A */
        fprintf(stderr, "FAIL: shadow round-trip mismatch\n");
        return 1;
    }

    unsigned n_dirty = 0;
    for (unsigned p = 0; p < PVS_N_PAGES; p++) n_dirty += dirty[p];

    /* --- 1b. wire form 0x7: inflate small payload, then apply --- */
    uint8_t *rle2 = malloc(3 * PVS_FRAME_RAW);
    if (!rle2) { perror("malloc"); return 1; }
    t0 = now_us();
    for (int i = 0; i < iters; i++) {
        uLongf dlen = 3 * PVS_FRAME_RAW;
        uint32_t lit;
        if (uncompress(rle2, &dlen, wire, wlen) != Z_OK ||
            pvs_delta_rle_apply(rle2, dlen, shadow, PVS_FRAME_RAW,
                                dirty, &lit) != 0) {
            fprintf(stderr, "0x7 decode failed\n");
            return 1;
        }
    }
    uint64_t dt_apply7 = (now_us() - t0) / iters;
    if (memcmp(shadow, fa, PVS_FRAME_RAW) != 0) {
        fprintf(stderr, "FAIL: 0x7 shadow round-trip mismatch\n");
        return 1;
    }

    /* --- 2. dirty-span sync (union of the same delta twice = same pages) --- */
    t0 = now_us();
    for (int i = 0; i < iters; i++) {
        for (unsigned p = 0; p < PVS_N_PAGES; ) {
            if (!dirty[p]) { p++; continue; }
            unsigned q = p;
            while (q < PVS_N_PAGES && dirty[q]) q++;
            memcpy(bank + ((size_t)p << PVS_PAGE_SHIFT),
                   shadow + ((size_t)p << PVS_PAGE_SHIFT),
                   (size_t)(q - p) << PVS_PAGE_SHIFT);
            p = q;
        }
    }
    uint64_t dt_sync = (now_us() - t0) / iters;

    /* --- 3. reference PVS1 path: zlib inflate + full-frame memcpy --- */
    t0 = now_us();
    for (int i = 0; i < iters; i++) {
        uLongf dlen = PVS_FRAME_RAW;
        if (uncompress(shadow, &dlen, zbuf, zlen) != Z_OK) return 1;
    }
    uint64_t dt_inflate = (now_us() - t0) / iters;
    t0 = now_us();
    for (int i = 0; i < iters; i++) {
        memcpy(bank, shadow, PVS_FRAME_RAW);
        __asm__ volatile("" ::: "memory");   /* 阻止重复 memcpy 被合并 */
    }
    uint64_t dt_fullcpy = (now_us() - t0) / iters;

    printf("dirty pages: %u/%u (%.1f%%) -> %u KiB bank write\n",
           n_dirty, PVS_N_PAGES, 100.0 * n_dirty / PVS_N_PAGES,
           n_dirty * 4);
    printf("%-28s %8llu us/frame  (%7.1f MB/s of frame)\n", "delta+RLE apply",
           (unsigned long long)dt_apply,
           dt_apply ? PVS_FRAME_RAW / (double)dt_apply : 0.0);
    printf("%-28s %8llu us/frame  (inflate %luB + apply)\n",
           "0x7 inflate+apply",
           (unsigned long long)dt_apply7, (unsigned long)wlen);
    printf("%-28s %8llu us/frame  (%7.1f MB/s written)\n", "dirty-span bank sync",
           (unsigned long long)dt_sync,
           dt_sync ? (n_dirty * 4096.0) / dt_sync : 0.0);
    printf("%-28s %8llu us/frame  (v2 total: %llu us)\n", "  -> v2 decode+write",
           (unsigned long long)(dt_apply + dt_sync),
           (unsigned long long)(dt_apply + dt_sync));
    printf("%-28s %8llu us/frame  (%7.1f MB/s)\n", "zlib-6 inflate (PVS1)",
           (unsigned long long)dt_inflate,
           dt_inflate ? PVS_FRAME_RAW / (double)dt_inflate : 0.0);
    printf("%-28s %8llu us/frame  (PVS1 total: %llu us)\n", "full 4.4MB memcpy",
           (unsigned long long)dt_fullcpy,
           (unsigned long long)(dt_inflate + dt_fullcpy));
    printf("v2 speedup vs PVS1 path: %.1fx\n",
           (double)(dt_inflate + dt_fullcpy) / (double)(dt_apply + dt_sync ? dt_apply + dt_sync : 1));
    return 0;
}
