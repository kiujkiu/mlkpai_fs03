/*
 * pov_rxd.c - board-side receiver daemon for the PVS POV frame stream
 * (PVS1 + v2 delta, see stream/protocol.h + stream/pc/protocol.md).
 *
 * Target: Zynq-7020 (MLKPAI-FS03), ARM Cortex-A9, Debian buster userspace,
 * kernel 6.6. Build static with arm-linux-gnueabihf-gcc (see Makefile).
 *
 * Protocol: TCP server on :9500; per frame: 16B header | payload; reply
 * 1 ACK byte. v2 adds flags bit2 = delta (XOR mask vs previous frame,
 * normally RLE-coded). First frame of each connection must be a full
 * frame; a delta that cannot be anchored is NAKed WITHOUT closing the
 * connection (sender resends as keyframe).
 *
 * Hardware contract (verified PL POV engine @ 0x40010000):
 *   0x00 R  STATUS       engine health
 *   0x10 W  POV_CTRL     n_slices<<16 | fake_en<<1 | pov_en
 *        R               bit31=locked, [15:0]=current slice_idx
 *   0x14 W  fake_period  aclk ticks/slice @ 50 MHz (R: rev_period)
 *   0x18 W  slice_base   DDR byte addr of frame start; latched per-slice
 *                        at fetch_go -> takes effect from the next slice.
 *
 * Frame region: Linux boots with mem=256M, so phys 0x10000000..0x1FFFFFFF
 * is invisible to the kernel and reserved for frames. Double buffer:
 *   bank A @ 0x10000000, bank B @ 0x10500000, each 360*0x3000 = 0x438000 B.
 *
 * Cache coherency: on 32-bit ARM, kernel 6.6 arch/arm/mm/mmu.c
 * phys_mem_access_prot() returns pgprot_noncached() for any pfn where
 * !pfn_valid(pfn). With mem=256M the frame region (and the PL register
 * page) are not kernel-managed RAM, so the /dev/mem mmap is uncached /
 * strongly-ordered: CPU stores go straight to DRAM, nothing is stuck in
 * L1/L2, and the PL HP-port reads see the data with no cache maintenance.
 * We still issue __sync_synchronize() (DMB) after the bank writes and
 * before/after the slice_base register write so the frame data is
 * globally observable before the engine can latch the new base.
 *
 * 30 fps ingest path (v2): the current raw frame lives in a CACHED
 * malloc'd shadow. Deltas are RLE-applied directly to the shadow (cost
 * scales with changed bytes, not the 4.4 MB frame), tracking dirty 4 KiB
 * pages. Each DDR bank keeps a dirty accumulator = pages where the bank
 * differs from the shadow; after every frame only the accumulated dirty
 * spans of the inactive bank are memcpy'd through the uncached mapping
 * and its accumulator cleared, while this frame's dirty pages are OR-ed
 * into BOTH accumulators. This holds the invariant "bank == shadow for
 * every page not in its accumulator" regardless of flips, drops (same
 * bank written twice) or keyframes (all pages dirty), so the stale
 * inactive bank (normally 2 frames behind = union of last two deltas)
 * always comes out exactly equal to the current frame. --verify memcmps
 * the invariant after every frame (test/debug).
 *
 * Display-never-blocks policy: the PL engine keeps refetching whatever
 * slice_base points at; we only flip banks when the slice counter wraps
 * near 0. If the sender outruns the display (data already pending on the
 * socket while we wait for the wrap), the just-received frame is ACKed
 * and dropped (no flip) and we immediately receive the newer one.
 * --bench disables the wrap gate (flip + ACK immediately) so raw ingest
 * throughput is measurable beyond the rotation rate.
 *
 * Build modes:
 *   default          real /dev/mem + PL registers (ARM board)
 *   -DSIM_NO_DEVMEM  x86 dev-machine simulation: malloc banks, stubbed
 *                    registers with an advancing fake slice counter.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <stdarg.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <zlib.h>

#include "../protocol.h"
#include "pvs_codec.h"

/* ---- hardware constants ------------------------------------------------ */
#define REG_PHYS_DEFAULT   0x40010000u
#define REG_MAP_LEN        0x1000u
#define REG_STATUS         0x00
#define REG_POV_CTRL       0x10
#define REG_FAKE_PERIOD    0x14
#define REG_SLICE_BASE     0x18

#define FRAME_PHYS_DEFAULT 0x10000000u
#define BANK_STRIDE        0x00500000u          /* bank B offset from base */
#define BANK_BYTES         PVS_FRAME_RAW        /* 0x438000, page multiple */
#define FRAME_MAP_LEN      (BANK_STRIDE + BANK_BYTES)

#define ACLK_HZ            50000000u
#define SLICE_WRAP_THRESH  8                    /* "near 0" */
#define FLIP_TIMEOUT_MS    2000                 /* engine idle? flip anyway */
#define COMP_LEN_MAX       (PVS_FRAME_RAW + 0x10000u)
#define STAT_EVERY         30                   /* frames per stats line */

/* ---- logging ------------------------------------------------------------ */
static void logts(const char *fmt, ...)
{
    struct timespec ts;
    struct tm tm;
    va_list ap;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    printf("[%02d:%02d:%02d.%03ld] ", tm.tm_hour, tm.tm_min, tm.tm_sec,
           ts.tv_nsec / 1000000L);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

static uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000u + (uint64_t)(t.tv_nsec / 1000);
}

/* ---- register + bank access (real vs sim) ------------------------------- */
static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) { (void)sig; g_stop = 1; }

static uint8_t  *g_bank[2];      /* virtual addresses of bank A/B */
static uint32_t  g_bank_phys[2]; /* physical addresses (what 0x18 wants) */

/* options */
static int g_bench  = 0;         /* ACK right after decode+write, no wrap gate */
static int g_verify = 0;         /* memcmp bank-union invariant every frame */
static int g_crc    = 0;         /* crc32 the shadow every frame (tests) */

/* v2 state: cached shadow of the current raw frame + per-bank dirty accum */
static uint8_t *g_shadow;
static int      g_shadow_valid;                  /* delta anchor available? */
static uint8_t  g_dirty_accum[2][PVS_N_PAGES];   /* bank differs from shadow */

#ifndef SIM_NO_DEVMEM

static volatile uint32_t *g_regs;

static int hw_init(uint32_t reg_phys, uint32_t frame_phys)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem (need root)"); return -1; }

    void *r = mmap(NULL, REG_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                   fd, reg_phys);
    if (r == MAP_FAILED) { perror("mmap regs"); return -1; }
    g_regs = (volatile uint32_t *)r;

    void *f = mmap(NULL, FRAME_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                   fd, frame_phys);
    if (f == MAP_FAILED) { perror("mmap frame region"); return -1; }
    g_bank[0] = (uint8_t *)f;
    g_bank[1] = (uint8_t *)f + BANK_STRIDE;
    g_bank_phys[0] = frame_phys;
    g_bank_phys[1] = frame_phys + BANK_STRIDE;
    close(fd); /* mappings stay valid */
    return 0;
}

static uint32_t reg_rd(uint32_t off)            { return g_regs[off / 4]; }
static void     reg_wr(uint32_t off, uint32_t v){ g_regs[off / 4] = v;    }

#else /* SIM_NO_DEVMEM: x86 test build ------------------------------------ */

static uint32_t g_sim_regs[REG_MAP_LEN / 4];
static uint32_t g_sim_slice;   /* fake advancing slice counter */

static int hw_init(uint32_t reg_phys, uint32_t frame_phys)
{
    (void)reg_phys;
    uint8_t *f = malloc(FRAME_MAP_LEN);
    if (!f) { perror("malloc banks"); return -1; }
    g_bank[0] = f;
    g_bank[1] = f + BANK_STRIDE;
    g_bank_phys[0] = frame_phys;
    g_bank_phys[1] = frame_phys + BANK_STRIDE;
    logts("SIM: banks malloc'd, registers stubbed");
    return 0;
}

static uint32_t reg_rd(uint32_t off)
{
    if (off == REG_POV_CTRL) {   /* advance fake slice_idx on every read */
        g_sim_slice = (g_sim_slice + 23) % PVS_N_SLICES;
        return 0x80000000u | g_sim_slice;
    }
    return g_sim_regs[off / 4];
}

static void reg_wr(uint32_t off, uint32_t v)
{
    g_sim_regs[off / 4] = v;
    logts("SIM: reg[0x%02x] <= 0x%08x", off, v);
}

#endif /* SIM_NO_DEVMEM */

/* ---- socket helpers ------------------------------------------------------ */
static int recv_full(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len) {
        ssize_t n = recv(fd, p, len, 0);
        if (n == 0) return 0;                       /* peer closed */
        if (n < 0) {
            if (errno == EINTR) { if (g_stop) return -1; continue; }
            return -1;
        }
        p += n; len -= (size_t)n;
    }
    return 1;
}

static int send_byte(int fd, uint8_t b)
{
    ssize_t n;
    do n = send(fd, &b, 1, MSG_NOSIGNAL); while (n < 0 && errno == EINTR && !g_stop);
    return n == 1 ? 0 : -1;
}

static int sock_has_pending(int fd)
{
    struct pollfd pf = { .fd = fd, .events = POLLIN };
    return poll(&pf, 1, 0) > 0;
}

/* ---- bank sync ------------------------------------------------------------
 * Copy every page accumulated as dirty for bank `b` from the shadow into the
 * uncached bank mapping, coalescing consecutive pages into single memcpy
 * spans (glibc memcpy uses NEON on the A9), then clear the accumulator.
 * Returns bytes written.
 */
static size_t sync_bank(int b)
{
    uint8_t *acc = g_dirty_accum[b];
    size_t written = 0;
    for (unsigned p = 0; p < PVS_N_PAGES; ) {
        if (!acc[p]) { p++; continue; }
        unsigned q = p;
        while (q < PVS_N_PAGES && acc[q]) q++;
        size_t off = (size_t)p << PVS_PAGE_SHIFT;
        size_t len = (size_t)(q - p) << PVS_PAGE_SHIFT;
        memcpy(g_bank[b] + off, g_shadow + off, len);
        written += len;
        p = q;
    }
    memset(acc, 0, PVS_N_PAGES);
    return written;
}

/* ---- flip logic ----------------------------------------------------------
 * Wait until slice_idx wraps near 0, then point the engine at `bank`.
 * While waiting, watch the socket: if a newer frame is already arriving,
 * bail out (caller drops this frame). Returns 1=flipped, 0=dropped.
 */
static int flip_when_wrapped(int client_fd, int bank)
{
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (;;) {
        uint32_t slice = reg_rd(REG_POV_CTRL) & 0xffffu;
        if (slice < SLICE_WRAP_THRESH)
            break;
        if (client_fd >= 0 && sock_has_pending(client_fd))
            return 0;                                    /* newer frame: drop */
        clock_gettime(CLOCK_MONOTONIC, &t);
        long ms = (t.tv_sec - t0.tv_sec) * 1000 + (t.tv_nsec - t0.tv_nsec) / 1000000;
        if (ms > FLIP_TIMEOUT_MS) {
            logts("WARN: slice_idx never wrapped (engine idle?), flipping anyway");
            break;
        }
        usleep(200);
        if (g_stop) break;
    }
    __sync_synchronize();                 /* frame data globally visible ... */
    reg_wr(REG_SLICE_BASE, g_bank_phys[bank]);
    __sync_synchronize();                 /* ... before + after base update  */
    return 1;
}

/* ---- per-connection frame loop ------------------------------------------ */
static int flags_ok(uint16_t f)
{
    switch (f) {
    case 0:
    case PVS_FLAG_RLE:
    case PVS_FLAG_ZLIB:
    case PVS_FLAG_DELTA:                    /* raw XOR mask */
    case PVS_FLAG_DELTA | PVS_FLAG_RLE:     /* RLE-coded mask */
    case PVS_FLAG_DELTA | PVS_FLAG_ZLIB:    /* zlib'd raw mask */
    case PVS_FLAG_DELTA | PVS_FLAG_RLE | PVS_FLAG_ZLIB: /* zlib'd RLE stream
                                             * (normal v2 wire form) */
        return 1;
    default:
        return 0;
    }
}

static int serve_client(int fd, uint8_t *cbuf, uint8_t *staging)
{
    static int active = 0;    /* persists across connections */
    unsigned seq = 0;
    uint64_t st_recv = 0, st_dec = 0, st_wr = 0;      /* µs accumulators */
    uint64_t mx_recv = 0, mx_dec = 0, mx_wr = 0;
    uint64_t st_dirty = 0, st_wire = 0;
    unsigned st_n = 0;

    /* v2: sender state is per-connection - a fresh connection means the
     * sender can't know our shadow, so require a keyframe before deltas. */
    g_shadow_valid = 0;

    for (;;) {
        pvs_hdr_t h;
        int r = recv_full(fd, &h, sizeof h);
        if (r <= 0) return r;

        if (memcmp(h.magic, PVS_MAGIC, 4) != 0 ||
            h.raw_len  != PVS_FRAME_RAW        ||
            h.n_slices != PVS_N_SLICES         ||
            h.comp_len == 0 || h.comp_len > COMP_LEN_MAX ||
            !flags_ok(h.flags)) {
            logts("NAK: bad header (magic=%.4s comp=%u raw=%u n=%u flags=0x%x)",
                  h.magic, h.comp_len, h.raw_len, h.n_slices, h.flags);
            send_byte(fd, PVS_NAK);
            return -1;
        }

        int is_delta = (h.flags & PVS_FLAG_DELTA) != 0;
        uint16_t codec = h.flags & (PVS_FLAG_RLE | PVS_FLAG_ZLIB);

        if (codec == 0 && h.comp_len != h.raw_len) {
            logts("NAK: uncoded payload but comp_len %u != raw_len", h.comp_len);
            send_byte(fd, PVS_NAK);
            return -1;
        }

        /* receive payload. Full raw frames go straight into the shadow
         * (cached), everything else into cbuf. */
        uint64_t t0 = now_us();
        uint8_t *dst = cbuf;
        if (!is_delta && codec == 0) {
            dst = g_shadow;
            g_shadow_valid = 0;           /* invalid while partially written */
        }
        r = recv_full(fd, dst, h.comp_len);
        if (r <= 0) return r;
        uint64_t t1 = now_us();

        /* v2: delta without an anchor -> NAK but keep the connection; the
         * sender recovers by resending this frame as a keyframe. */
        if (is_delta && !g_shadow_valid) {
            logts("NAK: delta frame without anchor (keyframe needed), conn kept");
            if (send_byte(fd, PVS_NAK) != 0) return -1;
            continue;
        }

        /* decode into the cached shadow, tracking dirty 4K pages */
        uint8_t this_dirty[PVS_N_PAGES];
        memset(this_dirty, 0, sizeof this_dirty);
        uint32_t n_lit = 0;

        if (!is_delta) {
            if (h.flags & PVS_FLAG_ZLIB) {
                uLongf dlen = PVS_FRAME_RAW;
                g_shadow_valid = 0;
                int zr = uncompress(g_shadow, &dlen, cbuf, h.comp_len);
                if (zr != Z_OK || dlen != PVS_FRAME_RAW) {
                    logts("NAK: zlib inflate failed (rc=%d dlen=%lu)", zr,
                          (unsigned long)dlen);
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } else if (h.flags & PVS_FLAG_RLE) {
                g_shadow_valid = 0;
                if (pvs_rle_decode(cbuf, h.comp_len, g_shadow, PVS_FRAME_RAW) != 0) {
                    logts("NAK: RLE decode failed");
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } /* else raw: already in shadow */
            /* keyframe: whole frame dirty for both banks */
            memset(g_dirty_accum[0], 1, PVS_N_PAGES);
            memset(g_dirty_accum[1], 1, PVS_N_PAGES);
            n_lit = PVS_FRAME_RAW;
        } else {
            const uint8_t *stream = cbuf;
            size_t slen = h.comp_len;
            if (codec == (PVS_FLAG_RLE | PVS_FLAG_ZLIB)) {
                /* zlib-wrapped RLE stream: inflate cost scales with the
                 * (small) RLE stream, not the 4.4MB frame */
                uLongf dlen = PVS_FRAME_RAW;   /* RLE stream must fit */
                int zr = uncompress(staging, &dlen, cbuf, h.comp_len);
                if (zr != Z_OK) {
                    logts("NAK: delta rle+zlib inflate failed (rc=%d)", zr);
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
                stream = staging;
                slen = dlen;
                codec = PVS_FLAG_RLE;          /* fall through to RLE apply */
            }
            if (codec == PVS_FLAG_RLE) {
                /* fast path: streaming apply, touches only changed bytes */
                if (pvs_delta_rle_apply(stream, slen, g_shadow,
                                        PVS_FRAME_RAW, this_dirty, &n_lit) != 0) {
                    g_shadow_valid = 0;   /* shadow possibly half-applied */
                    logts("NAK: delta RLE apply failed");
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } else {
                const uint8_t *mask = cbuf;
                if (codec == PVS_FLAG_ZLIB) {
                    uLongf dlen = PVS_FRAME_RAW;
                    int zr = uncompress(staging, &dlen, cbuf, h.comp_len);
                    if (zr != Z_OK || dlen != PVS_FRAME_RAW) {
                        logts("NAK: delta zlib inflate failed (rc=%d)", zr);
                        send_byte(fd, PVS_NAK);
                        return -1;
                    }
                    mask = staging;
                }
                pvs_xor_apply_pages(mask, g_shadow, PVS_FRAME_RAW, this_dirty);
            }
            /* this frame's dirt goes into BOTH bank accumulators */
            for (unsigned p = 0; p < PVS_N_PAGES; p++)
                if (this_dirty[p]) {
                    g_dirty_accum[0][p] = 1;
                    g_dirty_accum[1][p] = 1;
                }
        }
        g_shadow_valid = 1;
        uint64_t t2 = now_us();

        /* write only the accumulated dirty spans of the inactive bank */
        int inactive_bank = active ^ 1;
        size_t wrote = sync_bank(inactive_bank);
        __sync_synchronize();
        uint64_t t3 = now_us();

        uint32_t crc = g_crc ? crc32(0L, g_shadow, PVS_FRAME_RAW) : 0;

        if (g_verify) {   /* THE invariant: synced bank == current frame */
            if (memcmp(g_bank[inactive_bank], g_shadow, PVS_FRAME_RAW) != 0) {
                logts("VERIFY FAIL seq=%u bank=%d != shadow", seq, inactive_bank);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            logts("VERIFY OK seq=%u bank=%d", seq, inactive_bank);
        }

        int flipped = 0;
        if (g_bench) {
            /* bench: no wrap gate, no drop path - flip + ACK immediately */
            __sync_synchronize();
            reg_wr(REG_SLICE_BASE, g_bank_phys[inactive_bank]);
            __sync_synchronize();
            flipped = 1;
        } else if (!sock_has_pending(fd)) {
            /* drop path: newer frame already queued? ACK + skip flip */
            flipped = flip_when_wrapped(fd, inactive_bank);
        }

        if (send_byte(fd, PVS_ACK) != 0) return -1;
        if (flipped) active = inactive_bank;

        logts("FRAME seq=%u comp=%u flags=0x%x crc=%08x bank=%d %s "
              "lit=%u wrote=%uK",
              seq++, h.comp_len, h.flags, crc, inactive_bank,
              flipped ? "flipped" : "DROPPED(no-flip)",
              n_lit, (unsigned)(wrote >> 10));

        /* timing stats */
        uint64_t d_recv = t1 - t0, d_dec = t2 - t1, d_wr = t3 - t2;
        st_recv += d_recv; st_dec += d_dec; st_wr += d_wr;
        if (d_recv > mx_recv) mx_recv = d_recv;
        if (d_dec  > mx_dec)  mx_dec  = d_dec;
        if (d_wr   > mx_wr)   mx_wr   = d_wr;
        st_dirty += wrote; st_wire += h.comp_len;
        if (++st_n == STAT_EVERY) {
            logts("STAT %u frames: recv avg %lluus max %llu | decode avg %lluus "
                  "max %llu | write avg %lluus max %llu | wrote avg %lluK | "
                  "wire avg %lluK",
                  st_n,
                  (unsigned long long)(st_recv / st_n), (unsigned long long)mx_recv,
                  (unsigned long long)(st_dec / st_n),  (unsigned long long)mx_dec,
                  (unsigned long long)(st_wr / st_n),   (unsigned long long)mx_wr,
                  (unsigned long long)(st_dirty / st_n >> 10),
                  (unsigned long long)(st_wire / st_n >> 10));
            st_recv = st_dec = st_wr = 0;
            mx_recv = mx_dec = mx_wr = 0;
            st_dirty = st_wire = 0;
            st_n = 0;
        }
        if (g_stop) return -1;
    }
}

/* ---- main ---------------------------------------------------------------- */
static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [--port N] [--base HEXADDR] [--regs HEXADDR] [--fake RPS]\n"
        "          [--bench] [--verify] [--crc]\n"
        "  --port N       TCP listen port (default %d)\n"
        "  --base ADDR    frame region phys base (default 0x%08x)\n"
        "  --regs ADDR    POV engine AXI base    (default 0x%08x)\n"
        "  --fake RPS     enable motor-less fake-spin at RPS revs/sec\n"
        "                 (programs fake_period + POV_CTRL; otherwise the\n"
        "                  daemon never touches POV_CTRL - JTAG owns it)\n"
        "  --bench        ACK right after decode+write (no slice-wrap gate):\n"
        "                 measures ingest rate beyond the rotation speed\n"
        "  --verify       memcmp bank-vs-shadow after every frame (slow; test)\n"
        "  --crc          log crc32 of every decoded frame (slow; test)\n",
        argv0, PVS_PORT, FRAME_PHYS_DEFAULT, REG_PHYS_DEFAULT);
}

int main(int argc, char **argv)
{
    int port = PVS_PORT;
    uint32_t frame_phys = FRAME_PHYS_DEFAULT;
    uint32_t reg_phys = REG_PHYS_DEFAULT;
    double fake_rps = 0.0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--base") && i + 1 < argc)
            frame_phys = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--regs") && i + 1 < argc)
            reg_phys = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--fake") && i + 1 < argc)
            fake_rps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--bench"))  g_bench = 1;
        else if (!strcmp(argv[i], "--verify")) g_verify = 1;
        else if (!strcmp(argv[i], "--crc"))    g_crc = 1;
        else { usage(argv[0]); return 2; }
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    struct sigaction sa = { .sa_handler = on_sig };  /* no SA_RESTART: EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (hw_init(reg_phys, frame_phys) != 0) return 1;

    logts("pov_rxd v2: banks A=0x%08x B=0x%08x (%u B each), regs=0x%08x%s%s%s",
          g_bank_phys[0], g_bank_phys[1], (unsigned)BANK_BYTES, reg_phys,
          g_bench ? " [bench]" : "", g_verify ? " [verify]" : "",
          g_crc ? " [crc]" : "");
    logts("engine STATUS=0x%08x POV_CTRL=0x%08x",
          reg_rd(REG_STATUS), reg_rd(REG_POV_CTRL));

    /* start on bank A; POV_CTRL is left alone unless --fake */
    reg_wr(REG_SLICE_BASE, g_bank_phys[0]);
    if (fake_rps > 0.0) {
        uint32_t period = (uint32_t)((double)ACLK_HZ / (fake_rps * PVS_N_SLICES) + 0.5);
        reg_wr(REG_FAKE_PERIOD, period);
        reg_wr(REG_POV_CTRL, ((uint32_t)PVS_N_SLICES << 16) | (1u << 1) | 1u);
        logts("fake-spin: %.2f rps -> fake_period=%u ticks/slice, POV_CTRL set",
              fake_rps, period);
    }

    uint8_t *cbuf    = malloc(COMP_LEN_MAX);
    uint8_t *staging = malloc(PVS_FRAME_RAW);   /* delta-zlib mask scratch */
    g_shadow         = malloc(PVS_FRAME_RAW);   /* cached current frame */
    if (!cbuf || !staging || !g_shadow) { perror("malloc"); return 1; }
    memset(g_shadow, 0, PVS_FRAME_RAW);
    memset(g_dirty_accum, 1, sizeof g_dirty_accum);  /* banks = garbage */

    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr = { .sin_family = AF_INET,
                                .sin_addr.s_addr = htonl(INADDR_ANY),
                                .sin_port = htons((uint16_t)port) };
    if (bind(lfd, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("bind"); return 1; }
    if (listen(lfd, 1) < 0) { perror("listen"); return 1; }
    logts("listening on :%d", port);

    while (!g_stop) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof peer;
        int cfd = accept(lfd, (struct sockaddr *)&peer, &plen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            break;
        }
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        logts("client %s:%d connected", inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
        serve_client(cfd, cbuf, staging);
        close(cfd);
        logts("client disconnected");
    }

    close(lfd);
    logts("exiting (signal)");
    return 0;
}
