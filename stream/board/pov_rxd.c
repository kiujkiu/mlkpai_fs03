/*
 * pov_rxd.c - board-side receiver daemon for the PVS1 POV frame stream. (v2)
 *
 * Target: Zynq-7020 (MLKPAI-FS03), ARM Cortex-A9, Debian buster userspace,
 * kernel 6.6. Build static with arm-linux-gnueabihf-gcc (see Makefile).
 *
 * Protocol: stream/protocol.h + stream/pc/protocol.md (PVS1 + DELTA flag).
 * TCP server on :9500; per frame: 16B header | payload; reply 1 ACK byte.
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
 * v2 (26 页/秒方案, docs/design_icnd2047/04_sw_stream_26fps.md §3):
 *   - 三缓冲 + 双线程: RX 线程 (recv + inflate + delta 重建 + 立即 ACK) 与
 *     flip 线程 (等翻页窗 -> memcpy 最新就绪缓冲进空闲 DDR bank -> 翻页)
 *     解耦。ACK 节拍 = 解码吞吐, 不再被翻页窗拖住 (解耦前封顶 13 fps)。
 *     三个 cached staging 缓冲 (写入/就绪/拷贝) 用 mutex + 代数计数轮转,
 *     RX 发布时若旧 ready 未被消费则直接顶替 = 天然丢帧策略 (最新帧赢)。
 *   - DELTA 帧: raw = prev_acked_raw ^ decoded (protocol.h flag bit2)。
 *     参考帧就是上一帧解码所在的 staging 缓冲 (cached, 不从 uncached bank
 *     读回); 缓冲轮转保证 RX 下一个写入缓冲永远不是参考帧, flip 线程对
 *     参考帧只读, 并发安全。连接建立后首帧带 DELTA -> NAK (无参考帧)。
 *   - 翻页窗: 默认单窗 slice<8 (每圈一翻); --flip-window dual 加
 *     |slice-180|<8 半圈双窗 (双屏对置)。同窗防双触发: 翻完必须先观察到
 *     离开该窗才允许再翻。
 *   - crc32 移到 --crc 选项后 (省 11-18 ms/帧), 联调开量产关。
 *   - 帧区映射优先走 /dev/povmem (povmem.ko, Write-Combine, memcpy 实效
 *     300-800 MB/s), 不在则回退 /dev/mem (Strongly-Ordered, 60-120 MB/s)。
 *     寄存器页永远走 /dev/mem (寄存器就该 SO)。
 *
 * Cache coherency: on 32-bit ARM, kernel 6.6 arch/arm/mm/mmu.c
 * phys_mem_access_prot() returns pgprot_noncached() for any pfn where
 * !pfn_valid(pfn). With mem=256M the frame region (and the PL register
 * page) are not kernel-managed RAM, so the /dev/mem mmap is uncached /
 * strongly-ordered; the povmem.ko mmap is Normal-NC (write-combine). In
 * both cases nothing is stuck in L1/L2 and the PL HP-port sees the data
 * with no cache maintenance. WC is weakly ordered though, so after the
 * bank memcpy we issue a DSB (not just DMB) to drain the write buffer
 * before the slice_base register write can take effect.
 *
 * Display-never-blocks policy: the PL engine keeps refetching whatever
 * slice_base points at; the flip thread only redirects it inside a flip
 * window. If the sender outruns the display, RX keeps ACKing and the
 * ready buffer is overwritten in place - the newest frame wins.
 *
 * Build modes:
 *   default          real /dev/mem (+ optional /dev/povmem) (ARM board)
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
#include <pthread.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <zlib.h>

#include "../protocol.h"

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

#define POVMEM_DEV         "/dev/povmem"        /* povmem.ko WC window */
#define POVMEM_PHYS_BASE   0x10000000u          /* povmem.ko `base` param */

#define ACLK_HZ            50000000u
#define SLICE_WRAP_THRESH  8                    /* window half-width */
#define WIN_DUAL_CENTER    180                  /* second window @ slice 180 */
#define FLIP_TIMEOUT_MS    2000                 /* engine idle? flip anyway */
#define COMP_LEN_MAX       (PVS_FRAME_RAW + 0x10000u)
#define PVS_FLAGS_KNOWN    (PVS_FLAG_RLE | PVS_FLAG_ZLIB | PVS_FLAG_DELTA)

/* ---- logging ------------------------------------------------------------ */
/* 单次 fputs 整行输出: RX/flip 两个线程并发打日志, 拼好再写才不串行 */
static void logts(const char *fmt, ...)
{
    struct timespec ts;
    struct tm tm;
    va_list ap;
    char line[512];
    size_t n;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    n = (size_t)snprintf(line, sizeof line, "[%02d:%02d:%02d.%03ld] ",
                         tm.tm_hour, tm.tm_min, tm.tm_sec,
                         ts.tv_nsec / 1000000L);
    va_start(ap, fmt);
    n += (size_t)vsnprintf(line + n, sizeof line - n - 2, fmt, ap);
    va_end(ap);
    if (n > sizeof line - 2) n = sizeof line - 2;
    line[n] = '\n'; line[n + 1] = '\0';
    fputs(line, stdout);
    fflush(stdout);
}

static long mono_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}

/* ---- register + bank access (real vs sim) ------------------------------- */
static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) { (void)sig; g_stop = 1; }

static uint8_t  *g_bank[2];      /* virtual addresses of bank A/B */
static uint32_t  g_bank_phys[2]; /* physical addresses (what 0x18 wants) */
static int       g_frame_wc = 0; /* 1 = frame map is write-combine (povmem) */

/* DSB: WC 是弱序, 翻页前必须排空 write buffer; SO 路径 DMB 也够, 统一用最强 */
static inline void wmb_frame(void)
{
#if defined(__arm__)
    __asm__ volatile("dsb sy" ::: "memory");
#else
    __sync_synchronize();
#endif
}

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

    /* frame region: try the WC window first (povmem.ko), fall back to the
     * old strongly-ordered /dev/mem path if the module isn't loaded */
    void *f = MAP_FAILED;
    if (frame_phys >= POVMEM_PHYS_BASE) {
        int pfd = open(POVMEM_DEV, O_RDWR | O_SYNC);
        if (pfd >= 0) {
            f = mmap(NULL, FRAME_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                     pfd, frame_phys - POVMEM_PHYS_BASE);
            if (f == MAP_FAILED)
                perror("mmap " POVMEM_DEV " (falling back to /dev/mem)");
            else
                g_frame_wc = 1;
            close(pfd);
        }
    }
    if (f == MAP_FAILED) {
        f = mmap(NULL, FRAME_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                 fd, frame_phys);
        if (f == MAP_FAILED) { perror("mmap frame region"); return -1; }
    }
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
        uint32_t s = __sync_fetch_and_add(&g_sim_slice, 23) + 23;
        return 0x80000000u | (s % PVS_N_SLICES);
    }
    return g_sim_regs[off / 4];
}

static void reg_wr(uint32_t off, uint32_t v)
{
    g_sim_regs[off / 4] = v;
    logts("SIM: reg[0x%02x] <= 0x%08x", off, v);
}

#endif /* SIM_NO_DEVMEM */

/* ---- triple-buffer staging + thread handoff ------------------------------
 * 三个 cached staging 缓冲:
 *   g_wr    RX 线程正在解码写入 (仅 RX 访问)
 *   g_ready 最新就绪帧 (mutex 保护的交接槽, 代数计数标新旧)
 *   g_disp  flip 线程持有 (memcpy 进 DDR bank 的源, 仅 flip 访问)
 * RX 发布 = swap(g_wr, g_ready) + gen++; flip 消费 = swap(g_ready, g_disp)。
 * DELTA 参考帧 g_prev 指向"最后 ACK 的 raw 帧"所在缓冲: 发布后它在 ready
 * 槽, 被 flip 消费后在 disp 槽 —— 两处都没人写 (flip 只读), RX 下一个拿到
 * 的写入缓冲永远是第三块, 所以参考帧内容在下一次发布前始终有效。
 */
static uint8_t *g_wr, *g_ready, *g_disp;
static unsigned g_ready_gen, g_consumed_gen;
static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;

static uint8_t *g_prev;          /* DELTA 参考帧 (仅 RX 线程读写指针) */
static int      g_prev_valid;    /* 连接内是否已有 ACK 过的帧 */

static int g_crc_on   = 0;       /* --crc: 每帧算 crc32 (联调用, 量产关) */
static int g_win_dual = 0;       /* --flip-window dual: 半圈双窗 */

/* stats (RX 线程写, flip 线程读, 32-bit 对齐字, 统计精度要求低) */
static unsigned g_st_rx, g_st_flip, g_st_drop, g_st_forced;
static unsigned long g_st_dec_us;

/* ---- socket helpers ------------------------------------------------------ */
static int recv_full(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len) {
        ssize_t n = recv(fd, p, len, 0);
        if (n == 0) return 0;                       /* peer closed */
        if (n < 0) {
            if (errno == EINTR) { if (g_stop) return -1; continue; }
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                logts("recv timeout: dropping stale client (ghost guard)");
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

/* ---- decompression -------------------------------------------------------
 * Zero-run RLE (protocol.md): 0x00 escape byte followed by run:u16le emits
 * that many zero bytes; any other byte is a literal. Bare 0x00 never occurs.
 */
static int rle_decode(const uint8_t *src, size_t slen, uint8_t *dst, size_t dlen)
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

/* DELTA 重建: dst = dst ^ src, 32-bit 字循环 (-mfpu=neon 下 gcc 自动向量化,
 * A9 实测预算 ~8-15 ms/4.4MB)。缓冲都是 malloc 的, 4 字节对齐成立。 */
static void xor_frame(uint8_t *dst, const uint8_t *src, size_t n)
{
    uint32_t *d = (uint32_t *)dst;
    const uint32_t *s = (const uint32_t *)src;
    for (size_t i = 0; i < n / 4; i++)
        d[i] ^= s[i];
}

/* ---- flip thread ----------------------------------------------------------
 * 消费最新就绪缓冲: 先 memcpy 进空闲 bank (WC 6-15 ms / SO 37-73 ms, 必须
 * 在窗口开启前做完 —— 13 rps 下窗口本身只有 ~1.7 ms), 再等翻页窗写
 * slice_base。同窗防双触发: 记录上次翻页的窗口 id, 必须先观察到离开
 * (slice 出窗或换窗) 才允许再翻。显示永不阻塞: 没新帧就重扫旧帧。
 */
static int slice_window(uint32_t slice)
{
    if (slice < SLICE_WRAP_THRESH) return 0;
    if (g_win_dual) {
        int d = (int)slice - WIN_DUAL_CENTER;
        if (d < 0) d = -d;
        if (d < SLICE_WRAP_THRESH) return 1;
    }
    return -1;
}

static void *flip_thread(void *arg)
{
    (void)arg;
    int active = 0;              /* 当前显示 bank (main 启动时已指向 bank A) */
    int last_flip_win = -1;      /* 上次翻页所在窗口 id (-1 = 无约束) */
    long stat_t0 = mono_ms();
    unsigned stat_rx0 = 0, stat_flip0 = 0;

    while (!g_stop) {
        /* 每秒一行统计 (26 fps 下逐帧 printf 走串口是可观开销) */
        long now = mono_ms();
        if (now - stat_t0 >= 1000) {
            unsigned rx = g_st_rx, fl = g_st_flip;
            if (rx != stat_rx0 || fl != stat_flip0)
                logts("STAT rx=%u flip=%u drop=%u forced=%u dec_avg=%.1fms",
                      rx, fl, g_st_drop, g_st_forced,
                      rx ? (double)g_st_dec_us / rx / 1000.0 : 0.0);
            stat_rx0 = rx; stat_flip0 = fl; stat_t0 = now;
        }

        /* 取最新就绪缓冲 (代数计数; 没有新帧就小睡重试) */
        int fresh = 0;
        pthread_mutex_lock(&g_mu);
        if (g_ready_gen != g_consumed_gen) {
            uint8_t *t = g_disp; g_disp = g_ready; g_ready = t;
            g_consumed_gen = g_ready_gen;
            fresh = 1;
        }
        pthread_mutex_unlock(&g_mu);
        if (!fresh) { usleep(500); continue; }

        /* 拷贝进空闲 bank, DSB 排空 write buffer 后引擎才可能取到 */
        int idle = active ^ 1;
        memcpy(g_bank[idle], g_disp, PVS_FRAME_RAW);
        wmb_frame();

        /* 等翻页窗 (先离开上次翻页的窗口, 再命中任一窗口) */
        long t0 = mono_ms();
        int need_leave = (last_flip_win >= 0);
        int win = -1, forced = 0;
        for (;;) {
            uint32_t slice = reg_rd(REG_POV_CTRL) & 0xffffu;
            win = slice_window(slice);
            if (need_leave && win != last_flip_win)
                need_leave = 0;
            if (!need_leave && win >= 0)
                break;
            if (mono_ms() - t0 > FLIP_TIMEOUT_MS) {
                logts("WARN: no flip window in %d ms (engine idle?), flipping anyway",
                      FLIP_TIMEOUT_MS);
                forced = 1;
                break;
            }
            if (g_stop) return NULL;
            usleep(200);
        }

        wmb_frame();                       /* frame data globally visible ... */
        reg_wr(REG_SLICE_BASE, g_bank_phys[idle]);
        wmb_frame();                       /* ... before + after base update  */
        active = idle;
        last_flip_win = forced ? -1 : win;
        g_st_flip++;
        if (forced) g_st_forced++;
        logts("FLIP gen=%u bank=%d win=%d%s", g_consumed_gen, idle, win,
              forced ? " FORCED" : "");
    }
    return NULL;
}

/* ---- per-connection frame loop (RX 线程 = 主线程) ------------------------ */
static int serve_client(int fd, uint8_t *cbuf)
{
    unsigned seq = 0;
    g_prev_valid = 0;    /* DELTA 参考帧按连接失效: 重连后首帧必须 keyframe */

    for (;;) {
        pvs_hdr_t h;
        int r = recv_full(fd, &h, sizeof h);
        if (r <= 0) return r;

        /* 头校验: 未知 flag 用位掩码 (给未来留位), RLE+ZLIB 互斥 */
        if (memcmp(h.magic, PVS_MAGIC, 4) != 0 ||
            h.raw_len  != PVS_FRAME_RAW        ||
            h.n_slices != PVS_N_SLICES         ||
            h.comp_len == 0 || h.comp_len > COMP_LEN_MAX ||
            (h.flags & ~PVS_FLAGS_KNOWN) != 0  ||
            ((h.flags & PVS_FLAG_RLE) && (h.flags & PVS_FLAG_ZLIB))) {
            logts("NAK: bad header (magic=%.4s comp=%u raw=%u n=%u flags=0x%x)",
                  h.magic, h.comp_len, h.raw_len, h.n_slices, h.flags);
            send_byte(fd, PVS_NAK);
            return -1;
        }
        /* DELTA 无参考帧 (连接首帧/重连后) -> NAK, 发送端降级重发关键帧 */
        if ((h.flags & PVS_FLAG_DELTA) && !g_prev_valid) {
            logts("NAK: DELTA frame with no reference (need keyframe first)");
            send_byte(fd, PVS_NAK);
            return -1;
        }

        /* receive payload; uncompressed frames go straight into g_wr */
        int comp = h.flags & (PVS_FLAG_RLE | PVS_FLAG_ZLIB);
        if (!comp && h.comp_len != h.raw_len) {
            logts("NAK: raw frame but comp_len %u != raw_len", h.comp_len);
            send_byte(fd, PVS_NAK);
            return -1;
        }
        r = recv_full(fd, comp ? cbuf : g_wr, h.comp_len);
        if (r <= 0) return r;

        long dec_t0 = mono_ms();

        /* decode into g_wr */
        if (h.flags & PVS_FLAG_ZLIB) {
            uLongf dlen = PVS_FRAME_RAW;
            int zr = uncompress(g_wr, &dlen, cbuf, h.comp_len);
            if (zr != Z_OK || dlen != PVS_FRAME_RAW) {
                logts("NAK: zlib inflate failed (rc=%d dlen=%lu)", zr, (unsigned long)dlen);
                send_byte(fd, PVS_NAK);
                return -1;
            }
        } else if (h.flags & PVS_FLAG_RLE) {
            if (rle_decode(cbuf, h.comp_len, g_wr, PVS_FRAME_RAW) != 0) {
                logts("NAK: RLE decode failed");
                send_byte(fd, PVS_NAK);
                return -1;
            }
        } /* else raw: already in g_wr */

        /* DELTA 重建: raw = prev_acked_raw ^ decoded (原地 XOR) */
        if (h.flags & PVS_FLAG_DELTA)
            xor_frame(g_wr, g_prev, PVS_FRAME_RAW);

        uint32_t crc = 0;
        if (g_crc_on)
            crc = crc32(0L, g_wr, PVS_FRAME_RAW);

        long dec_ms = mono_ms() - dec_t0;

        /* 发布给 flip 线程 + 记参考帧; 旧 ready 没被消费就顶替 (丢帧计数) */
        g_prev = g_wr;
        g_prev_valid = 1;
        pthread_mutex_lock(&g_mu);
        if (g_ready_gen != g_consumed_gen) g_st_drop++;
        uint8_t *t = g_wr; g_wr = g_ready; g_ready = t;
        g_ready_gen++;
        pthread_mutex_unlock(&g_mu);
        g_st_rx++;
        g_st_dec_us += (unsigned long)dec_ms * 1000;

        /* ACK 立即发 (与翻页解耦, ACK 节拍 = 解码吞吐) */
        if (send_byte(fd, PVS_ACK) != 0) return -1;

        if (g_crc_on)
            logts("FRAME seq=%u comp=%u flags=0x%x crc=%08x dec=%ldms",
                  seq++, h.comp_len, h.flags, crc, dec_ms);
        else
            logts("FRAME seq=%u comp=%u flags=0x%x dec=%ldms",
                  seq++, h.comp_len, h.flags, dec_ms);
        if (g_stop) return -1;
    }
}

/* ---- main ---------------------------------------------------------------- */
static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [--port N] [--base HEXADDR] [--regs HEXADDR] [--fake RPS]\n"
        "          [--crc] [--flip-window single|dual]\n"
        "  --port N       TCP listen port (default %d)\n"
        "  --base ADDR    frame region phys base (default 0x%08x)\n"
        "  --regs ADDR    POV engine AXI base    (default 0x%08x)\n"
        "  --fake RPS     enable motor-less fake-spin at RPS revs/sec\n"
        "                 (programs fake_period + POV_CTRL; otherwise the\n"
        "                  daemon never touches POV_CTRL - JTAG owns it)\n"
        "  --crc          crc32 every decoded frame + log it (costs 11-18 ms\n"
        "                 per frame on the A9; debug only, default off)\n"
        "  --flip-window  single = flip near slice 0 only (default);\n"
        "                 dual   = also near slice 180 (dual-panel, 26 pps)\n",
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
        else if (!strcmp(argv[i], "--crc"))
            g_crc_on = 1;
        else if (!strcmp(argv[i], "--flip-window") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "dual")) g_win_dual = 1;
            else if (strcmp(m, "single")) { usage(argv[0]); return 2; }
        }
        else { usage(argv[0]); return 2; }
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    struct sigaction sa = { .sa_handler = on_sig };  /* no SA_RESTART: EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (hw_init(reg_phys, frame_phys) != 0) return 1;

    logts("pov_rxd v2: banks A=0x%08x B=0x%08x (%u B each), regs=0x%08x",
          g_bank_phys[0], g_bank_phys[1], (unsigned)BANK_BYTES, reg_phys);
    logts("frame map: %s, crc=%s, flip-window=%s",
          g_frame_wc ? "WC via " POVMEM_DEV : "SO via /dev/mem",
          g_crc_on ? "on" : "off", g_win_dual ? "dual" : "single");
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

    uint8_t *cbuf = malloc(COMP_LEN_MAX);
    g_wr    = malloc(PVS_FRAME_RAW);   /* 三缓冲: cached staging (见上) */
    g_ready = malloc(PVS_FRAME_RAW);
    g_disp  = malloc(PVS_FRAME_RAW);
    if (!cbuf || !g_wr || !g_ready || !g_disp) { perror("malloc"); return 1; }

    pthread_t flip_tid;
    if (pthread_create(&flip_tid, NULL, flip_thread, NULL) != 0) {
        perror("pthread_create flip");
        return 1;
    }

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
        /* ghost guard: a killed WSL sender leaves this socket ESTAB forever
         * (FIN/RST never reaches us) and the single client slot deadlocks.
         * Keepalive detects a dead peer in ~10+3*3 s; the recv/send timeouts
         * cover the black-hole case where probes are silently eaten. */
        setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
        int ka = 10; setsockopt(cfd, IPPROTO_TCP, TCP_KEEPIDLE,  &ka, sizeof ka);
        ka = 3;      setsockopt(cfd, IPPROTO_TCP, TCP_KEEPINTVL, &ka, sizeof ka);
        ka = 3;      setsockopt(cfd, IPPROTO_TCP, TCP_KEEPCNT,   &ka, sizeof ka);
        int rbuf = 512 * 1024;   /* 5G 突发吸收 (§3.5-9) */
        setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof rbuf);
        struct timeval rto = { .tv_sec = 30, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof rto);
        rto.tv_sec = 10;
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &rto, sizeof rto);
        logts("client %s:%d connected", inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
        serve_client(cfd, cbuf);
        close(cfd);
        logts("client disconnected");
    }

    close(lfd);
    pthread_join(flip_tid, NULL);
    logts("STAT rx=%u flip=%u drop=%u forced=%u (final)",
          g_st_rx, g_st_flip, g_st_drop, g_st_forced);
    logts("exiting (signal)");
    return 0;
}
