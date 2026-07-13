/*
 * bench_s0.c - S0 板端微基准 (docs/design_icnd2047/04_sw_stream_26fps.md §5)。
 *
 * 26 页/秒 CPU 账的 5 个真实数, 各跑 N 轮 (默认 10) 取中位数, 一行一个 ms:
 *   so_memcpy_ms   4.4 MB cached -> /dev/mem SO 映射 (现路径, 估 37-73 ms)
 *   wc_memcpy_ms   4.4 MB cached -> /dev/povmem WC 映射 (povmem.ko 在才测)
 *   inflate_ms     130 KB 级典型帧 zlib inflate -> 4.4 MB (决定要不要后手)
 *   crc32_ms       4.4 MB crc32 (--crc 选项的代价)
 *   xor_ms         4.4 MB XOR (DELTA 重建的代价)
 *
 * 目标 bank 用 bank B 偏移 (base+0x500000), 避开正在显示的 bank A ——
 * 但引擎在跑时照样别跑 bench (会抢 DDR 带宽, 数不准)。
 *
 * 板上 (静态 ARM 版): sudo ./bench_s0
 * x86 版 (bench_s0_x86) 只测 inflate/crc32/xor, mmap 两项自动 skip。
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <zlib.h>

#include "../protocol.h"

#define BANK_OFF   0x00500000u   /* 测 bank B, 别踩显示中的 bank A */
#define MAX_LOOPS  99

static double now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

static int cmp_d(const void *a, const void *b)
{
    double d = *(const double *)a - *(const double *)b;
    return d < 0 ? -1 : d > 0 ? 1 : 0;
}

static double median(double *v, int n)
{
    qsort(v, n, sizeof *v, cmp_d);
    return n & 1 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
}

/* 典型帧: 大片零 + 稀疏点亮 (跟 test_local gen_frame 同款, zlib-6 后
 * ~100-130 KB, 贴近角色类实测线上体积) */
static void gen_frame(uint8_t *f, uint32_t seed)
{
    memset(f, 0, PVS_FRAME_RAW);
    uint32_t s = seed * 2654435761u + 1;
    for (int i = 0; i < 40000; i++) {
        s = s * 1664525u + 1013904223u;
        uint32_t pos = (s >> 8) % PVS_FRAME_RAW;
        uint8_t val = (uint8_t)(s & 0xff);
        f[pos] = val ? val : 0x5a;
    }
}

static void xor_frame(uint8_t *dst, const uint8_t *src, size_t n)
{
    uint32_t *d = (uint32_t *)dst;
    const uint32_t *s = (const uint32_t *)src;
    for (size_t i = 0; i < n / 4; i++)
        d[i] ^= s[i];
}

/* mmap 一段物理帧区; dev = "/dev/mem" (off=绝对物理) 或 "/dev/povmem"
 * (off=相对模块 base)。失败返回 NULL (调用方打 skip)。 */
static uint8_t *map_phys(const char *dev, off_t off, size_t len)
{
    int fd = open(dev, O_RDWR | O_SYNC);
    if (fd < 0) return NULL;
    void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
    close(fd);
    return p == MAP_FAILED ? NULL : (uint8_t *)p;
}

int main(int argc, char **argv)
{
    uint32_t base = 0x10000000u;
    int loops = 10;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--base") && i + 1 < argc)
            base = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--loops") && i + 1 < argc)
            loops = atoi(argv[++i]);
        else {
            fprintf(stderr, "usage: %s [--base HEXADDR] [--loops N]\n", argv[0]);
            return 2;
        }
    }
    if (loops < 1 || loops > MAX_LOOPS) loops = 10;

    uint8_t *src = malloc(PVS_FRAME_RAW);
    uint8_t *dst = malloc(PVS_FRAME_RAW);
    uint8_t *cbuf = malloc(compressBound(PVS_FRAME_RAW));
    if (!src || !dst || !cbuf) { perror("malloc"); return 1; }
    gen_frame(src, 42);

    double t[MAX_LOOPS];
    fprintf(stderr, "bench_s0: %d loops each, frame=%u B, bank off=0x%08x\n",
            loops, (unsigned)PVS_FRAME_RAW, base + BANK_OFF);

    /* 1. SO memcpy: cached src -> /dev/mem 映射 (现 pov_rxd 无 povmem 路径) */
    uint8_t *so = map_phys("/dev/mem", (off_t)base + BANK_OFF, PVS_FRAME_RAW);
    if (so) {
        for (int i = 0; i < loops; i++) {
            double t0 = now_ms();
            memcpy(so, src, PVS_FRAME_RAW);
            __sync_synchronize();
            t[i] = now_ms() - t0;
        }
        printf("so_memcpy_ms %.2f\n", median(t, loops));
        munmap(so, PVS_FRAME_RAW);
    } else
        printf("so_memcpy_ms skip\n");

    /* 2. WC memcpy: cached src -> /dev/povmem 映射 (povmem.ko 装了才有) */
    uint8_t *wc = map_phys("/dev/povmem", BANK_OFF, PVS_FRAME_RAW);
    if (wc) {
        for (int i = 0; i < loops; i++) {
            double t0 = now_ms();
            memcpy(wc, src, PVS_FRAME_RAW);
#if defined(__arm__)
            __asm__ volatile("dsb sy" ::: "memory");  /* 排空 WC 才算写完 */
#else
            __sync_synchronize();
#endif
            t[i] = now_ms() - t0;
        }
        printf("wc_memcpy_ms %.2f\n", median(t, loops));
        munmap(wc, PVS_FRAME_RAW);
    } else
        printf("wc_memcpy_ms skip\n");

    /* 3. inflate: 典型帧 zlib-6 压一次, 反复解压计时 */
    uLongf clen = compressBound(PVS_FRAME_RAW);
    if (compress2(cbuf, &clen, src, PVS_FRAME_RAW, 6) != Z_OK) {
        fprintf(stderr, "compress2 failed\n");
        return 1;
    }
    fprintf(stderr, "bench_s0: typical frame compressed to %lu B\n",
            (unsigned long)clen);
    for (int i = 0; i < loops; i++) {
        uLongf dlen = PVS_FRAME_RAW;
        double t0 = now_ms();
        if (uncompress(dst, &dlen, cbuf, clen) != Z_OK || dlen != PVS_FRAME_RAW) {
            fprintf(stderr, "uncompress failed\n");
            return 1;
        }
        t[i] = now_ms() - t0;
    }
    printf("inflate_ms %.2f\n", median(t, loops));

    /* 4. crc32 (--crc 选项每帧的代价) */
    for (int i = 0; i < loops; i++) {
        double t0 = now_ms();
        uint32_t c = crc32(0L, src, PVS_FRAME_RAW);
        t[i] = now_ms() - t0;
        if (c == 0xdeadbeefu) putchar(0);   /* 防被优化掉 */
    }
    printf("crc32_ms %.2f\n", median(t, loops));

    /* 5. XOR (DELTA 重建, cached-cached) */
    for (int i = 0; i < loops; i++) {
        double t0 = now_ms();
        xor_frame(dst, src, PVS_FRAME_RAW);
        t[i] = now_ms() - t0;
    }
    printf("xor_ms %.2f\n", median(t, loops));

    return 0;
}
