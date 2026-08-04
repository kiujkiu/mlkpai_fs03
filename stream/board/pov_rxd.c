/*
 * pov_rxd.c - board-side receiver daemon for the PVS1 POV frame stream. (v2)
 *
 * Target: Zynq-7020 (MLKPAI-FS03), ARM Cortex-A9, Debian buster userspace,
 * kernel 6.6. Build static with arm-linux-gnueabihf-gcc (see Makefile).
 *
 * Protocol: stream/protocol.h + stream/pc/protocol.md (PVS1 + DELTA flag).
 * TCP server on :9500; per frame: 16B header | payload; reply 1 ACK byte.
 *
 * Hardware contract (verified PL POV engine @ 0x40010000, RTL 定稿版):
 *   0x00 R  STATUS       engine health
 *                        [15]=fold_a_en 回读 (确认 0x10[6] 写进去了)
 *                        [16]=base_b_act 回读 (= slice_base_b != 0)
 *   0x10 W  POV_CTRL     n_slices<<16 | fold_a_en<<6 | fake_en<<1 | pov_en
 *        R               bit31=locked, [15:0]=current slice_idx  (= 只写寄存器,
 *                        读回来的不是写进去的值 -> 软件必须自己留影子)
 *        (n_slices here = 引擎每圈的片数 = 360, 与帧里的 hdr.n_slices 无关:
 *         双面帧 720 片是「两面各 360」, 引擎一圈仍然是 360 片)
 *   0x14 W  fake_period  aclk ticks/slice @ 50 MHz (R: rev_period)
 *   0x18 W  slice_base   面A 的 DDR 帧基址; latched per-slice at fetch_go
 *                        -> takes effect from the next slice.
 *   0x1C W  PHASE_B[8:0] 屏B 的 slice 偏移, RTL 复位默认 180 (老的「屏B ≡
 *                        屏A+180」共享数据玩法)。读 0x1C 给的是实时 idx_B,
 *                        不是写进去的值 -> 同样必须留影子。
 *   0x24 R  POV_CTRL 影子回读 (位序与 0x10 写口逐位对齐)。有了它, fold_a_en
 *                        就能安全 RMW, 不必跟 JTAG 抢所有权:
 *                        ctrl = rd(0x24); wr(0x10, ctrl|0x40) / (ctrl&~0x40)。
 *                        (R 0x28 是留给 frame_period_o 的, 不要读。)
 *        !! 硬约束 PHASE_B < n_slices: RTL 的 idx_b_live 只做「一次条件减」
 *           (idx+PHASE_B >= n_slices 就减一次 n_slices), 不是取模。PHASE_B
 *           >= n_slices 时索引直接越界 -> 屏B 读到野地址 -> 花屏。
 *           所以: DUAL_FACE (两面各有独立数据) 一律把 0x1C 写 0; 任何时候
 *           写 POV_CTRL 的 n_slices 之前都要复检, 不满足就把 0x1C 钳到 0。
 *   0x28 W  slice_base_b 面B 的 DDR 帧基址 (v3.1 新增, 复位 0)。写 0 = 回落:
 *                        PL 两面都用 0x18 (= v3.0 之前的老行为)。单面帧本守护
 *                        进程写 0。
 *
 * ---- 帧区地址表 (v3.2) ---------------------------------------------------
 * Linux boots with mem=256M, so phys 0x10000000..0x1FFFFFFF (256 MB) is
 * invisible to the kernel and reserved for frames. 一帧最大已从 0x438000
 * (360 片) 涨到 PVS_FRAME_RAW_MAX = 0x870000 (720 片 = 8.85 MB), 老的
 * 5 MB bank stride 装不下, 且会撞上曾预留给三缓冲的 bank C@0x10A00000,
 * 所以整套重排到 16 MB 一格 (地址一眼可读, 每格留 7 MB 余量):
 *
 *   phys 起址    大小(槽)   用途                       实际用量
 *   0x10000000   16 MB      bank A = 翻页缓冲 0        0..0x870000
 *   0x11000000   16 MB      bank B = 翻页缓冲 1        0..0x870000
 *   0x12000000   16 MB      bank C = 翻页缓冲 2        0..0x870000
 *   0x13000000   208 MB     空闲 (下一块从这里开)      -
 *   (mmap 的窗口 = FRAME_MAP_LEN = bank A 起 .. bank C 末 = 0x2870000)
 *
 * v3.2 起 DDR 侧是**三缓冲**: 引擎正在扫 active, flip 线程往 idle 里灌下一帧,
 * 第三块是「刚灌完还没轮到」的余量 —— PL 的 base_lat 是 pair 级快照, 双 bank
 * 时最坏情况下刚写完的那块下一轮就要被覆盖, 三块把这个窗口拉开一整轮。
 *
 * bank 内布局 = 解压后载荷原样 (面 A/B 连续), flip 时按面拆基址:
 *   单面 (n_slices 片):  [bank+0 , bank+n_slices*0x3000)
 *                        0x18 <= bank        0x28 <= 0
 *   PVS_FLAG_DUAL_FACE:  面A [bank+0 , bank+nA*0x3000)
 *                        面B [bank+nA*0x3000 , bank+n_slices*0x3000)
 *                        0x18 <= bank        0x28 <= bank + nA*0x3000
 *   nA = PVS_FLAG_FOLD_A ? 180 : 360 -> 面B 偏移 0x21C000 / 0x438000。
 *   PHASE_B / SLICE_BASE_B / (fold_a_en) / SLICE_BASE 四个寄存器在同一个翻页
 *   窗内背靠背写, RTL 在 idx 变化那一拍同时快照 base_lat/base_lat_b, 所以两面
 *   永远来自同一帧 (不撕裂)。
 *
 * 注意 FOLD_A 省的是 **DDR 占用 + 链路带宽 + 上位机渲染量**, 不省 PL 侧的
 * DDR 读带宽: 每个 slice_idx 照样整片 fetch, 后半圈只是重复读前半圈那块
 * 地址 (再做镜像置换)。上面这张表里的「实际用量」是占用量, 不是读带宽。
 *
 * !! povmem.ko size: v3.2 三缓冲后 **FRAME_MAP_LEN = 0x2870000 = 42401792 B
 * (40.4 MiB)**。povmem.ko 的默认 size 在 2026-07-31 从 16 MB 提到了 0x1900000
 * (25.6 MB) —— 对双缓冲够, 对三缓冲**又不够了**, 必须
 *     insmod povmem.ko base=0x10000000 size=0x2870000
 * (或把 .ko 默认再提到 0x2870000)。不够时 mmap 直接 -EINVAL, 自动回落到
 * /dev/mem 的 Strongly-Ordered 映射 (8.85 MB 要 74-148 ms, 会拖垮帧率) ——
 * 启动日志会把所需的最小值原样打出来。程序启动时无条件打印一行
 * "povmem needs size>=0x%x", 照抄即可。
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
 *   v3.1 (偏心屏, protocol.h 的 n_slices 权威化):
 *   - 帧长度不再是编译期常量: 头里的 n_slices 是权威, raw_len 必须
 *     == n_slices*PVS_SLICE_STRIDE, n_slices ∈ [1, PVS_N_SLICES_MAX]。
 *     越界/不自洽一律 NAK (不再 assert/崩)。缓冲一律按 MAX 分配。
 *   - PVS_FLAG_DUAL_FACE: 载荷 = [面A][面B], 落到两个基址 (0x18/0x28), 见
 *     上面的地址表; PVS_FLAG_FOLD_A 让面A 只占 180 片。
 *   - DELTA 参考帧必须与当前帧等长 (几何切换必须发关键帧), 否则 NAK ——
 *     XOR 跨长度没有定义, 静默做会得到半帧垃圾。
 *   - 单面 360 路径逐字节不变: 同样一次 memcpy 整帧到 bank, 0x18 同值。
 *   v3.2 (双面定案 + 解码并行化):
 *   - DUAL_FACE 载荷 = **两条独立压缩流**:
 *       [u32 LE comp_len_A][面A 流][面B 流]   (comp_len 含这 4 字节)
 *       面B 流长度 = comp_len - 4 - comp_len_A
 *     单面帧排布完全不变 (没有前缀)。DELTA 时各面各自 XOR 上一帧的同面数据
 *     —— 参考帧是同一个 staging 缓冲, 面偏移一致, 所以逐面 XOR 天然成立
 *     (前提: 参考帧与本帧**布局相同**, 不只是等长 —— 见 g_prev_face_b_off)。
 *   - 两面并行解压: 两个常驻工作线程, 分别 pthread_setaffinity_np 到
 *     CPU0/CPU1。A9 单核 zlib 实测 ~69 MB/s, 双面 540 片 = 6.64 MB 单核 96 ms,
 *     两核 ~48 ms。两个 job 只碰各自那半个 staging 缓冲 (面边界 nA*0x3000
 *     天然 4 KB 对齐, 不会伪共享), 只读各自那半个压缩缓冲 / 参考帧, zlib 的
 *     uncompress() 无全局状态 —— 没有跨核共享写。
 *     ⚠ 历史包袱: 裸机时代 ENABLE_DUAL_CORE=1 出过 UART desync。现在是 Linux
 *     用户态线程, 机理无关, 但保留 `--decode serial` 一键退回单核。
 *   - DDR 侧从双 bank 扩到**三 bank** (bank C 上线, 见上面的地址表)。
 *     ACK 与翻页的解耦 v2 就做了 (RX 解完立刻 ACK, 从不等翻页窗), v3.2 只是
 *     把 DDR 侧也补成三缓冲, 并在日志里把这件事显式化。
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
#include <sys/stat.h>
#include <sys/mman.h>
#include <stdarg.h>
#include <pthread.h>
#include <sched.h>
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
#define REG_PHASE_B        0x1C                 /* [8:0] 屏B slice 偏移 */
#define REG_POV_CTRL_RB    0x24                 /* R: POV_CTRL 影子回读 */
#define REG_SLICE_BASE_B   0x28                 /* W: 面B 基址; 0 = 用 0x18。
                                                 * R 0x28 是 frame_period_o 的
                                                 * 预留口, 不要读 */

#define CTRL_FOLD_A_EN     (1u << 6)            /* POV_CTRL[6] 面A 半圈折叠 */
#define STATUS_FOLD_A_EN   (1u << 15)           /* STATUS[15] fold_a_en 回读 */
#define STATUS_BASE_B_ACT  (1u << 16)           /* STATUS[16] slice_base_b != 0 */
#define PHASE_B_RESET      180u                 /* RTL 复位默认 (老共享数据玩法) */
/* DUAL_FACE 帧屏B 用的相位。**直觉上该是 0**(两面各有各的数据, idx 就是 idx),
 * 但 2026-08-03 真机实测必须是 180 —— 这是在补偿**渲染侧的一个约定错误**:
 *   面B 是从 +X 侧观察的, 相对面A 观察方向相反 ⇒ 它的垂距符号和左右手性**都要翻**。
 *   povstream 目前把面B 渲成「跟面A 同一套约定的 +13.4mm」, 两个都没翻。
 * 数值恒等式(已验证 6/6, 不加镜像 0/6):
 *   render(θ, −d, mir) ≡ mirror( render(θ+180, +d, mir) )
 * ⇒ 用 PHASE_B=180 且 mirror_b=0 显示, 效果正好等于「−d 且手性翻转」= 物理真相。
 * 🔴 若将来把渲染侧改成原生正确(面B 渲 −13.4mm 且 mirror_u 取反), **必须把这里改回 0**,
 *    否则两处补偿叠加又错回去。二选一, 不要都做。见 memory
 *    project_pov3d_v31_dualface_geometry_solved。 */
#define PHASE_B_DUAL       180u

/* 帧区布局 —— 完整地址表见文件头。16 MB 一格, 每格实际最多用 0x870000。 */
#define FRAME_PHYS_DEFAULT 0x10000000u
#define FRAME_REGION_BASE  0x10000000u          /* mem=256M 让出的保留区 */
#define FRAME_REGION_END   0x20000000u          /* (半开区间) */
#define BANK_STRIDE        0x01000000u          /* 16 MiB, bank 间距 */
#define BANK_BYTES         PVS_FRAME_RAW_MAX    /* 0x870000, 页整数倍 */
#define FRAME_BANKS        3                    /* v3.2: A/B/C 三缓冲翻页 */
/* 只映射到最后一个 bank 的末尾, 不白占后面的地址空间 = 0x2870000 (40.4 MiB) */
#define FRAME_MAP_LEN      ((FRAME_BANKS - 1) * BANK_STRIDE + BANK_BYTES)

#define POVMEM_DEV         "/dev/povmem"        /* povmem.ko WC window */
#define POVMEM_PHYS_BASE   0x10000000u          /* povmem.ko `base` param */
#define POVMEM_MIN_SIZE    FRAME_MAP_LEN        /* insmod povmem.ko size>=这个 */

#define ACLK_HZ            50000000u
#define SLICE_WRAP_THRESH  8                    /* window half-width */
#define WIN_DUAL_CENTER    180                  /* second window @ slice 180 */
#define FLIP_TIMEOUT_MS    2000                 /* engine idle? flip anyway */
#define COMP_LEN_MAX       (PVS_FRAME_RAW_MAX + 0x10000u)
#define DUAL_PFX_LEN       4u                   /* [u32 LE comp_len_A] */
#define DEC_WORKERS        2                    /* 面A / 面B 各一个核 */
/* PC 端 window=2 (最多 2 帧在途), 板端要能吸掉 ~1.5 帧压缩数据。双面实测
 * 约 300 KB/帧 -> 450 KB; 取 768 KB 留余量 (内核会 x2 记账)。 */
#define RCVBUF_BYTES       (768 * 1024)
#define RCVBUF_MIN_EFF     (450 * 1024)         /* 生效值低于此就告警 */
#define PVS_FLAGS_KNOWN    (PVS_FLAG_RLE | PVS_FLAG_ZLIB | PVS_FLAG_DELTA | \
                            PVS_FLAG_DUAL_FACE | PVS_FLAG_FOLD_A)

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

/* 解码耗时要比毫秒细: 双核 48 ms vs 单核 96 ms 的对比要看得清 */
static long mono_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000000L + t.tv_nsec / 1000L;
}

/* ---- register + bank access (real vs sim) ------------------------------- */
static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) { (void)sig; g_stop = 1; }

static uint8_t  *g_bank[FRAME_BANKS];      /* virtual addresses of bank A/B/C */
static uint32_t  g_bank_phys[FRAME_BANKS]; /* physical addresses (0x18/0x28) */
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
            if (f == MAP_FAILED) {
                perror("mmap " POVMEM_DEV " (falling back to /dev/mem)");
                /* v3.2 最常见原因: 三缓冲把帧区推到 40.4 MB, 超过 povmem.ko
                 * 当前默认的 0x1900000 窗口, mmap 直接 -EINVAL。SO 慢 5-10 倍。*/
                logts("HINT: need `insmod povmem.ko base=0x%08x size=0x%x` "
                      "(window must cover %u B)",
                      POVMEM_PHYS_BASE, (unsigned)POVMEM_MIN_SIZE,
                      (unsigned)FRAME_MAP_LEN);
            } else
                g_frame_wc = 1;
            close(pfd);
        }
    }
    if (f == MAP_FAILED) {
        f = mmap(NULL, FRAME_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED,
                 fd, frame_phys);
        if (f == MAP_FAILED) { perror("mmap frame region"); return -1; }
    }
    for (int i = 0; i < FRAME_BANKS; i++) {
        g_bank[i]      = (uint8_t *)f + (uint32_t)i * BANK_STRIDE;
        g_bank_phys[i] = frame_phys  + (uint32_t)i * BANK_STRIDE;
    }
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
    for (int i = 0; i < FRAME_BANKS; i++) {
        g_bank[i]      = f + (uint32_t)i * BANK_STRIDE;
        g_bank_phys[i] = frame_phys + (uint32_t)i * BANK_STRIDE;
    }
    logts("SIM: %d banks malloc'd, registers stubbed", FRAME_BANKS);
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
    /* 模拟 RTL 的 STATUS 回读位, 让一致性自检在 x86 上也走真路径 */
    if (off == REG_SLICE_BASE_B) {
        if (v) g_sim_regs[0] |=  STATUS_BASE_B_ACT;
        else   g_sim_regs[0] &= ~STATUS_BASE_B_ACT;
    } else if (off == REG_POV_CTRL) {
        if (v & CTRL_FOLD_A_EN) g_sim_regs[0] |=  STATUS_FOLD_A_EN;
        else                    g_sim_regs[0] &= ~STATUS_FOLD_A_EN;
        g_sim_regs[REG_POV_CTRL_RB / 4] = v;   /* 0x24 = POV_CTRL 影子回读 */
    }
    logts("SIM: reg[0x%02x] <= 0x%08x", off, v);
}

#endif /* SIM_NO_DEVMEM */

/* ---- 只写寄存器的影子 + PHASE_B 防御 -------------------------------------
 * 0x10 和 0x1C 读回来的都不是写进去的值 (一个给 locked|slice_idx, 一个给实时
 * idx_B), 所以软件必须自己记影子。影子初值 = RTL 复位值。
 *
 * PHASE_B 硬约束 (RTL idx_b_live 只做「一次条件减」而不是取模):
 *   PHASE_B >= n_slices  =>  idx_B 越界  =>  屏B 读野地址 => 花屏。
 * 两道防御:
 *   (1) DUAL_FACE 帧屏B 有自己那份数据, idx 就是 idx, 一律把 0x1C 写 0;
 *   (2) 任何写 POV_CTRL.n_slices 的地方先复检 PHASE_B < n_slices, 不满足钳 0。
 */
static uint32_t g_phase_b = PHASE_B_RESET;      /* 0x1C 影子 (0x1C 读不回来) */

static void phase_b_set(uint32_t v, const char *why)
{
    if (v == g_phase_b) return;              /* 纯老流永远命中这里 = 不碰 0x1C */
    reg_wr(REG_PHASE_B, v);
    logts("PHASE_B %u -> %u (%s)", g_phase_b, v, why);
    g_phase_b = v;
}

/* 写 POV_CTRL 的唯一入口: 先满足 PHASE_B < n_slices, 再落寄存器 */
static void pov_ctrl_write(uint32_t engine_slices, uint32_t bits)
{
    if (g_phase_b >= engine_slices) {
        logts("WARN: PHASE_B=%u >= n_slices=%u -> 钳到 0 "
              "(RTL 只做一次条件减, 否则屏B slice 索引越界)",
              g_phase_b, engine_slices);
        reg_wr(REG_PHASE_B, 0);
        g_phase_b = 0;
    }
    reg_wr(REG_POV_CTRL, (engine_slices << 16) | bits);
}

/* fold_a_en 是逐帧属性, 住在 POV_CTRL 里。RTL 加了 R 0x24 = POV_CTRL 影子
 * 回读 (与 0x10 写口逐位对齐), 所以这里可以安全 RMW —— 不用维护影子, 也不
 * 用跟 JTAG 抢所有权: 读到什么就在什么基础上改那一位。 */
static void fold_a_apply(int want_fold)
{
    uint32_t want = want_fold ? CTRL_FOLD_A_EN : 0u;
    uint32_t ctrl = reg_rd(REG_POV_CTRL_RB);
    if ((ctrl & CTRL_FOLD_A_EN) == want) return;   /* 已经是想要的样子 */
    reg_wr(REG_POV_CTRL, (ctrl & ~CTRL_FOLD_A_EN) | want);
}

/* 翻页后核对 STATUS 的两个回读位与本帧期望是否一致 (状态变化时才打日志,
 * 26 fps 下不刷屏) */
static void engine_check_status(int want_base_b, int want_fold)
{
    static int last = -1;
    uint32_t st = reg_rd(REG_STATUS);
    int got_b = !!(st & STATUS_BASE_B_ACT), got_f = !!(st & STATUS_FOLD_A_EN);
    int bad = (got_b != !!want_base_b) | ((got_f != !!want_fold) << 1);
    if (bad == last) return;
    last = bad;
    if (bad)
        logts("WARN: engine STATUS=0x%08x mismatch: base_b_act=%d(want %d) "
              "fold_a_en=%d(want %d) [0x28/0x10 写了但 PL 没认]",
              st, got_b, !!want_base_b, got_f, !!want_fold);
}

/* ---- triple-buffer staging + thread handoff ------------------------------
 * 三个 cached staging 缓冲:
 *   g_wr    RX 线程正在解码写入 (仅 RX 访问)
 *   g_ready 最新就绪帧 (mutex 保护的交接槽, 代数计数标新旧)
 *   g_disp  flip 线程持有 (memcpy 进 DDR bank 的源, 仅 flip 访问)
 * RX 发布 = swap(g_wr, g_ready) + gen++; flip 消费 = swap(g_ready, g_disp)。
 * DELTA 参考帧 g_prev 指向"最后 ACK 的 raw 帧"所在缓冲: 发布后它在 ready
 * 槽, 被 flip 消费后在 disp 槽 —— 两处都没人写 (flip 只读), RX 下一个拿到
 * 的写入缓冲永远是第三块, 所以参考帧内容在下一次发布前始终有效。
 *
 * v3.1: 帧长度可变, 所以交接的不只是指针 —— 每个 staging 槽带上本帧的
 * raw_len / n_slices / 面B 偏移 (0 = 单面), flip 线程照着拷贝+写基址。
 */
typedef struct {
    uint8_t *buf;            /* 容量恒为 PVS_FRAME_RAW_MAX */
    uint32_t raw_len;        /* 本帧有效字节数 = n_slices*PVS_SLICE_STRIDE */
    uint32_t n_slices;
    uint32_t face_b_off;     /* 面B 在 buf 内的字节偏移; 0 = 单面帧 */
    uint32_t fold_a;         /* PVS_FLAG_FOLD_A -> PL 的 POV_CTRL[6] */
} stage_t;

static stage_t g_wr, g_ready, g_disp;
static unsigned g_ready_gen, g_consumed_gen;
static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;

static uint8_t *g_prev;          /* DELTA 参考帧 (仅 RX 线程读写指针) */
static uint32_t g_prev_len;      /* 参考帧长度: DELTA 必须等长, 否则 NAK */
static uint32_t g_prev_face_b_off; /* 参考帧的面边界: 逐面 DELTA 要求也一致 */
static int      g_prev_valid;    /* 连接内是否已有 ACK 过的帧 */

static int g_crc_on   = 0;       /* --crc: 每帧算 crc32 (联调用, 量产关) */
static int g_win_dual = 0;       /* --flip-window dual: 半圈双窗 */
static const char *idle_path = NULL;   /* --idle-anim 容器路径 */
static int g_swap_faces = 0;     /* --swap-faces: 两面数据对调到另一块屏 (FOLD_A 帧忽略) */

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

/* ---- 逐面解码 job + 双核工作线程池 --------------------------------------
 * DUAL_FACE 的两条流各自独立 (protocol.h v3.2), 所以一个 job = 一条流 ->
 * 一段 staging 缓冲。两个 job 的 dst 区间以 nA*0x3000 为界, 天然 4 KB 对齐,
 * 不存在伪共享; src / prev 也各读各的; zlib 的 uncompress() 没有全局状态。
 * 因此两个 job 之间**没有任何共享写**, 可以直接摊到两个核上。
 *
 * ⚠ 裸机时代 ENABLE_DUAL_CORE=1 出过 UART desync 的历史教训: 那是 AMP 下两
 * 个核抢同一套外设寄存器/中断的问题, 与这里的用户态线程无关 (我们只碰自己
 * malloc 的内存, 寄存器只有 flip 线程一个写者)。真出问题用 `--decode serial`
 * 一键退回单核。
 */
typedef struct {
    const uint8_t *src;      /* 压缩流起点; codec==0 时忽略 (数据已就位) */
    uint32_t       src_len;
    uint8_t       *dst;      /* staging 缓冲里本面的起点 */
    uint32_t       dst_len;  /* 本面解压后字节数 (必然是 0x3000 的整数倍) */
    const uint8_t *prev;     /* DELTA 参考帧里**同面同偏移**的位置; NULL=不做 */
    uint32_t       codec;    /* PVS_FLAG_ZLIB / PVS_FLAG_RLE / 0 */
    int            rc;       /* 0 = ok */
    char           err[80];
} face_job_t;

/* 纯函数: 只读 src/prev, 只写 dst。两个 job 并发跑没有共享状态。 */
static void face_decode(face_job_t *j)
{
    j->rc = 0;
    j->err[0] = '\0';
    if (j->codec & PVS_FLAG_ZLIB) {
        uLongf dl = j->dst_len;
        int zr = uncompress(j->dst, &dl, j->src, j->src_len);
        if (zr != Z_OK || dl != j->dst_len) {
            snprintf(j->err, sizeof j->err, "zlib rc=%d dlen=%lu want=%u",
                     zr, (unsigned long)dl, j->dst_len);
            j->rc = -1;
            return;
        }
    } else if (j->codec & PVS_FLAG_RLE) {
        if (rle_decode(j->src, j->src_len, j->dst, j->dst_len) != 0) {
            snprintf(j->err, sizeof j->err, "RLE decode failed");
            j->rc = -1;
            return;
        }
    }
    if (j->prev)
        xor_frame(j->dst, j->prev, j->dst_len);
}

static face_job_t g_job[DEC_WORKERS];
static pthread_t  g_dec_tid[DEC_WORKERS];
static pthread_mutex_t g_dmu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_dc_start = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_dc_done  = PTHREAD_COND_INITIALIZER;
static unsigned g_dec_gen;        /* 每提交一批 +1 */
static unsigned g_dec_left;       /* 本批未完成的 job 数 */
static int      g_dec_pool;       /* 1 = 工作线程就绪 */
static int      g_dec_parallel = 1;   /* --decode serial 关掉 */
static int      g_dec_pinned;         /* 成功绑核的线程数 */

static void *dec_thread(void *arg)
{
    int idx = (int)(intptr_t)arg;
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    if (ncpu > 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET((size_t)(idx % (int)ncpu), &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) == 0) {
            __sync_fetch_and_add(&g_dec_pinned, 1);
            logts("decode worker %d pinned to CPU%d", idx, idx % (int)ncpu);
        } else {
            logts("WARN: decode worker %d: pthread_setaffinity_np failed (%s), "
                  "letting the scheduler place it", idx, strerror(errno));
        }
    }

    unsigned seen = 0;
    pthread_mutex_lock(&g_dmu);
    for (;;) {
        while (g_dec_gen == seen && !g_stop)
            pthread_cond_wait(&g_dc_start, &g_dmu);
        if (g_stop) break;
        seen = g_dec_gen;
        pthread_mutex_unlock(&g_dmu);
        face_decode(&g_job[idx]);          /* 锁外跑, 两核真并行 */
        pthread_mutex_lock(&g_dmu);
        if (--g_dec_left == 0)
            pthread_cond_signal(&g_dc_done);
    }
    pthread_mutex_unlock(&g_dmu);
    return NULL;
}

static void dec_pool_start(void)
{
    if (!g_dec_parallel) { logts("decode: serial (single core, --decode serial)"); return; }
    for (int i = 0; i < DEC_WORKERS; i++) {
        if (pthread_create(&g_dec_tid[i], NULL, dec_thread, (void *)(intptr_t)i) != 0) {
            logts("WARN: pthread_create decode worker %d failed (%s) -> 退回单核",
                  i, strerror(errno));
            for (int k = 0; k < i; k++) {      /* 起了一半就全收掉 */
                pthread_mutex_lock(&g_dmu);
                g_stop = 1; pthread_cond_broadcast(&g_dc_start);
                pthread_mutex_unlock(&g_dmu);
                pthread_join(g_dec_tid[k], NULL);
            }
            g_stop = 0;
            return;
        }
    }
    g_dec_pool = 1;
    logts("decode: %d parallel workers (dual-face faces decode concurrently)",
          DEC_WORKERS);
}

static void dec_pool_stop(void)
{
    if (!g_dec_pool) return;
    pthread_mutex_lock(&g_dmu);
    pthread_cond_broadcast(&g_dc_start);       /* g_stop 已由信号置位 */
    pthread_mutex_unlock(&g_dmu);
    for (int i = 0; i < DEC_WORKERS; i++)
        pthread_join(g_dec_tid[i], NULL);
    g_dec_pool = 0;
}

/* 提交两个 job 并等它们跑完。池子没起来 / --decode serial 时就地串行跑,
 * 结果逐字节相同 (face_decode 是纯函数)。 */
static int dec_run_pair(face_job_t *a, face_job_t *b)
{
    if (!g_dec_pool) {
        face_decode(a);
        face_decode(b);
    } else {
        pthread_mutex_lock(&g_dmu);
        g_job[0] = *a;
        g_job[1] = *b;
        g_dec_left = DEC_WORKERS;
        g_dec_gen++;
        pthread_cond_broadcast(&g_dc_start);
        while (g_dec_left)
            pthread_cond_wait(&g_dc_done, &g_dmu);
        *a = g_job[0];
        *b = g_job[1];
        pthread_mutex_unlock(&g_dmu);
    }
    return (a->rc || b->rc) ? -1 : 0;
}

/* ---- 空闲动画 (--idle-anim FILE) -----------------------------------------
 * 需求: 上电就有画面, 一旦有人推流就显示推的内容。
 * 做法: 没有客户端连接时, 由本进程按 --idle-fps 逐帧播放一个预压缩容器;
 *       有连接时 accept 循环进客户端分支, 自然停播。**单进程独占 DDR**,
 *       不存在端口争抢或两个写者打架 (那正是 pov_boot.sh 也起 pov_rxd 时
 *       出现的 bind 失败 + service 无限重启, 见 2026-08-03)。
 * 容器 anim.pvs 布局 (小端):
 *   'PVSA' | u32 n_frames | u32 n_slices | u32 flags | n×(u32 off,u32 len) | 压缩载荷…
 * 载荷就是 PVS1 的 payload 原样 (zlib), 所以复用同一条解码路径, 零特例。 */
static uint8_t *g_anim;              /* mmap 的整个容器 */
static size_t   g_anim_sz;
static uint32_t g_anim_n, g_anim_slices, g_anim_flags, g_anim_cur;
static const uint32_t *g_anim_idx;   /* 指向容器里的 (off,len) 表 */
static double   g_idle_fps = 8.0;
static long     idle_t0;        /* 上一帧起点, 用于扣掉解码耗时算等待 */

static int idle_anim_load(const char *path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) { logts("idle-anim: 打不开 %s (%s), 空闲时不播", path, strerror(errno)); return -1; }
    struct stat st;
    if (fstat(fd, &st) < 0 || (size_t)st.st_size < 16) { close(fd); return -1; }
    g_anim_sz = (size_t)st.st_size;
    g_anim = mmap(NULL, g_anim_sz, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (g_anim == MAP_FAILED) { g_anim = NULL; return -1; }
    if (memcmp(g_anim, "PVSA", 4) != 0) { logts("idle-anim: magic 不对"); g_anim = NULL; return -1; }
    const uint32_t *h = (const uint32_t *)(g_anim + 4);
    g_anim_n = h[0]; g_anim_slices = h[1]; g_anim_flags = h[2];
    g_anim_idx = h + 3;
    if (!g_anim_n || g_anim_slices > PVS_N_SLICES_MAX) { g_anim = NULL; return -1; }
    logts("idle-anim: %s %u 帧 n_slices=%u flags=0x%x (%.1f MB) @ %.1f fps",
          path, g_anim_n, g_anim_slices, g_anim_flags,
          g_anim_sz / 1048576.0, g_idle_fps);
    return 0;
}

/* 播下一帧: 解码 -> 发布, 走与网络帧完全相同的 staging 交接 */
static void idle_anim_step(void)
{
    if (!g_anim) return;
    uint32_t off = g_anim_idx[g_anim_cur * 2], len = g_anim_idx[g_anim_cur * 2 + 1];
    if ((size_t)off + len > g_anim_sz) { g_anim_cur = 0; return; }
    g_anim_cur = (g_anim_cur + 1) % g_anim_n;

    uint32_t raw_len = g_anim_slices * PVS_SLICE_STRIDE;
    uint32_t nA = (g_anim_flags & PVS_FLAG_DUAL_FACE)
                  ? ((g_anim_flags & PVS_FLAG_FOLD_A) ? PVS_N_SLICES_FOLD : PVS_N_SLICES) : 0;
    uint32_t fbo = nA ? nA * PVS_SLICE_STRIDE : 0;
    /* 容器里存的就是 PVS1 payload 原样, 所以走与网络帧同一条双流解码路径。
     * 空闲动画不带 DELTA (每帧都是关键帧), prev 恒为 NULL。 */
    const uint8_t *p0 = g_anim + off;
    uint32_t clen_a = 0;
    if (fbo) {
        clen_a = (uint32_t)p0[0] | ((uint32_t)p0[1] << 8)
               | ((uint32_t)p0[2] << 16) | ((uint32_t)p0[3] << 24);
        p0 += 4; len -= 4;
        if (!clen_a || clen_a >= len) return;
    }
    face_job_t ja, jb;
    memset(&ja, 0, sizeof ja); memset(&jb, 0, sizeof jb);
    ja.src = p0;              ja.src_len = fbo ? clen_a : len;
    ja.dst = g_wr.buf;        ja.dst_len = fbo ? fbo : raw_len;
    uint32_t codec = g_anim_flags & (PVS_FLAG_RLE | PVS_FLAG_ZLIB);
    ja.codec = codec;
    if (fbo) {
        jb.src = p0 + clen_a; jb.src_len = len - clen_a;
        jb.dst = g_wr.buf + fbo; jb.dst_len = raw_len - fbo;
        jb.codec = codec;
        if (dec_run_pair(&ja, &jb) != 0) return;
    } else {
        face_decode(&ja);
        if (ja.rc) return;
    }

    g_wr.raw_len = raw_len; g_wr.n_slices = g_anim_slices; g_wr.face_b_off = fbo;
    g_wr.fold_a = !!(g_anim_flags & PVS_FLAG_FOLD_A);
    pthread_mutex_lock(&g_mu);
    if (g_ready_gen != g_consumed_gen) g_st_drop++;
    stage_t t = g_wr; g_wr = g_ready; g_ready = t;
    g_ready_gen++;
    pthread_mutex_unlock(&g_mu);
    g_st_rx++;
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
            stage_t t = g_disp; g_disp = g_ready; g_ready = t;
            g_consumed_gen = g_ready_gen;
            fresh = 1;
        }
        pthread_mutex_unlock(&g_mu);
        if (!fresh) { usleep(500); continue; }

        /* 拷贝进空闲 bank, DSB 排空 write buffer 后引擎才可能取到。
         * 双面帧的 [面A][面B] 是连着的, 所以仍然只是一次 memcpy; 拆分只体现
         * 在两个基址寄存器上。单面 360 帧: raw_len == PVS_FRAME_RAW, 与 v2
         * 逐字节相同。 */
        int idle = active + 1;
        if (idle >= FRAME_BANKS) idle = 0;      /* A -> B -> C -> A 轮转 */
        memcpy(g_bank[idle], g_disp.buf, g_disp.raw_len);
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

        /* 本帧的几何全部在同一个翻页窗内背靠背落寄存器: 中间隔一整圈的话,
         * 一面来自新帧另一面来自旧帧 = 撕裂。顺序 PHASE_B -> BASE_B ->
         * fold_a_en -> BASE_A: 0x18 放最后, 因为 RTL 在 idx 变化那一拍同时
         * 快照 base_lat/base_lat_b, 前面几个先就位最稳。
         * 先写 B 再写 A 还有一层好处: PL 端 0x28==0 回落到 0x18, 所以
         * 「B 已新 / A 还旧」这个中间态最多让面B 早一个 pair 跳帧, 不会出现
         * 面B 指向上一帧 bank 的野地址。 */
        uint32_t base_a = g_bank_phys[idle];
        uint32_t base_b = g_disp.face_b_off ? base_a + g_disp.face_b_off : 0u;
        /* --swap-faces: 把两面的数据对调到另一块物理屏上 (调试/确认哪块屏是贴轴那面)。
         * ⚠ 只在两面等长时才允许 —— FOLD_A 时面A 只有 180 片且 fold_a_en 是
         * **引擎A 专属**的, 换过去引擎A 会拿着 360 片的数据还做半圈折叠 = 全错。
         * 所以 FOLD_A 帧直接忽略本开关 (启动时已告警), 要对调请渲不带 --fold-a 的 720 片。 */
        if (g_swap_faces && base_b && !g_disp.fold_a) {
            uint32_t t = base_a; base_a = base_b; base_b = t;
        }
        /* PHASE_B: 双面帧屏B 有自己那份数据 -> idx 就是 idx -> 必须写 0。
         * 单面帧回到 RTL 复位默认 180 (老的「屏B ≡ 屏A+180」共享数据玩法);
         * 纯老流两边都是 180, phase_b_set 直接短路, 一个字都不写。 */
        phase_b_set(base_b ? PHASE_B_DUAL : PHASE_B_RESET,
                    base_b ? "DUAL_FACE: 屏B 自己的数据 (180 补偿渲染侧面B 符号/手性)" : "单面: 共享数据默认");
        wmb_frame();                       /* frame data globally visible ... */
        reg_wr(REG_SLICE_BASE_B, base_b);
        fold_a_apply((int)g_disp.fold_a);
        reg_wr(REG_SLICE_BASE, base_a);
        wmb_frame();                       /* ... before + after base update  */
        engine_check_status(base_b != 0, (int)g_disp.fold_a);
#ifdef SIM_NO_DEVMEM
        /* SIM 自检: 把真正落进 bank 的内容按面算 crc32 打出来, x86 回归测试
         * 靠这行核对「面A/面B 写到了正确的物理偏移」。板上编译不进来。 */
        {
            uint32_t la = g_disp.face_b_off ? g_disp.face_b_off : g_disp.raw_len;
            uint32_t lb = g_disp.face_b_off ? g_disp.raw_len - g_disp.face_b_off : 0;
            logts("SIM: bankcrc A@0x%08x len=%u crc=%08x B@0x%08x len=%u crc=%08x",
                  base_a, la, (uint32_t)crc32(0L, g_bank[idle], la),
                  base_b, lb,
                  lb ? (uint32_t)crc32(0L, g_bank[idle] + g_disp.face_b_off, lb) : 0u);
        }
#endif
        active = idle;
        last_flip_win = forced ? -1 : win;
        g_st_flip++;
        if (forced) g_st_forced++;
        if (base_b || g_disp.fold_a)
            logts("FLIP gen=%u bank=%d win=%d n=%u A=0x%08x B=0x%08x fold=%u%s",
                  g_consumed_gen, idle, win, g_disp.n_slices, base_a, base_b,
                  g_disp.fold_a, forced ? " FORCED" : "");
        else
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
    g_prev_len   = 0;
    g_prev_face_b_off = 0;

    for (;;) {
        pvs_hdr_t h;
        int r = recv_full(fd, &h, sizeof h);
        if (r <= 0) return r;

        /* 头校验 (v3.1): n_slices 是权威, raw_len 必须与它自洽; 不再硬比
         * PVS_N_SLICES。未知 flag 仍用位掩码拒掉 (给未来留位), RLE+ZLIB 互斥。
         * 注意用 uint32_t 算长度: 720*0x3000 = 8847360, u16 会溢出。 */
        uint32_t n_slices = h.n_slices;
        uint32_t need_raw = n_slices * (uint32_t)PVS_SLICE_STRIDE;
        if (memcmp(h.magic, PVS_MAGIC, 4) != 0 ||
            n_slices < 1 || n_slices > PVS_N_SLICES_MAX ||
            h.raw_len != need_raw               ||
            h.comp_len == 0 || h.comp_len > COMP_LEN_MAX ||
            (h.flags & ~PVS_FLAGS_KNOWN) != 0   ||
            ((h.flags & PVS_FLAG_RLE) && (h.flags & PVS_FLAG_ZLIB))) {
            logts("NAK: bad header (magic=%.4s comp=%u raw=%u n=%u flags=0x%x)",
                  h.magic, h.comp_len, h.raw_len, h.n_slices, h.flags);
            send_byte(fd, PVS_NAK);
            return -1;
        }

        /* 面拆分: 双面 = [面A nA 片][面B 360 片], nA = FOLD_A ? 180 : 360。
         * 单面折叠 (FOLD_A 无 DUAL_FACE) = 只有面A 的 180 片。片数与 flag
         * 不自洽就 NAK —— 拆错了会把面B 的数据当成面A 写出去。 */
        uint32_t n_a = n_slices, face_b_off = 0;
        if (h.flags & PVS_FLAG_DUAL_FACE) {
            n_a = (h.flags & PVS_FLAG_FOLD_A) ? PVS_N_SLICES_FOLD : PVS_N_SLICES;
            if (n_slices != n_a + PVS_N_SLICES) {
                logts("NAK: DUAL_FACE n_slices=%u != %u (nA=%u + nB=%u)",
                      n_slices, n_a + PVS_N_SLICES, n_a, PVS_N_SLICES);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            face_b_off = n_a * (uint32_t)PVS_SLICE_STRIDE;
        } else if (h.flags & PVS_FLAG_FOLD_A) {
            if (n_slices != PVS_N_SLICES_FOLD) {
                logts("NAK: FOLD_A single face needs n_slices=%d, got %u",
                      PVS_N_SLICES_FOLD, n_slices);
                send_byte(fd, PVS_NAK);
                return -1;
            }
        }

        /* DELTA 无参考帧 (连接首帧/重连后) -> NAK, 发送端降级重发关键帧 */
        if ((h.flags & PVS_FLAG_DELTA) && !g_prev_valid) {
            logts("NAK: DELTA frame with no reference (need keyframe first)");
            send_byte(fd, PVS_NAK);
            return -1;
        }
        /* 几何切换 (360 -> 720 等) 时 XOR 没有定义: 参考帧短了就是读越界,
         * 长了就是残留旧面数据。逐面 DELTA 还要求**面边界也一样** —— 比如
         * 540 单面 和 540 双面+折叠 长度相同但面A/面B 的分界不同, 拿来互相
         * XOR 会把两个面的数据搅在一起。要求发送端在切几何时发关键帧。 */
        if ((h.flags & PVS_FLAG_DELTA) &&
            (h.raw_len != g_prev_len || face_b_off != g_prev_face_b_off)) {
            logts("NAK: DELTA layout %u/%u != reference %u/%u "
                  "(raw_len/face_b_off; geometry change needs a keyframe)",
                  h.raw_len, face_b_off, g_prev_len, g_prev_face_b_off);
            send_byte(fd, PVS_NAK);
            return -1;
        }

        /* ---- 载荷排布 (protocol.h v3.2) -----------------------------------
         *   单面: [压缩流]                            comp_len = 流长度
         *   双面: [u32 LE comp_len_A][面A 流][面B 流]  comp_len 含这 4 字节
         * 未压缩帧直接收进 g_wr.buf (双面时 A/B 连着收就正好是解压后的排布)。
         */
        int comp = h.flags & (PVS_FLAG_RLE | PVS_FLAG_ZLIB);
        uint32_t body_len = h.comp_len;      /* 去掉 4B 前缀后的净载荷 */
        uint32_t clen_a = 0, clen_b = 0;

        if (face_b_off) {
            uint8_t pfx[DUAL_PFX_LEN];
            if (h.comp_len < DUAL_PFX_LEN + 2) {   /* 两条流至少各 1 字节 */
                logts("NAK: DUAL_FACE comp_len=%u too short for [u32][A][B]",
                      h.comp_len);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            r = recv_full(fd, pfx, DUAL_PFX_LEN);
            if (r <= 0) return r;
            clen_a = (uint32_t)pfx[0]        | ((uint32_t)pfx[1] << 8) |
                     ((uint32_t)pfx[2] << 16) | ((uint32_t)pfx[3] << 24);
            body_len = h.comp_len - DUAL_PFX_LEN;
            if (clen_a == 0 || clen_a >= body_len) {
                logts("NAK: bad comp_len_A=%u (payload after prefix = %u, "
                      "两条流都必须非空)", clen_a, body_len);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            clen_b = body_len - clen_a;
            if (!comp && (clen_a != face_b_off || body_len != h.raw_len)) {
                logts("NAK: raw DUAL_FACE wants comp_len_A=%u body=%u, got %u/%u",
                      face_b_off, h.raw_len, clen_a, body_len);
                send_byte(fd, PVS_NAK);
                return -1;
            }
        } else if (!comp && h.comp_len != h.raw_len) {
            logts("NAK: raw frame but comp_len %u != raw_len", h.comp_len);
            send_byte(fd, PVS_NAK);
            return -1;
        }

        r = recv_full(fd, comp ? cbuf : g_wr.buf, body_len);
        if (r <= 0) return r;

        long dec_t0 = mono_us();

        if (face_b_off) {
            /* 双面: 两条独立流 -> 两个 job -> 两个核。各面各自 XOR 上一帧的
             * 同面数据 (参考帧布局已在头校验里确认与本帧一致)。 */
            const uint8_t *ref = (h.flags & PVS_FLAG_DELTA) ? g_prev : NULL;
            face_job_t ja, jb;
            memset(&ja, 0, sizeof ja);
            memset(&jb, 0, sizeof jb);
            ja.src     = comp ? cbuf : NULL;
            ja.src_len = clen_a;
            ja.dst     = g_wr.buf;
            ja.dst_len = face_b_off;
            ja.prev    = ref;
            ja.codec   = (uint32_t)comp;
            jb.src     = comp ? cbuf + clen_a : NULL;
            jb.src_len = clen_b;
            jb.dst     = g_wr.buf + face_b_off;
            jb.dst_len = h.raw_len - face_b_off;
            jb.prev    = ref ? ref + face_b_off : NULL;
            jb.codec   = (uint32_t)comp;
            if (dec_run_pair(&ja, &jb) != 0) {
                logts("NAK: dual-face decode failed (A: %s | B: %s)",
                      ja.rc ? ja.err : "ok", jb.rc ? jb.err : "ok");
                send_byte(fd, PVS_NAK);
                return -1;
            }
        } else {
            /* 单面: 与 v2 完全同一条路径 (不过线程池, 不多一次交接) */
            if (h.flags & PVS_FLAG_ZLIB) {
                uLongf dlen = h.raw_len;
                int zr = uncompress(g_wr.buf, &dlen, cbuf, h.comp_len);
                if (zr != Z_OK || dlen != h.raw_len) {
                    logts("NAK: zlib inflate failed (rc=%d dlen=%lu)", zr,
                          (unsigned long)dlen);
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } else if (h.flags & PVS_FLAG_RLE) {
                if (rle_decode(cbuf, h.comp_len, g_wr.buf, h.raw_len) != 0) {
                    logts("NAK: RLE decode failed");
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } /* else raw: already in g_wr.buf */

            /* DELTA 重建: raw = prev_acked_raw ^ decoded (原地 XOR) */
            if (h.flags & PVS_FLAG_DELTA)
                xor_frame(g_wr.buf, g_prev, h.raw_len);
        }

        long dec_us = mono_us() - dec_t0;   /* 解码耗时: 不含 crc, 不含翻页 */

        uint32_t crc = 0;
        if (g_crc_on)
            crc = crc32(0L, g_wr.buf, h.raw_len);

        /* 发布给 flip 线程 + 记参考帧; 旧 ready 没被消费就顶替 (丢帧计数) */
        g_wr.raw_len    = h.raw_len;
        g_wr.n_slices   = n_slices;
        g_wr.face_b_off = face_b_off;
        g_wr.fold_a     = (h.flags & PVS_FLAG_FOLD_A) ? 1u : 0u;
        g_prev = g_wr.buf;
        g_prev_len = h.raw_len;
        g_prev_face_b_off = face_b_off;
        g_prev_valid = 1;
        pthread_mutex_lock(&g_mu);
        if (g_ready_gen != g_consumed_gen) g_st_drop++;
        stage_t t = g_wr; g_wr = g_ready; g_ready = t;
        g_ready_gen++;
        pthread_mutex_unlock(&g_mu);
        g_st_rx++;
        g_st_dec_us += (unsigned long)dec_us;

        /* ACK 立即发: 解码完就回, **绝不等翻页窗**。翻页由 flip 线程独立轮询
         * slice 计数完成, 被顶替的 ready 帧就是天然丢帧 (最新帧赢), 显示永不
         * 阻塞。ACK 节拍 = 解码吞吐, 与机械转速彻底解耦。 */
        if (send_byte(fd, PVS_ACK) != 0) return -1;

        if (g_crc_on)
            logts("FRAME seq=%u n=%u comp=%u flags=0x%x crc=%08x dec=%.1fms%s",
                  seq++, n_slices, h.comp_len, h.flags, crc, dec_us / 1000.0,
                  face_b_off ? (g_dec_pool ? " par" : " ser") : "");
        else
            logts("FRAME seq=%u n=%u comp=%u flags=0x%x dec=%.1fms%s",
                  seq++, n_slices, h.comp_len, h.flags, dec_us / 1000.0,
                  face_b_off ? (g_dec_pool ? " par" : " ser") : "");
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
        "                 dual   = also near slice 180 (dual-panel, 26 pps)\n"
        "  --idle-anim F  无客户端时循环播放 F (anim.pvs 容器); 有推流自动让位\n"
        "  --idle-fps N   空闲动画帧率 (default 8)\n"
        "  --decode       parallel = 双面两条流分别解到 CPU0/CPU1 (default);\n"
        "                 serial   = 单核串行解 (对拍 / 出问题时退回)\n",
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
        else if (!strcmp(argv[i], "--swap-faces")) g_swap_faces = 1;
        else if (!strcmp(argv[i], "--idle-anim") && i + 1 < argc) idle_path = argv[++i];
        else if (!strcmp(argv[i], "--idle-fps")  && i + 1 < argc) g_idle_fps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--flip-window") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "dual")) g_win_dual = 1;
            else if (strcmp(m, "single")) { usage(argv[0]); return 2; }
        }
        else if (!strcmp(argv[i], "--decode") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "serial")) g_dec_parallel = 0;
            else if (strcmp(m, "parallel")) { usage(argv[0]); return 2; }
        }
        else { usage(argv[0]); return 2; }
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    struct sigaction sa = { .sa_handler = on_sig };  /* no SA_RESTART: EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    /* v3.1 帧区从 9.3 MB 涨到 24.4 MB, --base 给歪了就会写到内核 RAM 上,
     * 这里先按 mem=256M 的保留区 (0x10000000..0x1FFFFFFF) 体检一遍 */
    if (frame_phys < FRAME_REGION_BASE ||
        (uint64_t)frame_phys + FRAME_MAP_LEN > FRAME_REGION_END) {
        logts("WARN: frame window 0x%08x+0x%x escapes the mem=256M reserve "
              "0x%08x..0x%08x - check --base / kernel cmdline",
              frame_phys, (unsigned)FRAME_MAP_LEN,
              (unsigned)FRAME_REGION_BASE, (unsigned)(FRAME_REGION_END - 1));
    }
    if (frame_phys & 0xfffffu)
        logts("WARN: frame base 0x%08x is not 1 MB aligned", frame_phys);

    if (hw_init(reg_phys, frame_phys) != 0) return 1;

    logts("pov_rxd v3.2: %d DDR banks A=0x%08x B=0x%08x C=0x%08x "
          "(stride 0x%x, %u B max used each), regs=0x%08x",
          FRAME_BANKS, g_bank_phys[0], g_bank_phys[1], g_bank_phys[2],
          (unsigned)BANK_STRIDE, (unsigned)BANK_BYTES, reg_phys);
    logts("frame map: %s (%u B = 0x%x), crc=%s, flip-window=%s",
          g_frame_wc ? "WC via " POVMEM_DEV : "SO via /dev/mem",
          (unsigned)FRAME_MAP_LEN, (unsigned)FRAME_MAP_LEN,
          g_crc_on ? "on" : "off", g_win_dual ? "dual" : "single");
    /* 无条件打出 povmem 最小 size: 三缓冲后默认值 (0x1900000) 又不够了 */
    logts("povmem needs size>=0x%x (%u B); if the WC mmap failed above, "
          "`insmod povmem.ko base=0x%08x size=0x%x`",
          (unsigned)POVMEM_MIN_SIZE, (unsigned)POVMEM_MIN_SIZE,
          POVMEM_PHYS_BASE, (unsigned)POVMEM_MIN_SIZE);
    logts("frames: n_slices 1..%d, raw<=%u B; dual-face -> 0x18/0x28, "
          "fold-a -> 0x10[6], PHASE_B(0x1C) shadow=%u (RTL 复位值, 读不回来)",
          PVS_N_SLICES_MAX, (unsigned)PVS_FRAME_RAW_MAX, g_phase_b);
    logts("engine STATUS=0x%08x POV_CTRL=0x%08x",
          reg_rd(REG_STATUS), reg_rd(REG_POV_CTRL));

    /* start on bank A, 面B 基址清 0 (= PL 回落到 0x18 的单面老行为);
     * POV_CTRL is left alone unless --fake */
    reg_wr(REG_SLICE_BASE_B, 0);
    reg_wr(REG_SLICE_BASE, g_bank_phys[0]);
    if (fake_rps > 0.0) {
        uint32_t period = (uint32_t)((double)ACLK_HZ / (fake_rps * PVS_N_SLICES) + 0.5);
        reg_wr(REG_FAKE_PERIOD, period);
        /* pov_ctrl_write 会先保证 PHASE_B < n_slices 再落 0x10 */
        pov_ctrl_write(PVS_N_SLICES, (1u << 1) | 1u);
        logts("fake-spin: %.2f rps -> fake_period=%u ticks/slice, POV_CTRL set",
              fake_rps, period);
    }
    logts("POV_CTRL readback 0x24=0x%08x (fold_a_en=%u) -> fold 走安全 RMW",
          reg_rd(REG_POV_CTRL_RB),
          (reg_rd(REG_POV_CTRL_RB) & CTRL_FOLD_A_EN) ? 1u : 0u);

    /* 三缓冲 staging: 帧长度可变, 一律按最大帧分配 (3*8.85 MB + 8.9 MB 压缩
     * 缓冲 ≈ 35 MB, mem=256M 的 Linux 侧放得下), 每帧只用前 raw_len 字节。 */
    uint8_t *cbuf = malloc(COMP_LEN_MAX);
    g_wr.buf    = malloc(PVS_FRAME_RAW_MAX);
    g_ready.buf = malloc(PVS_FRAME_RAW_MAX);
    g_disp.buf  = malloc(PVS_FRAME_RAW_MAX);
    if (!cbuf || !g_wr.buf || !g_ready.buf || !g_disp.buf) {
        perror("malloc"); return 1;
    }

    dec_pool_start();

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
    if (idle_path) idle_anim_load(idle_path);
    logts("listening on :%d", port);

    while (!g_stop) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof peer;
        /* 无客户端时播空闲动画: poll 超时驱动帧率, 有连接立刻让位。
         * 不用阻塞 accept 是为了在等连接的同时还能出画面。 */
        if (g_anim) {
            /* ⚠ 节奏必须**减去解码耗时**再等: 早先写成固定等 1000/fps 再解码,
             * 两段时间相加 -> 实际帧率只有目标的一半 (11fps 目标实测 6fps)。 */
            struct pollfd pfd = { lfd, POLLIN, 0 };
            long period_us = (long)(1000000.0 / (g_idle_fps > 0.1 ? g_idle_fps : 0.1));
            long spent_us  = mono_us() - idle_t0;
            int  wait_ms   = (int)((period_us - spent_us) / 1000);
            if (wait_ms < 1) wait_ms = 1;          /* 解码已超预算 -> 立刻再来一帧 */
            int pr = poll(&pfd, 1, wait_ms);
            if (pr == 0) { idle_t0 = mono_us(); idle_anim_step(); continue; }
            if (pr < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        }
        int cfd = accept(lfd, (struct sockaddr *)&peer, &plen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            break;
        }
        if (g_anim) logts("idle-anim 暂停 (客户端接入)");
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        /* ghost guard: a killed WSL sender leaves this socket ESTAB forever
         * (FIN/RST never reaches us) and the single client slot deadlocks.
         * Keepalive detects a dead peer in ~10+3*3 s; the recv/send timeouts
         * cover the black-hole case where probes are silently eaten. */
        setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
        int ka = 10; setsockopt(cfd, IPPROTO_TCP, TCP_KEEPIDLE,  &ka, sizeof ka);
        ka = 3;      setsockopt(cfd, IPPROTO_TCP, TCP_KEEPINTVL, &ka, sizeof ka);
        ka = 3;      setsockopt(cfd, IPPROTO_TCP, TCP_KEEPCNT,   &ka, sizeof ka);
        /* 接收缓冲: PC 端 window=2 (最多 2 帧在途), 要能吸掉 ~1.5 帧压缩数据
         * (双面约 300 KB/帧)。内核记账时会把设定值 x2, 所以回读一下打出来。 */
        int rbuf = RCVBUF_BYTES;
        const char *how = "SO_RCVBUF";
        /* 先试 SO_RCVBUFFORCE: 本进程本来就要 root (/dev/mem), 有 CAP_NET_ADMIN
         * 就能绕过 net.core.rmem_max 的上限 —— 默认 rmem_max 常常只有 208 KB,
         * 普通 SO_RCVBUF 会被悄悄砍到一半需求量。失败再退回普通设置。 */
        if (setsockopt(cfd, SOL_SOCKET, SO_RCVBUFFORCE, &rbuf, sizeof rbuf) == 0)
            how = "SO_RCVBUFFORCE";
        else if (setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof rbuf) != 0)
            logts("WARN: setsockopt SO_RCVBUF %d failed (%s)", rbuf, strerror(errno));
        int rbuf_eff = 0;
        socklen_t rlen = sizeof rbuf_eff;
        if (getsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf_eff, &rlen) == 0) {
            logts("SO_RCVBUF: requested %d B via %s -> effective %d B "
                  "(%.1f 帧双面压缩 @300KB)", rbuf, how, rbuf_eff,
                  rbuf_eff / (300.0 * 1024.0));
            if (rbuf_eff < RCVBUF_MIN_EFF)
                logts("WARN: SO_RCVBUF effective %d B < %d B: window=2 的突发"
                      "可能压不住 (非 root? 抬 net.core.rmem_max)",
                      rbuf_eff, RCVBUF_MIN_EFF);
        }
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
    dec_pool_stop();
    logts("STAT rx=%u flip=%u drop=%u forced=%u dec_avg=%.1fms (final)",
          g_st_rx, g_st_flip, g_st_drop, g_st_forced,
          g_st_rx ? (double)g_st_dec_us / g_st_rx / 1000.0 : 0.0);
    logts("exiting (signal)");
    return 0;
}
