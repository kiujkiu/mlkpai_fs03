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
 *                        [17]=bpp_mode 回读 (v3.4 3-bit; RTL 若还没实现就恒 0,
 *                             本程序**不假设它存在**, 见 bcm_apply 的被动探测)
 *   0x0C W  CFG_MISC     [31:30]=subcmd。本程序只用 **subcmd=01** (v3.4 新增,
 *                        以前顶层未实现的那个槽):
 *                          [7:0]=oe_w1  plane1 的 OE 沿数 (默认 92)
 *                          [15:8]=oe_w2 plane2 的 OE 沿数 (默认 46)
 *                          [16]=bpp_mode 0=1-bit(兼容旧内容) 1=3-bit 行内 BCM
 *                        ⚠ oe_w0 (plane0 = **MSB**, 默认 184) **不在这里** ——
 *                        它复用 subcmd=10 的 oe_window[15:8]。而 1-bit 的亮度上限
 *                        也是同一个 oe_window (现固化成 111), 所以切 3-bit 必须连
 *                        oe_window 一起改成 184, 否则 4:2:1 比例变成
 *                        111:54:108 = 乱的。**subcmd=10 由 pov_boot.sh 固化,
 *                        本守护进程一个字都不写** (那个字里还带着 dclk_fast /
 *                        overlap_en / au_rows_max / oe_set_pulse, 从这里 RMW
 *                        就是拿整台机器的时序去赌)。见 pov_boot.sh 的 CFG_MISC。
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
 * bank 内布局 = 解压后载荷原样 (面 A/B 连续), flip 时按面拆基址
 * (stride = PVS_STRIDE(flags): 1-bit 0x3000 / 3-bit 0x9000):
 *   单面 (n_slices 片):  [bank+0 , bank+n_slices*stride)
 *                        0x18 <= bank        0x28 <= 0
 *   PVS_FLAG_DUAL_FACE:  面A [bank+0 , bank+nA*stride)
 *                        面B [bank+nA*stride , bank+n_slices*stride)
 *                        0x18 <= bank        0x28 <= bank + nA*stride
 *   nA = FOLD_A ? n_slices/3 : n_slices/2  (**从帧长推, 不是写死的 360/180**;
 *   老帧 720 -> 360+360 偏移 0x438000, fold540 -> 180+360 偏移 0x21C000, 同旧)。
 *
 * ---- v3.4 3-bit 帧放不放得下 (算式, 别再重算一遍) ------------------------
 * 一个 bank 的容量是 BANK_BYTES = PVS_FRAME_RAW_MAX = 0x870000 = 8847360 B,
 * 与色深无关 —— 变的只是「一片多大」:
 *   1-bit: 8847360 / 0x3000 = 720 片  (= PVS_N_SLICES_MAX)
 *   3-bit: 8847360 / 0x9000 = 240 片  (= PVS_N_SLICES_MAX_3BIT, 整除)
 * 方案 (05_3bit_bcm.md §4) 要的是**每面 60 槽**: 双面 120 片 * 0x9000
 *   = 4423680 B = 0x438000 = bank 的 **50%** ⇒ 三缓冲 / povmem 映射窗 /
 *   staging 缓冲**一个字节都不用改**, 现有 size=0x2900000 够用有余。
 * 什么时候才需要改: 想跑 3-bit **720 片** (26.5 MB/帧) 的话 ——
 *   720 * 0x9000 = 0x1950000 = 26542080 B > BANK_STRIDE(0x1000000) ⇒ bank 会
 *   踩到下一个 bank 头上, 必须 BANK_STRIDE -> 0x2000000 (32 MB),
 *   FRAME_MAP_LEN = 2*0x2000000 + 0x1950000 = 0x5950000 = 93716480 B (89.4 MiB),
 *   povmem 要 `size=0x5950000`; 保留区 0x10000000..0x1FFFFFFF (256 MB) 装得下,
 *   但 Linux 侧 staging 也要跟着涨到 3*26.5+26.6 = 106 MB (mem=256M 下要重新
 *   算)。**今天没人要这个配置**, 所以只留算式不改代码。
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
 *   v3.3 (LZ4 顶掉 zlib, 12 fps -> 48 fps):
 *   - 新增 PVS_FLAG_LZ4 (protocol.h bit5) = **LZ4 raw block**, 与 ZLIB/RLE
 *     互斥。A9 实测 (anime_dual720.bin, 720 片双面 8847360 B, 单核):
 *       zlib-6  376780 B 23.5x 163.5 ms  51.6 MB/s
 *       lz4-HC9 388166 B 22.8x  41.2 ms 204.6 MB/s
 *     压缩比只掉 3%, 解压快 4 倍。双面两条流照旧双核并行 -> ~20.6 ms/帧。
 *   - 🔴 只认 raw block (LZ4_decompress_safe), 不认 CLI 的 .lz4 帧格式;
 *     raw block 不带原长, dstCapacity 由 hdr.raw_len / 面长度给。
 *   - liblz4 是**静态链**进来的 (deps/arm/liblz4.a), 理由见下面 #include。
 *   - 老流 (ZLIB/RLE/raw) 逐字节不变: 压缩位没置 LZ4 就走原来的分支。
 *   - PVS_FLAG_MSTREAM (bit6): 载荷改成「流表 + 可变条数独立流」, 因为并行
 *     解码要按**工作量**切而不是按**面**切。fold540 (面A 折 180 + 面B 360)
 *     按面切时两核 makespan 被面B 的 360 片封顶 = 20.6 ms; 切成
 *     180/90/270 三条 (核0 拿前两条 = 270 片, 核1 拿第三条 = 270 片) 降到
 *     15.47 ms。⚠ 顺带纠正一个旧说法: **FOLD_A 只省链路 (−31%), 解码一分钱
 *     不省** —— 按面切的 makespan 由面B 封顶, 折不折叠一样。
 *     分组必须**连续**不能轮转 (见 dec_plan)。没置 MSTREAM 的帧照旧走
 *     「单流 / 按面两流」老分支, 逐字节不变。
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
/* ---- LZ4: 为什么是**静态链进来**而不是 dlopen / -l:liblz4.so.1 ------------
 * 板上确实有 /usr/lib/arm-linux-gnueabihf/liblz4.so.1.8.3, 但没有开发包
 * (apt 装不到 lz4/liblz4-dev)。三种接法只有一种在这里成立:
 *   -llz4          ✗ 交叉环境没有 ARM 的 liblz4.so + lz4.h, 链不了。
 *   -l:liblz4.so.1 ✗ 同上 (仍要一份 ARM 的 .so 在 sysroot 里), 而且本程序是
 *                    **-static** 构建 (见 Makefile / README: "板子不需要任何
 *                    工具链和匹配的 libz"), 静态可执行文件不能链动态库。
 *   dlopen()       ✗ 静态链接的 glibc 里 dlopen 事实上不可用 (要求运行时有
 *                    与构建时**完全同版本**的 libc.so, 否则直接失败), 为了一个
 *                    编解码器把整个二进制改成动态 = 丢掉「板子零依赖」这条
 *                    已经用了一年的性质, 还得赌板上 1.8.3 的 ABI。
 *   deps/arm/liblz4.a  ✓ 与既有 deps/arm/libz.a 同一套做法: 交叉编译一份
 *                    静态库committed 进仓库, 二进制自带解码器, 板上有没有
 *                    liblz4 都无所谓, 也不受板上 1.8.3 这个老版本影响。
 * 只用到 LZ4_decompress_safe() (板端只解不压), 但 .a 里两个 .o 都在, 链接器
 * 只会拉进真正用到的那个。LZ4 raw block 的格式跨版本稳定, 所以 PC 侧
 * liblz4 1.10.0 压的流, 这里 1.10.0 静态解, 与板上 1.8.3 也是互通的。 */
#include <lz4.h>

#include "../protocol.h"

/* ---- hardware constants ------------------------------------------------ */
#define REG_PHYS_DEFAULT   0x40010000u
#define REG_MAP_LEN        0x1000u
#define REG_STATUS         0x00
#define REG_CFG_MISC       0x0C                 /* W: [31:30]=subcmd, 见文件头 */
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

/* ---- v3.4 0x0C subcmd=01: BCM 权重 + bpp_mode ----------------------------
 * 写口是一个整字, 三个字段一起落 —— 没有 RMW, 也就没有"读到陈旧影子把别人的
 * 位误伤掉"这条路。 */
#define CFG_SUB_BCM        (1u << 30)           /* wdata[31:30] = 2'b01 */
#define BCM_BPP_MODE       (1u << 16)           /* [16] 0=1-bit 1=3-bit */
#define STATUS_BPP_MODE    (1u << 17)           /* STATUS[17] bpp_mode 回读 (可选) */
/* BCM 三个权重 27/54/108 沿 = 1:2:4, 全部 <= 111 ⇒ 不插 LWAIT ⇒ 时间上免费
 * (05_3bit_bcm.md §1)。oe_w0=27 走 subcmd=10 的 oe_window, 本进程不写, 见文件头。*/
/* 🔴 2026-08-20 上板实测改的权重与位序 —— 不是随便排的, 改之前先读 05_3bit_bcm.md:
 *   硬件 LWAIT = max(0, oe-111) 里的 111 来自"OE 结束后还要等行驱推进 80 拍",
 *   而 **plane 边界不推进行驱** ⇒ 只有最后一个 plane(plane2) 受 111 限制,
 *   plane0/plane1 的 OE 上限是移位窗的 192。实测: w0=187 / w1=187 都不加拍,
 *   只有 w2=187 让 frame_period 31590 -> 35694。
 *   ⇒ 把最大权重放在不受限的 plane0 上, 于是 host 侧 plane0 装 **MSB**,
 *      权重 184/92/46 (精确 4:2:1)。占空比 0.550 vs 1-bit 的 0.569 = 96.7%,
 *      **亮度几乎无损**, 且 frame_period 一拍不涨。
 *   ⚠ 旧值 27/54/108 是 LSB-first 时代的, 与现在的位序**搭配即非单调**
 *     (码值1=108沿 会比 码值2=54沿 还亮)。两者必须同时改, 改一个就是坏的。 */
#define OE_W0_3BIT_HINT    184u                 /* 仅用于日志提醒, 不写寄存器 */
#define OE_W1_DEFAULT      92u
#define OE_W2_DEFAULT      46u
#define OE_W_MIN           2u                   /* RTL 内箝 [2,187] */
#define OE_W_MAX           187u
/* 影子每这么多次 apply 强制重写一次: 防的是"别人(JTAG/引导脚本)动过 0x0C 而
 * 影子不知道"这种静默不一致。26 fps 下约每 10 秒一次, 开销 = 一次 AXI 写。 */
#define BCM_REASSERT_EVERY 256u
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
/* 🔴 v3.4: 代码里不再直接用这个常量 —— 写死的 180 在非 360 槽 (3-bit 每面 60)
 * 下会越界。flip 线程改成写「引擎每圈片数/2」, 360 槽时算出来就是这个 180。
 * 常量保留是因为上面那段推导是这个数的唯一出处。 */
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
#define FLIP_TIMEOUT_MS    2000                 /* engine idle? flip anyway (默认) */
/* 🔬 运行时可调 (--flip-timeout): 转子不转时用它模拟"每圈翻一次页"的节奏。
 * 用途: 圈级 BCM 实验 —— 把位平面摊到连续多圈上显示, 靠视觉暂留合成灰度。
 * 那种方案的角分辨率不掉(每屏只扫 1 个平面), 代价是闪烁; 而闪烁感必须按
 * **真实圈时间**才测得准 (15rps => 66.7ms), 2000ms 差了一个量级根本测不出来。 */
static unsigned g_flip_timeout_ms = FLIP_TIMEOUT_MS;

/* 🔬 圈级 BCM (--ring-bcm): 把 3 个位平面摊到**连续 3 圈**上显示, 靠视觉暂留
 * 在时间上合成灰度, 而不是在一屏之内做行内 BCM。
 *   行内 BCM: 每屏扫 3 遍 => 整屏时间 x3 => 角分辨率掉到 1/3
 *   圈级 BCM: 每屏只扫 1 遍 => **角分辨率不掉**, 代价是闪烁 (转速/3 的频率)
 * 权重靠逐圈改 oe_window 实现 (184/92/46), 与 host 侧按 MSB->LSB 顺序推的
 * 三帧一一对应。⚠ 同步全靠"不丢帧": 丢一帧就整体错位, 颜色会乱。
 * 1-bit 载荷只有 3-bit 的 1/3, 正常不该丢, 但 flip<rx 时结果不可信。 */
static int      g_ring_bcm = 0;
static unsigned g_ring_idx = 0;
static const unsigned RING_OE[3] = { 184u, 92u, 46u };   /* MSB, mid, LSB */
/* 0x0C sub10 基值: [23:16]=54 行, bit29=0 fast, bit28 overlap, bit27 cfg_we */
#define CFG_SUB10_BASE  0x98360001u
#define COMP_LEN_MAX       (PVS_FRAME_RAW_MAX + 0x10000u)
#define DUAL_PFX_LEN       4u                   /* [u32 LE comp_len_A] (老两流格式) */
#define MSTR_ENT_LEN       8u                   /* 流表一条 = {u32 comp_len, u32 n_slices} */
#define MSTR_TBL_MAX       (4u + PVS_MAX_STREAMS * MSTR_ENT_LEN)   /* 132 B */
#define DEC_WORKERS        2                    /* A9 双核 */
/* ⚠ 这里原本有 RCVBUF_BYTES/RCVBUF_MIN_EFF 两个常量和一段"把接收缓冲设到
 * 768 KB"的逻辑, 2026-08-06 删了 —— 设 SO_RCVBUF 会关掉内核接收窗自动放大,
 * 是"收包只有 3 MB/s"的真身。详见 g_rcvbuf 的注释。 */
#define PVS_FLAGS_KNOWN    (PVS_FLAG_RLE | PVS_FLAG_ZLIB | PVS_FLAG_DELTA | \
                            PVS_FLAG_DUAL_FACE | PVS_FLAG_FOLD_A | PVS_FLAG_LZ4 | \
                            PVS_FLAG_MSTREAM | PVS_FLAG_3BIT)
/* 压缩位集合: 恰好置一位 (或一位都不置 = raw)。多于一位 = 非法帧 -> NAK。 */
#define PVS_FLAGS_CODEC    (PVS_FLAG_RLE | PVS_FLAG_ZLIB | PVS_FLAG_LZ4)

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
    } else if (off == REG_CFG_MISC && (v >> 30) == 1u) {
        /* v3.4: 模拟 subcmd=01 的 bpp_mode 回读位, 让 bcm_apply 的被动探测
         * 与一致性告警在 x86 上也走真路径 (真板上 RTL 若还没实现这一位,
         * 探测拿不到 hi 就永不告警 —— 那条分支同样被这里的用例覆盖)。 */
        if (v & BCM_BPP_MODE) g_sim_regs[0] |=  STATUS_BPP_MODE;
        else                  g_sim_regs[0] &= ~STATUS_BPP_MODE;
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

/* ---- v3.4 bpp_mode / BCM 权重 (0x0C subcmd=01) ---------------------------
 * 🔴 本仓库吃过一次「只写寄存器 + 影子跟踪 = 静默不一致」的亏: 0x10 / 0x1C 读
 * 回来的都不是写进去的值, 影子初值一旦按 RTL 复位值猜错, 之后每次都会因为
 * 「值没变」而短路掉那次写 —— 硬件从此和软件想的不是一回事, 而日志里一个字都
 * 没有。0x0C subcmd=01 又是一个纯写口, 所以这里四条一起上:
 *   1) 影子初值 = **无效**, 不猜 RTL 复位值 —— 连接后的第一帧 (1-bit 也一样)
 *      无条件写一次, 硬件状态由我们**建立**, 不是**假设**;
 *   2) 一次写整字 (oe_w1/oe_w2/bpp_mode 同在一个字) ⇒ 根本不存在 RMW, 也就没有
 *      「拿陈旧影子把别人的位误伤掉」这条路;
 *   3) 每 BCM_REASSERT_EVERY 次强制重写一次, 防别人 (JTAG/引导脚本) 偷偷动过;
 *   4) STATUS[17] 若真有回读就核对, 但**被动探测**: 只有当我们亲眼看见这一位
 *      在我们写 1 时确实读回 1 (g_bcm_rb_hi), 才认定它存在并开始告警。RTL 还没
 *      实现这一位时它恒 0, 一条假告警都不会有 —— 「没有回读」与「回读说不一致」
 *      是两件事, 不许混。
 * 1-bit 路径的代价: 每个连接的第一帧**多一次** 0x0C 写 (bpp=0 + 默认权重, 对
 * 1-bit 显示没有任何作用 —— 1-bit 只用 oe_window)。逐帧数据通路一个字节不变。 */
static uint32_t g_oe_w1 = OE_W1_DEFAULT, g_oe_w2 = OE_W2_DEFAULT;
static uint32_t g_bcm_word;      /* 最近一次真正写进 0x0C sub01 的整字 */
static int      g_bcm_valid;     /* 0 = 一次都没写过 -> 影子无效 -> 下次必写 */
static unsigned g_bcm_since;     /* 距上次强制重写过了几次 apply */
static int      g_bcm_rb_hi;     /* 被动探测: 见过 STATUS[17] 跟着我们变成 1 */

static uint32_t bcm_word(int three_bit)
{
    return CFG_SUB_BCM | (g_oe_w1 & 0xffu) | ((g_oe_w2 & 0xffu) << 8)
         | (three_bit ? BCM_BPP_MODE : 0u);
}

/* 每帧翻页时调 (与 0x18/0x28/0x10 同一个翻页窗内)。three_bit = 本帧色深。 */
static void bcm_apply(int three_bit)
{
    uint32_t w = bcm_word(three_bit);
    int force = !g_bcm_valid || g_bcm_since >= BCM_REASSERT_EVERY;
    if (force || w != g_bcm_word) {
        int changed = !g_bcm_valid || ((w ^ g_bcm_word) & BCM_BPP_MODE) != 0;
        reg_wr(REG_CFG_MISC, w);
        if (changed)                       /* 色深切换才打日志, 重申不刷屏 */
            logts("bpp_mode -> %d (%s): 0x0C sub01 <= 0x%08x "
                  "[oe_w1=%u oe_w2=%u]%s", three_bit ? 1 : 0,
                  three_bit ? "3-bit BCM" : "1-bit", w, g_oe_w1, g_oe_w2,
                  three_bit ? "; ⚠ oe_w0 = 0x0C sub10 的 oe_window, 由 "
                              "pov_boot.sh 固化, 3-bit 要 27 而不是 111" : "");
        g_bcm_word = w;
        g_bcm_valid = 1;
        g_bcm_since = 0;
    } else {
        g_bcm_since++;
    }
    /* 回读核对 (被动探测, 见上面第 4 条) */
    {
        static int last_bad = -1;
        int got = (reg_rd(REG_STATUS) & STATUS_BPP_MODE) ? 1 : 0;
        int want = three_bit ? 1 : 0;
        if (got == want) {
            if (got) g_bcm_rb_hi = 1;      /* 这一位确实存在且跟着我们走 */
            last_bad = 0;
            return;
        }
        if (!g_bcm_rb_hi) return;          /* 还没证明它存在 -> 不许告警 */
        if (last_bad == 1) return;
        last_bad = 1;
        logts("WARN: STATUS[17] bpp_mode=%d 但本帧要 %d (0x0C sub01 写了 "
              "0x%08x 却没生效?) —— 3-bit/1-bit 内容会被按错的色深扫出来",
              got, want, g_bcm_word);
    }
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
    uint8_t *buf;            /* 容量恒为 PVS_FRAME_RAW_MAX (字节数上限与色深无关) */
    uint32_t raw_len;        /* 本帧有效字节数 = n_slices*stride */
    uint32_t n_slices;
    uint32_t stride;         /* v3.4: 片距 0x3000 (1-bit) / 0x9000 (3-bit) */
    uint32_t face_b_off;     /* 面B 在 buf 内的字节偏移; 0 = 单面帧 */
    uint32_t fold_a;         /* PVS_FLAG_FOLD_A -> PL 的 POV_CTRL[6] */
    uint32_t bpp3;           /* v3.4: PVS_FLAG_3BIT -> PL 的 0x0C sub01 bpp_mode */
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

/* ---- 诊断计数器 (2026-08-04 丢帧/端到端帧率排查) --------------------------
 * 老 STAT 行只有 dec_avg, 而且是**自启动以来的累计均值** —— 跑上几百帧后对
 * 参数变化完全钝感 (换一档要等好久才看得出差别), 也看不见时间花在哪一段。
 * 这里补一组**每个统计窗内清零**的量, 把一帧的开销拆开:
 *   dec   RX 线程解码 (zlib + DELTA XOR), 双面时 = max(面A, 面B)
 *   A/B   两条流各自的解码耗时 —— 双核是否真的对半分, 一眼可见
 *   cpy   flip 线程 memcpy 8.85 MB 进 DDR bank (WC/SO 差 5-10 倍)
 *   wait  flip 线程等翻页窗的时间 (引擎没转时会顶到 FLIP_TIMEOUT_MS)
 * 只加计数与打印, 不碰编解码/翻页逻辑。--diag off 可关掉那一行。
 */
static int g_diag = 1;                       /* --diag off 关掉 DIAG 行 */
/* --diag-nocopy: **只用于消融实验**, 跳过 flip 线程那次 8.85 MB 的
 * staging->DDR bank memcpy (画面因此不再更新, 但 rx/解码路径逐字节不变)。
 * 用来回答「那次拷贝到底吃掉多少解码吞吐」—— 两个解码核和 flip 线程在
 * 同一颗双核 A9 上抢内存带宽, 拷贝的代价不是它自己那几十毫秒而已。 */
static int g_nocopy = 0;
/* --no-rmem-fix: 跳过抬 net.core.rmem_max 那一步。现在只在 --rcvbuf N (>0)
 * 时才有意义 —— 默认路径根本不设 SO_RCVBUF, 也就不受 rmem_max 上限约束。
 * ⚠ 2026-08-05 曾把"抬 rmem_max"当成本轮最大收益写在这里, 那是错的: 交错
 *   A/B 两组中位数都是 8.00 帧/秒。真正的问题是**设 SO_RCVBUF 这个动作本身**
 *   会关掉接收窗自动放大, 见 g_rcvbuf 的注释。 */
static int g_no_rmem_fix = 0;
/* --diag-rxonly: **只用于消融实验**, 收完帧体立刻 ACK —— 不解码、不发布、
 * 不翻页、不 memcpy。画面完全不动, 但 socket 上的收包节奏与真实推流逐字节
 * 相同 (同一个发送端、同样的 272 KB 分帧、同样的每帧 ACK)。
 * 🔴 它回答的是一个别的办法回答不了的问题: DIAG 里的 `body` 有多少是**链路
 *    本身**, 有多少是**板端自己的处理把链路挤慢的**。合成 sink 测出的
 *    5-7 MB/s 与 pov_rxd 实测的 3.2 MB/s 差了一倍, 而两者的协议形状一样 ——
 *    差别只可能在这个进程里, 只能在这个进程里量。
 * ⚠ 这个模式下 dec 恒为 0, phase_bench 的「dec 0 = 空闲动画」判据会把样本
 *    全滤掉, 要配合 phase_ab 的 dec0=1。 */
static int g_rxonly = 0;
/* --rcvbuf N: 接收缓冲策略。**默认 0 = 一个字节都不设, 全交给内核自动调**。
 * 🔴 这是 2026-08-06 找到的真正瓶颈, 反直觉到必须写清楚:
 *   Linux 的接收窗默认由 DRS (tcp_rcv_space_adjust) **随吞吐自动长大**,
 *   上限 tcp_rmem[2] (本板 1.9 MB)。而**一旦应用调用 setsockopt(SO_RCVBUF)**,
 *   内核就置上 SOCK_RCVBUF_LOCK, DRS 从此**整个关掉** —— 窗口被钉死在
 *   握手时按初始 rcvbuf 算出的 window_clamp (几十 KB) 上, 再也长不大。
 *   于是"把接收缓冲调大"这件事的净效果是**把接收窗调小**。
 *   实测 (同一个 povstream, 同一条链路, 交错):
 *     设 768 KB (老行为): 2.1-2.8 MB/s     不设 (本默认): 6.6-7.8 MB/s
 *   同一时间同一块板上, 一个 Python 写的最小 sink (什么都没设) 就有 6-8 MB/s,
 *   比 C 写的 pov_rxd 快 3 倍 —— 差别全在这一行 setsockopt 上。
 * ⚠ 别再"因为一帧 272 KB 比缓冲大所以要调大缓冲"了: 窗口 x RTT 才是吞吐,
 *   而这条链路 RTT 有 30 ms, 钉死的几十 KB 窗正好就是 2-3 MB/s。 */
static int g_rcvbuf = 0;
static unsigned long g_w_dec_us, g_w_dec_max, g_w_c0_us, g_w_c1_us, g_w_dec_n;
static unsigned long g_w_cpy_us, g_w_cpy_max, g_w_wait_us, g_w_wait_max, g_w_cpy_n;
/* 🔴 2026-08-05 加: 收包耗时。老 DIAG 只有 dec/cpy/wait —— 一帧的四段里
 * **偏偏漏了最大的那一段**, 于是"链路不是瓶颈"这个结论一直没人能证伪。
 *   hdr  阻塞在等下一帧帧头 = 发送端还没发 (发送端节奏/ACK 往返)
 *   body 收帧体的时长      = 真正的链路投递时间
 * 实测这两段加起来 55-80 ms/帧, 比 dec(28) + cpy(18) 还大。 */
static unsigned long g_w_hdr_us, g_w_hdr_max, g_w_body_us, g_w_body_max, g_w_rcv_n;
/* 相位仪表: 翻页窗是 slice<8 (一圈 360 片里只有 8 片 = 1.5 ms @15rps)。
 * 帧"到货"(flip 线程拿到 ready) 的 slice 决定了它还来不来得及在本圈翻中,
 * 所以真正要看的是**到货相位的分布**, 不是 wait 的均值。arr[] = 到货 slice
 * 的 8 分箱直方图; rev1/rev2 = 相邻两次翻页间隔了 1 圈 / >=2 圈 的次数。 */
#define PHASE_BINS 8
static unsigned long g_w_arr[PHASE_BINS];
static unsigned long g_w_gap_us, g_w_gap_max, g_w_gap_n, g_w_rev1, g_w_rev2;

/* ---- 翻页相位锁定 (--phase-lock on, 默认 off) -----------------------------
 * 🔴 先说结论, 免得下一个人再花一天走同一条路:
 *    **在本板实测到的所有工况下, 相位都不是损失点, 这个开关不该打开。**
 *    留着它是因为机制本身是对的, 且 PHASE 行的仪表很有用。
 *
 * 原假设 (2026-08-05 立项时的): 一圈 66.7 ms(900 RPM), 一帧的活 dec 28 +
 * cpy 18 = 46 ms 装得下, 但解码启动相位随机 -> 一半时候赔一整圈 -> 7.5 fps,
 * "翻中就 15, 翻不中直接掉一半, 没有中间态"。
 *
 * 实测把它推翻了, 三条证据:
 *  1) 真实工况 (15 rps, fold540 + lz4-HC9) 跑 5x20 s: rx 8.0/s, flip 8.0/s,
 *     **drop = 0 / 668 帧**。收到的每一帧都上了屏, 没有任何一帧因为错过翻页窗
 *     而白收 —— 相位可损失的量是 0。DIAG 里那个 "wait 33-39 ms" 不是错过的
 *     机会, 是 flip 线程**没帧可翻在空转**。
 *  2) 每圈翻中 0.53 这个数, 完全由 rx(8.0/s) / eng(15 rev/s) = 0.53 解释,
 *     和相位无关: 供给只有需求的一半, 一半的圈本来就没有新帧。
 *  3) 把链路影响去掉 (delta 流) 后自由跑本来就有 90-98% 的"每圈翻中":
 *     随机相位**不是稳定态** —— 一旦错过一次, 翻页被推后一整圈, 下一帧就
 *     已经在 ready 里等着了, 环路自己把相位纠回来。所以"掉到 7.5 fps 就回不来"
 *     这个模型不成立。8 rps 交替 A/B 实测: lock=off 98% 翻中 8.00 flip/s,
 *     lock=on 91% 翻中 7.99 flip/s —— 锁了反而略差。
 *
 * 真正的瓶颈是**收包**: 一帧 272 KB 压缩数据要收 85-96 ms (≈3 MB/s), 而一圈
 * 只有 66.7 ms。老 DIAG 把 dec/cpy/wait 拆得很细却唯独没量收包, 所以
 * "链路 125 Mbps 不是瓶颈" 这个前提一直没被证伪。要 15 fps, 得先让
 * body + dec <= 66.7 ms, 也就是收包压到 ~40 ms 以内 (≈7 MB/s)。
 *
 * 机制本身 (下面的代码) 是这样的: 帧体收完、开始解码**之前**, 把 RX 线程按
 * 转子相位停一小会儿, 让 (dec + cpy) 正好在翻页窗开启前结束:
 *     起解片号 target = (0 - (dec+cpy+margin) / 每片微秒) mod n_slices
 * dec/cpy 取实测 EMA; 每片微秒由 flip 线程轮询 slice 时顺手积分 (它本来就在
 * 死盯 slice_idx, 不额外读寄存器)。
 *
 * 🔴 三条保险 —— 少一条就会把板子锁死或者越锁越慢:
 *  1) 引擎没转 (每片微秒还没测出来) -> 放行。否则没电机 / fake 没开时 RX
 *     线程会在这里死等, 表现和"网络断了"一模一样, 又是一次误诊。
 *  2) **一帧的服务时间 (收包 + 解码 + 拷贝) 已经 >= 一圈** -> 放行 (state=over)。
 *     这种情况下等只会把周期硬钉到 2 圈 (7.5 fps); 而自由跑虽然抖 (1 圈/2 圈
 *     交替), 平均反而更高。锁相是"把够用的余量花在对齐上", 没余量就别锁。
 *  3) 只在"往前等不超过 GATE_MAX_FRAC 圈"时等, 且有硬超时。已经错过 target
 *     就**立刻开解**: 错过就是错过, 硬等下一圈只会让 RX 线程停止收包, 把
 *     发送端也一起拖住 -> 下一帧更晚 -> 越锁越差。
 */
#define LOCK_MARGIN_US    4000   /* dec/cpy 有抖动, 早到不亏, 晚到赔一整圈 */
#define GATE_SPAN_SLICES  4      /* 命中窗宽(片): 太窄会被 usleep 粒度跳过 */
#define GATE_MAX_FRAC     0.55   /* 最多等这么多圈, 超过就当"已经错过" */
#define GATE_TIMEOUT_MS   150    /* 硬超时, 任何情况下不许卡死 RX 线程 */
static int g_lock = 0;                     /* --phase-lock on */
static const char *g_lock_state = "off";   /* off/nospin/over/wait/late */
static volatile unsigned long g_ups_q8;    /* 每片微秒 << 8 (flip 线程写) */
static volatile unsigned long g_ema_dec_us, g_ema_cpy_us, g_ema_svc_us;

/* x = 7/8 x + 1/8 v; 首次直接赋值 (否则要爬几十帧才到位) */
static void ema_add(volatile unsigned long *x, long v)
{
    unsigned long o = *x;
    *x = o ? (unsigned long)((o * 7 + (unsigned long)v) / 8) : (unsigned long)v;
}

static uint32_t engine_n_slices(void)
{
    uint32_t n = (reg_rd(REG_POV_CTRL_RB) >> 16) & 0xffffu;
    return n ? n : (uint32_t)PVS_N_SLICES;
}

/* 收完帧体、开始解码之前调。返回等了多少微秒 (0 = 没等)。*/
static long phase_gate(void)
{
    unsigned long ups = g_ups_q8;                 /* 每片微秒 << 8 */
    if (!g_lock)  { g_lock_state = "off";    return 0; }
    if (ups < 16) { g_lock_state = "nospin"; return 0; }   /* 保险 1 */
    uint32_t n_eng = engine_n_slices();
    long rev_us = (long)((ups * n_eng) >> 8);
    /* 保险 2: RX 线程一帧的服务时间 (收包 + 解码) 装不进一圈就别锁。
     * ⚠ 这里**不能**把 cpy 加进来 —— 第一版加了, 结果条件过严。锁住之后的
     * 稳态是这样排的 (t=0 是翻页窗):
     *      t=target          放行, 开始解
     *      t=target+dec=-cpy 发布 + ACK; flip 线程开拷
     *      t=0               拷完, 翻页 ✅
     *      t=-cpy .. -cpy+body  RX 收下一帧 (**与 flip 线程的 memcpy 并行**)
     *      t=target+rev      再次放行
     * 收敛条件解出来只有 body + dec <= rev, cpy 整个被 flip 线程那条腿吸收掉了。
     * 把 cpy 算进来会在 body+dec 明明够用时误判成 over 而白白关掉锁。 */
    long need_us = (long)g_ema_svc_us + LOCK_MARGIN_US;
    if (need_us >= rev_us) { g_lock_state = "over"; return 0; }

    /* 保险 2b: 目标相位 = "往回退 dec+cpy"。这个量一旦 >= 一圈, 取模之后
     * 就**绕回来指向一个毫无意义的相位**, 锁会把解码对齐到错误的地方 ——
     * 实测 (delta @12rps, dec 59 + cpy 34 = 93 > 一圈 83) 就是这样把
     * 12.0 flip/s 锁成了 11.0。装不下就老实放行。 */
    long back_us = (long)g_ema_dec_us + (long)g_ema_cpy_us + LOCK_MARGIN_US;
    if (back_us >= rev_us) { g_lock_state = "over"; return 0; }

    /* 目标: dec + cpy 干完正好赶上 slice 回到 0 (翻页窗 slice<8) */
    long back = back_us * 256 / (long)ups;            /* 需要提前的片数 */
    long target = ((long)n_eng - back) % (long)n_eng;

    uint32_t s = reg_rd(REG_POV_CTRL) & 0xffffu;
    long fwd = ((long)target - (long)s + (long)n_eng) % (long)n_eng;
    /* 保险 3: 要等超过 GATE_MAX_FRAC 圈 = 其实是"刚刚错过", 立刻开解 */
    if (fwd > (long)(n_eng * GATE_MAX_FRAC)) { g_lock_state = "late"; return 0; }

    g_lock_state = "wait";
    long t0 = mono_us(), tm0 = mono_ms();
    for (;;) {
        s = reg_rd(REG_POV_CTRL) & 0xffffu;
        long d = ((long)target - (long)s + (long)n_eng) % (long)n_eng;
        if (d == 0 || d > (long)n_eng - GATE_SPAN_SLICES) break;  /* 到/刚过 */
        if (mono_ms() - tm0 > GATE_TIMEOUT_MS) { g_lock_state = "tmo"; break; }
        if (g_stop) break;
        usleep(200);
    }
    return mono_us() - t0;
}
/* 引擎转速: flip 线程等翻页窗时本来就在死盯 slice_idx, 顺手算转速。
 * ⚠ 不能数 wrap 次数 —— 轮询循环**正是被一次回绕(slice 进窗)终止的**, 采样
 * 区间的端点由事件本身定义, 数出来必然偏大 (实测 16 rps 报成 34.8)。改成
 * **积分 slice 增量**: rev/s = Σ(slice 增量)/每圈片数/轮询时长, 与端点无关。 */
static unsigned long g_w_poll_us, g_w_adv;


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
    uint32_t       codec;    /* 🔴 这里存的是**协议 flag 位**, 不是 0/1/2 的枚举:
                              * PVS_FLAG_ZLIB(2) / PVS_FLAG_RLE(1) / PVS_FLAG_LZ4(32)
                              * / 0 = raw。踩过的坑: 曾经按「第几种编解码器」填
                              * codec=1, 正好命中 PVS_FLAG_RLE, 于是每帧都拿 zlib
                              * 流去跑 RLE 解码 —— 静默失败, 日志里一个字都没有。
                              * 加新编解码器时**只**加 flag 位, 别改成序号。 */
    int            rc;       /* 0 = ok */
    long           us;       /* 诊断: 本面实际解码耗时 (µs) */
    char           err[80];
} face_job_t;

/* 纯函数: 只读 src/prev, 只写 dst。两个 job 并发跑没有共享状态。
 * (LZ4_decompress_safe 与 zlib 的 uncompress 一样是无全局状态的纯函数。) */
static void face_decode(face_job_t *j)
{
    long t_face = mono_us();
    j->rc = 0;
    j->err[0] = '\0';
    if (j->codec & PVS_FLAG_LZ4) {
        /* raw block: 流里不带原长, dstCapacity 必须自己给 = 本面解压后字节数。
         * 返回值 = 实际写出的字节数; < 0 = 流损坏 (最常见的是被喂了 .lz4
         * **帧格式** —— 魔数 0x184D2204 会被当 token 解, 直接负数)。 */
        int n = LZ4_decompress_safe((const char *)j->src, (char *)j->dst,
                                    (int)j->src_len, (int)j->dst_len);
        if (n < 0 || (uint32_t)n != j->dst_len) {
            snprintf(j->err, sizeof j->err, "lz4 rc=%d want=%u (raw block?)",
                     n, j->dst_len);
            j->rc = -1;
            return;
        }
    } else if (j->codec & PVS_FLAG_ZLIB) {
        uLongf dl = j->dst_len;
        int zr = uncompress(j->dst, &dl, j->src, j->src_len);
        if (zr != Z_OK || dl != j->dst_len) {
            snprintf(j->err, sizeof j->err, "zlib rc=%d dlen=%lu want=%u",
                     zr, (unsigned long)dl, j->dst_len);
            j->rc = -1;
            j->us = mono_us() - t_face;
            return;
        }
    } else if (j->codec & PVS_FLAG_RLE) {
        if (rle_decode(j->src, j->src_len, j->dst, j->dst_len) != 0) {
            snprintf(j->err, sizeof j->err, "RLE decode failed");
            j->rc = -1;
            j->us = mono_us() - t_face;
            return;
        }
    }
    if (j->prev)
        xor_frame(j->dst, j->prev, j->dst_len);
    j->us = mono_us() - t_face;
}

/* ---- 把 N 条流静态派给 DEC_WORKERS 个核 --------------------------------
 * 分组必须是**连续区间**, 不能轮转。反例: 三条流 180/90/270 片, 轮转会得到
 * 核0={流0,流2}=450 片 / 核1={流1}=90 片, makespan 450 —— 比按面切还差;
 * 连续分组能取到 {流0,流1}=270 / {流2}=270 的完美平衡 (makespan 270)。
 * 权重用 dst_len (解压后字节数 ∝ 片数), 因为解压耗时基本正比于输出量。
 * 纯静态派活, 不做工作窃取: 流表是发送端算好的, 板端不需要再猜。
 */
static void dec_plan(const face_job_t *j, int n, int *first, int *count)
{
    uint64_t total = 0, left;
    if (n <= DEC_WORKERS) {              /* 流数不多于核数: 一核一条, 多的核闲着 */
        for (int w = 0; w < DEC_WORKERS; w++) {
            first[w] = w < n ? w : n;
            count[w] = w < n ? 1 : 0;
        }
        return;
    }
    for (int i = 0; i < n; i++) total += j[i].dst_len;
    left = total;
    int i = 0;
    for (int w = 0; w < DEC_WORKERS; w++) {
        first[w] = i;
        count[w] = 0;
        int wleft = DEC_WORKERS - w;            /* 含本 worker 在内还剩几个 */
        if (wleft == 1) { count[w] = n - i; break; }   /* 最后一个吃光 */
        uint64_t target = left / (uint64_t)wleft, acc = 0;
        while (i < n - (wleft - 1)) {           /* 每个后续 worker 至少留一条 */
            uint64_t next = acc + j[i].dst_len;
            /* 已经拿了东西, 再拿一条会越过 target 且离得更远 -> 停 */
            if (acc && next > target && (next - target) > (target - acc)) break;
            acc = next; count[w]++; i++;
            if (acc >= target) break;
        }
        left -= acc;
    }
}

static face_job_t g_job[PVS_MAX_STREAMS];
/* v3.4: decode plan 日志里把 dst_len 换算成"片"要用本帧片距 (0x3000/0x9000)。
 * 只给日志用, 且只有 RX 线程 (serve_client / idle_anim_step) 会写它, 提交
 * dec_run 之前设好即可。 */
static uint32_t g_log_stride = PVS_SLICE_STRIDE;
static int        g_wfirst[DEC_WORKERS], g_wcount[DEC_WORKERS];
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
        int f = g_wfirst[idx], c = g_wcount[idx];
        pthread_mutex_unlock(&g_dmu);
        for (int k = 0; k < c; k++)        /* 锁外跑, 两核真并行 */
            face_decode(&g_job[f + k]);
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

/* 提交 n 条流并等它们跑完 (n ∈ [1, PVS_MAX_STREAMS])。池子没起来 /
 * --decode serial 时就地串行跑, 结果逐字节相同 (face_decode 是纯函数)。
 * 返回 -1 = 有流解失败 (jobs[i].rc/err 里有细节)。 */
static int dec_run(face_job_t *jobs, int n)
{
    if (!g_dec_pool) {
        for (int i = 0; i < n; i++) face_decode(&jobs[i]);
    } else {
        pthread_mutex_lock(&g_dmu);
        for (int i = 0; i < n; i++) g_job[i] = jobs[i];
        dec_plan(g_job, n, g_wfirst, g_wcount);
        /* 派活方案变了才打一行 (26+ fps 下不刷屏)。上板调参时这一行是唯一能
         * 看出「两个核到底各拿了多少片」的地方。 */
        {
            static int last_n = -1, last_c[DEC_WORKERS];
            int changed = (last_n != n);
            for (int w = 0; w < DEC_WORKERS; w++)
                if (last_c[w] != g_wcount[w]) changed = 1;
            if (changed) {
                char line[160];
                size_t k = 0;
                for (int w = 0; w < DEC_WORKERS && k + 1 < sizeof line; w++) {
                    uint64_t sl = 0;
                    for (int i = 0; i < g_wcount[w]; i++)
                        sl += g_job[g_wfirst[w] + i].dst_len / g_log_stride;
                    if (g_wcount[w] == 0)
                        k += (size_t)snprintf(line + k, sizeof line - k,
                                              "%score%d=空闲", w ? ", " : "", w);
                    else
                        k += (size_t)snprintf(line + k, sizeof line - k,
                                              "%score%d=流[%d..%d] %llu片", w ? ", " : "",
                                              w, g_wfirst[w],
                                              g_wfirst[w] + g_wcount[w] - 1,
                                              (unsigned long long)sl);
                }
                logts("decode plan: %d 条流 -> %s", n, line);
                last_n = n;
                for (int w = 0; w < DEC_WORKERS; w++) last_c[w] = g_wcount[w];
            }
        }
        g_dec_left = DEC_WORKERS;
        g_dec_gen++;
        pthread_cond_broadcast(&g_dc_start);
        while (g_dec_left)
            pthread_cond_wait(&g_dc_done, &g_dmu);
        for (int i = 0; i < n; i++) jobs[i] = g_job[i];
        pthread_mutex_unlock(&g_dmu);
    }
    for (int i = 0; i < n; i++)
        if (jobs[i].rc) return -1;
    return 0;
}

/* FRAME 日志的解码标签: 流数 + 并行/串行。单流单面时留空 (与老日志一致)。 */
static const char *dec_tag(uint32_t nstr, uint32_t face_b_off)
{
    static char tag[24];
    uint32_t n = nstr ? nstr : (face_b_off ? 2u : 1u);
    if (n < 2) return "";
    snprintf(tag, sizeof tag, " %ustr/%s", n, g_dec_pool ? "par" : "ser");
    return tag;
}

/* 出错时把每条流的状态拼成一行日志 (哪条流坏了要一眼看出来) */
static void dec_errline(const face_job_t *jobs, int n, char *out, size_t cap)
{
    size_t k = 0;
    for (int i = 0; i < n && k + 1 < cap; i++)
        k += (size_t)snprintf(out + k, cap - k, "%s#%d:%s", i ? " | " : "", i,
                              jobs[i].rc ? jobs[i].err : "ok");
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
    if (!g_anim_n || g_anim_slices > PVS_N_SLICES_MAX_F(g_anim_flags)) {
        logts("idle-anim: n_slices=%u 超过本色深上限 %u (flags=0x%x)",
              g_anim_slices, PVS_N_SLICES_MAX_F(g_anim_flags), g_anim_flags);
        g_anim = NULL; return -1;
    }
    /* 容器格式只覆盖「单流 / DUAL_FACE 两流」这两种排布 (下面 idle_anim_step
     * 就是照着这两种写的)。带 MSTREAM 流表的载荷这里解不了 —— 与其静默解出
     * 半帧垃圾, 不如当场拒掉。要用就重新打一份不带 MSTREAM 的容器。 */
    if (g_anim_flags & PVS_FLAG_MSTREAM) {
        logts("idle-anim: flags 带 PVS_FLAG_MSTREAM, 本容器路径不支持多流流表, 不播");
        g_anim = NULL; return -1;
    }
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

    /* v3.4: 片距与面拆分都从容器的 flags 推, 与网络帧同一套规则 (写死 360/180
     * 会让 60 槽的 3-bit 容器把面B 的数据当面A 写出去)。 */
    uint32_t stride  = PVS_STRIDE(g_anim_flags);
    uint32_t raw_len = g_anim_slices * stride;
    uint32_t nA = 0;
    if (g_anim_flags & PVS_FLAG_DUAL_FACE) {
        uint32_t div = (g_anim_flags & PVS_FLAG_FOLD_A) ? 3u : 2u;
        if (g_anim_slices % div) { g_anim = NULL; return; }   /* 容器不自洽, 停播 */
        nA = g_anim_slices / div;
    }
    uint32_t fbo = nA ? nA * stride : 0;
    g_log_stride = stride;
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
    face_job_t j2[2];
    memset(j2, 0, sizeof j2);
    j2[0].src = p0;              j2[0].src_len = fbo ? clen_a : len;
    j2[0].dst = g_wr.buf;        j2[0].dst_len = fbo ? fbo : raw_len;
    uint32_t codec = g_anim_flags & PVS_FLAGS_CODEC;   /* flag 位原样, 不是枚举 */
    j2[0].codec = codec;
    if (fbo) {
        j2[1].src = p0 + clen_a; j2[1].src_len = len - clen_a;
        j2[1].dst = g_wr.buf + fbo; j2[1].dst_len = raw_len - fbo;
        j2[1].codec = codec;
        if (dec_run(j2, 2) != 0) return;
    } else {
        face_decode(&j2[0]);
        if (j2[0].rc) return;
    }

    g_wr.raw_len = raw_len; g_wr.n_slices = g_anim_slices; g_wr.face_b_off = fbo;
    g_wr.stride = stride;
    g_wr.fold_a = !!(g_anim_flags & PVS_FLAG_FOLD_A);
    g_wr.bpp3   = !!(g_anim_flags & PVS_FLAG_3BIT);
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
    uint32_t n_eng = PVS_N_SLICES;   /* 引擎每圈片数 (诊断算转速用) */
    long last_flip_us = 0;           /* 上次翻页时刻 (算翻页间隔 = 几圈) */
    long stat_t0 = mono_ms();
    unsigned stat_rx0 = 0, stat_flip0 = 0;

    while (!g_stop) {
        /* 每秒一行统计 (26 fps 下逐帧 printf 走串口是可观开销) */
        long now = mono_ms();
        if (now - stat_t0 >= 1000) {
            unsigned rx = g_st_rx, fl = g_st_flip;
            if (rx != stat_rx0 || fl != stat_flip0) {
                logts("STAT rx=%u flip=%u drop=%u forced=%u dec_avg=%.1fms",
                      rx, fl, g_st_drop, g_st_forced,
                      rx ? (double)g_st_dec_us / rx / 1000.0 : 0.0);
                /* 窗内瞬时值: 累计均值对变化钝感, 排查参数时看这行 */
                if (g_diag) {
                    double dt = (now - stat_t0) / 1000.0;
                    unsigned long dn = g_w_dec_n, cn = g_w_cpy_n;
                    unsigned long rn = g_w_rcv_n;
                    logts("DIAG %.1fs eng=%.1frev/s rx=%.2f/s flip=%.2f/s drop=%u | "
                          "dec %.1f/%.1fms (core0 %.1f core1 %.1f) | cpy %.1f/%.1fms | "
                          "wait %.1f/%.1fms | hdr %.1f/%.1fms | body %.1f/%.1fms",
                          dt,
                          g_w_poll_us ? g_w_adv * 1e6 / n_eng / g_w_poll_us : 0.0,
                          (rx - stat_rx0) / dt, (fl - stat_flip0) / dt,
                          g_st_drop,
                          dn ? g_w_dec_us / (double)dn / 1000.0 : 0.0,
                          g_w_dec_max / 1000.0,
                          dn ? g_w_c0_us / (double)dn / 1000.0 : 0.0,
                          dn ? g_w_c1_us / (double)dn / 1000.0 : 0.0,
                          cn ? g_w_cpy_us / (double)cn / 1000.0 : 0.0,
                          g_w_cpy_max / 1000.0,
                          cn ? g_w_wait_us / (double)cn / 1000.0 : 0.0,
                          g_w_wait_max / 1000.0,
                          rn ? g_w_hdr_us / (double)rn / 1000.0 : 0.0,
                          g_w_hdr_max / 1000.0,
                          rn ? g_w_body_us / (double)rn / 1000.0 : 0.0,
                          g_w_body_max / 1000.0);
                    /* 单独一行, 免得 DIAG 行长到串口换行。字段顺序被
                     * tools/phase_bench.py 的 PHASE_RE 依赖, 改要一起改。 */
                    unsigned long gn = g_w_gap_n;
                    logts("PHASE arr=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu "
                          "gap %.1f/%.1fms rev1=%lu rev2+=%lu lock=%s",
                          g_w_arr[0], g_w_arr[1], g_w_arr[2], g_w_arr[3],
                          g_w_arr[4], g_w_arr[5], g_w_arr[6], g_w_arr[7],
                          gn ? g_w_gap_us / (double)gn / 1000.0 : 0.0,
                          g_w_gap_max / 1000.0, g_w_rev1, g_w_rev2,
                          g_lock_state);
                }
                g_w_dec_us = g_w_dec_max = g_w_c0_us = g_w_c1_us = g_w_dec_n = 0;
                g_w_cpy_us = g_w_cpy_max = g_w_wait_us = g_w_wait_max = g_w_cpy_n = 0;
                g_w_hdr_us = g_w_hdr_max = g_w_body_us = g_w_body_max = g_w_rcv_n = 0;
                memset(g_w_arr, 0, sizeof g_w_arr);
                g_w_gap_us = g_w_gap_max = g_w_gap_n = g_w_rev1 = g_w_rev2 = 0;
                g_w_poll_us = g_w_adv = 0;
            }
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

        /* 相位仪表: 帧"到货"时转子在哪一片。这才是决定本圈翻不翻得中的量,
         * 分布均匀 = 相位随机 (老行为), 集中在某一箱 = 锁住了。 */
        {
            uint32_t ne = engine_n_slices();
            uint32_t sa = reg_rd(REG_POV_CTRL) & 0xffffu;
            if (ne) g_w_arr[(sa * PHASE_BINS / ne) % PHASE_BINS]++;
        }

        /* 拷贝进空闲 bank, DSB 排空 write buffer 后引擎才可能取到。
         * 双面帧的 [面A][面B] 是连着的, 所以仍然只是一次 memcpy; 拆分只体现
         * 在两个基址寄存器上。单面 360 帧: raw_len == PVS_FRAME_RAW, 与 v2
         * 逐字节相同。 */
        int idle = active + 1;
        if (idle >= FRAME_BANKS) idle = 0;      /* A -> B -> C -> A 轮转 */
        long t_cpy = mono_us();
        if (!g_nocopy)                          /* --diag-nocopy: 消融实验 */
            memcpy(g_bank[idle], g_disp.buf, g_disp.raw_len);
        wmb_frame();
        t_cpy = mono_us() - t_cpy;              /* 诊断: 进 DDR bank 的耗时 */

        /* 等翻页窗 (先离开上次翻页的窗口, 再命中任一窗口) */
        long t_wait = mono_us();
        long t0 = mono_ms();
        int need_leave = (last_flip_win >= 0);
        int win = -1, forced = 0;
        /* 每圈片数取 POV_CTRL 影子回读 (0x24), 兜底 PVS_N_SLICES */
        n_eng = (reg_rd(REG_POV_CTRL_RB) >> 16) & 0xffffu;
        if (!n_eng) n_eng = PVS_N_SLICES;
        uint32_t slice = reg_rd(REG_POV_CTRL) & 0xffffu, prev_slice = slice;
        unsigned long adv = 0;
        for (;;) {
            slice = reg_rd(REG_POV_CTRL) & 0xffffu;
            adv += (slice - prev_slice + n_eng) % n_eng;   /* 走过的片数 */
            prev_slice = slice;
            win = slice_window(slice);
            if (need_leave && win != last_flip_win)
                need_leave = 0;
            if (!need_leave && win >= 0)
                break;
            if (mono_ms() - t0 > (long)g_flip_timeout_ms) {
                /* 🔴 这条以前只说 "engine idle?", 于是现场看到的只有 drop 一路
                 * 飙升, 被当成**网络丢帧**查了很久 (2026-08-04 定案: 电机不转时
                 * slice_idx 恒定, 翻页窗的「先离开再进入」去抖条件永远不成立,
                 * 每帧顶满 2 秒才强制翻一次 -> flip≈0.5/s, drop 90%+)。
                 * 现在把判据直接印出来: slice 卡住 = 机械/传感器问题, 不是链路。*/
                if (!adv)
                    logts("WARN: %d ms 内引擎一片都没走 (slice_idx 恒为 %u) —— "
                          "电机停了 / index 脉冲丢了 / 还在 sensor 模式而没上电? "
                          "本次强制翻页。⚠ 这种状态下 drop 会飙到 90%%+, "
                          "**与网络无关**, 别去查链路", g_flip_timeout_ms, slice);
                else
                    logts("WARN: no flip window in %d ms (slice=%u, 本轮走了 %lu 片, "
                          "翻页窗判据没命中?), flipping anyway",
                          g_flip_timeout_ms, slice, adv);
                forced = 1;
                break;
            }
            if (g_stop) return NULL;
            usleep(200);
        }
        t_wait = mono_us() - t_wait;            /* 诊断: 等翻页窗的耗时 */
        g_w_poll_us += (unsigned long)t_wait;   /* 轮询时长 = slice 增量的积分区间 */
        g_w_adv     += adv;
        g_w_cpy_us += (unsigned long)t_cpy;
        if ((unsigned long)t_cpy > g_w_cpy_max) g_w_cpy_max = (unsigned long)t_cpy;
        g_w_wait_us += (unsigned long)t_wait;
        if ((unsigned long)t_wait > g_w_wait_max) g_w_wait_max = (unsigned long)t_wait;
        g_w_cpy_n++;
        ema_add(&g_ema_cpy_us, t_cpy);
        /* 每片微秒: 轮询循环本来就在积分 slice 增量, 顺手换算。adv 太小时
         * (一进来就命中窗) 比值噪声大, 要求走过 >=16 片才采信。 */
        if (adv >= 16 && t_wait > 0)
            ema_add(&g_ups_q8, (long)((unsigned long)t_wait * 256 / adv));

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
        /* PHASE_B = **半圈** (两种帧都是):
         *   单面 = 老的「屏B ≡ 屏A + 半圈」共享数据玩法;
         *   双面 = 补偿渲染侧面B 的符号/手性 (见 PHASE_B_DUAL 上方那段推导)。
         * 🔴 半圈 = 引擎每圈片数/2, **不是常数 180** —— RTL 的 idx_b_live 只做
         * 一次条件减不取模, PHASE_B >= 引擎片数 = 屏B 索引越界 = 读野地址花屏。
         * n_eng=360 时算出来仍是 180 = RTL 复位值 ⇒ phase_b_set 直接短路, 纯老流
         * 一个字都不写 (逐位不变); 3-bit 走 60 槽时它必须是 30。 */
        uint32_t half_eng = n_eng ? n_eng / 2u : PHASE_B_RESET;
        phase_b_set(half_eng,
                    base_b ? "DUAL_FACE: 屏B 自己的数据 (半圈补偿渲染侧面B 符号/手性)"
                           : "单面: 共享数据默认 (半圈)");
        wmb_frame();                       /* frame data globally visible ... */
        reg_wr(REG_SLICE_BASE_B, base_b);
        fold_a_apply((int)g_disp.fold_a);
        /* 色深: 必须与本帧内容同一个翻页窗内落下去, 否则引擎会拿 1-bit 的布局
         * 去扫 3-bit 的数据 (或反过来) —— 放在 0x18 之前, 让"切基址"是最后
         * 一步, 中间态最多持续一个 pair。 */
        bcm_apply((int)g_disp.bpp3);
        if (g_ring_bcm) {              /* 圈级 BCM: 本圈用哪个位平面的权重 */
            unsigned oe = RING_OE[g_ring_idx % 3u];
            reg_wr(REG_CFG_MISC, CFG_SUB10_BASE | (oe << 8));
            g_ring_idx++;
        }
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
        /* 翻页间隔 -> 换算成"隔了几圈"。rev1 = 每圈翻中 (目标), rev2+ = 赔了
         * 至少一整圈。⚠ 这两个数才是"翻中率", flip/s 会被转速抖动稀释。 */
        {
            long now_us = mono_us();
            unsigned long ups = g_ups_q8;
            if (last_flip_us && ups >= 16) {
                long gap = now_us - last_flip_us;
                long rev_us = (long)((ups * engine_n_slices()) >> 8);
                g_w_gap_us += (unsigned long)gap;
                if ((unsigned long)gap > g_w_gap_max) g_w_gap_max = (unsigned long)gap;
                g_w_gap_n++;
                if (rev_us > 0 && gap < rev_us * 3 / 2) g_w_rev1++;
                else g_w_rev2++;
            }
            last_flip_us = now_us;
        }
        if (base_b || g_disp.fold_a || g_disp.bpp3)
            logts("FLIP gen=%u bank=%d win=%d n=%u stride=0x%x bpp=%u "
                  "A=0x%08x B=0x%08x fold=%u%s",
                  g_consumed_gen, idle, win, g_disp.n_slices, g_disp.stride,
                  g_disp.bpp3 ? 3u : 1u, base_a, base_b, g_disp.fold_a,
                  forced ? " FORCED" : "");
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
        /* 🔴 收包耗时单独计时。老 DIAG 把 dec/cpy/wait 拆得很细, 却唯独没量
         * 这一段 —— 而实测它比 dec + cpy 加起来还大, "链路不是瓶颈"这个前提
         * 因此一直没被证伪。hdr = 等下一帧帧头 (发送端还没发出来),
         * body = 收帧体 (真正的投递时间)。 */
        long t_hdr = mono_us();
        int r = recv_full(fd, &h, sizeof h);
        t_hdr = mono_us() - t_hdr;
        if (r <= 0) return r;

        /* 头校验 (v3.1): n_slices 是权威, raw_len 必须与它自洽; 不再硬比
         * PVS_N_SLICES。未知 flag 仍用位掩码拒掉 (给未来留位)。
         * 压缩位互斥 (RLE/ZLIB/LZ4 最多置一位): v3.3 加 LZ4 后从「两两比较」
         * 改成数位数, 免得以后再加编解码器时漏掉某一对组合。
         * 注意用 uint32_t 算长度: 720*0x3000 = 8847360, u16 会溢出。 */
        uint32_t n_slices = h.n_slices;
        /* 🔴 v3.4: 片距**从 flag 推**, 不再是常量 —— 3-bit 一片是 3 个位平面
         * = 0x9000。片数上限也跟着变 (字节上限 PVS_FRAME_RAW_MAX 才是硬的:
         * 一个 DDR bank / 一个 staging 缓冲就那么大):
         *   1-bit 720 片 * 0x3000 = 8847360 B
         *   3-bit 240 片 * 0x9000 = 8847360 B  (整除)
         * 用 PVS_N_SLICES_MAX_F(flags) 一次覆盖两种色深; need_raw 用 u32 算
         * (720*0x3000 已经溢出 u16, 3-bit 更大)。 */
        uint32_t stride   = PVS_STRIDE(h.flags);
        uint32_t need_raw = n_slices * stride;
        uint32_t codec_bits = h.flags & PVS_FLAGS_CODEC;
        if (memcmp(h.magic, PVS_MAGIC, 4) != 0 ||
            n_slices < 1 || n_slices > PVS_N_SLICES_MAX_F(h.flags) ||
            h.raw_len != need_raw               ||
            need_raw > (uint32_t)PVS_FRAME_RAW_MAX ||
            h.comp_len == 0 || h.comp_len > COMP_LEN_MAX ||
            (h.flags & ~PVS_FLAGS_KNOWN) != 0   ||
            (codec_bits & (codec_bits - 1)) != 0) {
            logts("NAK: bad header (magic=%.4s comp=%u raw=%u n=%u flags=0x%x "
                  "stride=0x%x want_raw=%u max_n=%u)",
                  h.magic, h.comp_len, h.raw_len, h.n_slices, h.flags,
                  stride, need_raw, PVS_N_SLICES_MAX_F(h.flags));
            send_byte(fd, PVS_NAK);
            return -1;
        }

        /* ---- 面拆分 ------------------------------------------------------
         * 🔴 2026-08-20: 这里以前写死了 PVS_N_SLICES(360):
         *      n_a = FOLD_A ? 180 : 360;  if (n_slices != n_a + 360) NAK;
         *   ⇒ **双面帧只认 720 或 540**, 任何别的槽数当场 NAK。3-bit 走的是
         *   每面 60 槽 (双面 120 片), 槽数优化那条线也要 90/面 —— 都会被这两行
         *   拦死, 而且报的是"n_slices 不对"这种指向完全错误的信息。
         *   改成从 n_slices **推**, 用的是排布本身的恒等式:
         *      面B 永远是一整面 nB;  面A = FOLD_A ? nB/2 : nB
         *      不折叠: n = 2nB          -> nA = n/2
         *      折叠:   n = nB/2 + nB    -> nA = n/3   (nB = 2n/3)
         *   整除性就是自洽校验 (拆错了会把面B 的数据当面A 写出去)。
         *   老帧逐字节不变: 720 -> 360+360, fold540 -> 180+360, 与写死时同值。 */
        uint32_t n_a = n_slices, face_b_off = 0;
        if (h.flags & PVS_FLAG_DUAL_FACE) {
            uint32_t div = (h.flags & PVS_FLAG_FOLD_A) ? 3u : 2u;
            if (n_slices % div) {
                logts("NAK: DUAL_FACE%s n_slices=%u 不是 %u 的整数倍 "
                      "(面B=一整面 nB, 面A=%s ⇒ n_slices=%s)",
                      (h.flags & PVS_FLAG_FOLD_A) ? "|FOLD_A" : "", n_slices, div,
                      div == 3 ? "nB/2" : "nB", div == 3 ? "1.5*nB" : "2*nB");
                send_byte(fd, PVS_NAK);
                return -1;
            }
            n_a = n_slices / div;
            face_b_off = n_a * stride;
        } else if (h.flags & PVS_FLAG_FOLD_A) {
            /* 单面折叠: 载荷 = 半圈。半圈是多少片取决于**引擎每圈片数**
             * (RTL 的 fold 用 n_slices_r>>1, 不是写死 180), 而引擎那个数不由
             * 帧决定 —— 所以协议层没有可校验的常量, 这里只做提示不 NAK。
             * (老代码写死 n_slices==180, 同样会拦死 3-bit 的 30 片折叠面。) */
            static uint32_t noted;            /* 同一种几何只提示一次, 不刷屏 */
            uint32_t n_eng = engine_n_slices();
            if (n_slices * 2u != n_eng && noted != n_slices) {
                noted = n_slices;
                logts("NOTE: FOLD_A 单面 n_slices=%u, 而引擎每圈 %u 片 "
                      "(PL 按 idx-引擎片数/2 折) —— 要么帧不是半圈, 要么引擎的 "
                      "0x10[31:16] 没按这个几何设", n_slices, n_eng);
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

        /* ---- 载荷排布 (protocol.h v3.3) -----------------------------------
         *   单面:    [压缩流]                            comp_len = 流长度
         *   双面:    [u32 LE comp_len_A][面A 流][面B 流]  comp_len 含这 4 字节
         *   MSTREAM: [u32 n][n×{u32 clen,u32 nsl}][流 0..n-1]  (取代上面的 4B 前缀)
         * 未压缩帧直接收进 g_wr.buf (前缀/流表另收到小缓冲, 剩下的连着收就正好
         * 是解压后的排布)。
         */
        /* comp = 本帧的**压缩 flag 位** (0 = raw), 原样传给 face_job_t.codec */
        int comp = (int)codec_bits;
        uint32_t body_len = h.comp_len;      /* 去掉前缀/流表后的净载荷 */
        uint32_t clen_a = 0, clen_b = 0;
        uint32_t nstr = 0;                   /* >0 = 走 MSTREAM 流表 */
        uint32_t s_clen[PVS_MAX_STREAMS], s_nsl[PVS_MAX_STREAMS];

        if (h.flags & PVS_FLAG_MSTREAM) {
            /* 流表: 先收 4B 条数, 再收 n*8 的表体。两个求和校验一个都不能省 ——
             * 少了它们, 一条被截断的载荷会解出半帧垃圾还照样 ACK。 */
            uint8_t tbl[PVS_MAX_STREAMS * MSTR_ENT_LEN];
            uint8_t n4[4];
            if (h.comp_len < 4u + MSTR_ENT_LEN + 1u) {
                logts("NAK: MSTREAM comp_len=%u 装不下流表", h.comp_len);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            r = recv_full(fd, n4, 4);
            if (r <= 0) return r;
            nstr = (uint32_t)n4[0] | ((uint32_t)n4[1] << 8) |
                   ((uint32_t)n4[2] << 16) | ((uint32_t)n4[3] << 24);
            if (nstr < 1 || nstr > PVS_MAX_STREAMS) {
                logts("NAK: MSTREAM n_streams=%u 越界 (1..%d)", nstr, PVS_MAX_STREAMS);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            uint32_t tlen = nstr * MSTR_ENT_LEN;
            if (h.comp_len < 4u + tlen + nstr) {   /* 每条流至少 1 字节 */
                logts("NAK: MSTREAM comp_len=%u < 4+%u+%u", h.comp_len, tlen, nstr);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            r = recv_full(fd, tbl, tlen);
            if (r <= 0) return r;
            uint64_t sum_c = 0, sum_s = 0;
            int bad_ent = 0;
            for (uint32_t i = 0; i < nstr; i++) {
                const uint8_t *e = tbl + i * MSTR_ENT_LEN;
                s_clen[i] = (uint32_t)e[0] | ((uint32_t)e[1] << 8) |
                            ((uint32_t)e[2] << 16) | ((uint32_t)e[3] << 24);
                s_nsl[i]  = (uint32_t)e[4] | ((uint32_t)e[5] << 8) |
                            ((uint32_t)e[6] << 16) | ((uint32_t)e[7] << 24);
                if (s_clen[i] == 0 || s_nsl[i] == 0) bad_ent = 1;
                /* raw 帧: 每条流就是原始数据, 压缩长度必须 == 解压长度
                 * (stride 随色深变, 3-bit 时每片 0x9000) */
                if (!comp && s_clen[i] != s_nsl[i] * stride)
                    bad_ent = 1;
                sum_c += s_clen[i];
                sum_s += s_nsl[i];
            }
            body_len = h.comp_len - 4u - tlen;
            if (bad_ent || sum_c != body_len || sum_s != n_slices) {
                logts("NAK: MSTREAM 流表不自洽 (n=%u Σclen=%llu want %u, "
                      "Σn_slices=%llu want %u%s)", nstr,
                      (unsigned long long)sum_c, body_len,
                      (unsigned long long)sum_s, n_slices,
                      bad_ent ? ", 有空流/raw 长度不符" : "");
                send_byte(fd, PVS_NAK);
                return -1;
            }
        } else if (face_b_off) {
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

        long t_body = mono_us();
        r = recv_full(fd, comp ? cbuf : g_wr.buf, body_len);
        if (r <= 0) return r;
        t_body = mono_us() - t_body;
        g_w_hdr_us  += (unsigned long)t_hdr;
        g_w_body_us += (unsigned long)t_body;
        if ((unsigned long)t_hdr  > g_w_hdr_max)  g_w_hdr_max  = (unsigned long)t_hdr;
        if ((unsigned long)t_body > g_w_body_max) g_w_body_max = (unsigned long)t_body;
        g_w_rcv_n++;

        /* ---- 消融: --diag-rxonly 收完就 ACK, 后面全跳过 ------------------- */
        if (g_rxonly) {
            g_st_rx++;
            if (send_byte(fd, PVS_ACK) != 0) return -1;
            if (g_stop) return -1;
            continue;
        }

        /* ---- 翻页相位锁定: 停在"解完 + 拷完正好赶上翻页窗"的相位再开解 ----
         * 必须放在收包**之后**、解码之前: 放在收包之前会让 socket 不被抽干,
         * 发送端被 TCP 背压顶住, 下一帧更晚 —— 越锁越差。 */
        long t_gate = phase_gate();

        g_log_stride = stride;              /* decode plan 日志按本帧片距换算 */
        long dec_t0 = mono_us();
        /* 🔴 诊断口径: **每个解码核的墙钟**, 不是"每一面"。
         * MSTREAM 之后流数可变(1..16), "面A/面B"这个概念不再成立; 而决定
         * 帧率的本来就是两个核的 makespan。合并两条分支时这里出过错 ——
         * 诊断分支写的是 ja.us/jb.us(固定两个 job), 而 lz4 分支已经把两 job
         * 换成了可变长 jobs[] + dec_plan, 文本能合但编译不过。 */
        long core0_us = 0, core1_us = 0;

        if (nstr || face_b_off) {
            /* 多条独立流 -> 多个 job -> dec_plan 按片数摊到两个核。
             * 每条流各自 XOR 参考帧里**同偏移**的那一段 (参考帧布局已在头校验
             * 里确认与本帧一致); 流的边界是片边界, 逐字节 XOR 天然对齐。
             * nstr==0 时就是老的「按面切两条」, 等价于 nstr==2 的特例。 */
            const uint8_t *ref = (h.flags & PVS_FLAG_DELTA) ? g_prev : NULL;
            face_job_t jobs[PVS_MAX_STREAMS];
            uint32_t n = nstr ? nstr : 2u;
            uint32_t cl[2] = { clen_a, clen_b };
            uint32_t dl[2] = { face_b_off, h.raw_len - face_b_off };
            uint32_t soff = 0, doff = 0;
            memset(jobs, 0, sizeof jobs);
            for (uint32_t i = 0; i < n; i++) {
                uint32_t clen = nstr ? s_clen[i] : cl[i];
                uint32_t dlen = nstr ? s_nsl[i] * stride : dl[i];
                jobs[i].src     = comp ? cbuf + soff : NULL;
                jobs[i].src_len = clen;
                jobs[i].dst     = g_wr.buf + doff;
                jobs[i].dst_len = dlen;
                jobs[i].prev    = ref ? ref + doff : NULL;
                jobs[i].codec   = (uint32_t)comp;
                soff += clen;
                doff += dlen;
            }
            if (dec_run(jobs, (int)n) != 0) {
                char why[256];
                dec_errline(jobs, (int)n, why, sizeof why);
                logts("NAK: %u-stream decode failed (%s)", n, why);
                send_byte(fd, PVS_NAK);
                return -1;
            }
            /* dec_plan 是纯函数, 这里再算一次拿分组, 把各核负责的流累加起来 */
            {
                int wf[DEC_WORKERS], wc[DEC_WORKERS];
                long sum[DEC_WORKERS] = { 0 };
                dec_plan(jobs, (int)n, wf, wc);
                for (int w = 0; w < DEC_WORKERS; w++)
                    for (int i = wf[w]; i < wf[w] + wc[w]; i++)
                        sum[w] += jobs[i].us;
                core0_us = sum[0];
                core1_us = DEC_WORKERS > 1 ? sum[1] : 0;
            }
        } else {
            /* 单面: 与 v2 完全同一条路径 (不过线程池, 不多一次交接) */
            if (h.flags & PVS_FLAG_LZ4) {
                /* raw block: dstCapacity = hdr.raw_len (流里不带原长) */
                int n = LZ4_decompress_safe((const char *)cbuf,
                                            (char *)g_wr.buf,
                                            (int)h.comp_len, (int)h.raw_len);
                if (n < 0 || (uint32_t)n != h.raw_len) {
                    logts("NAK: lz4 decode failed (rc=%d want=%u; 载荷必须是 "
                          "raw block, 不是 .lz4 帧格式)", n, h.raw_len);
                    send_byte(fd, PVS_NAK);
                    return -1;
                }
            } else if (h.flags & PVS_FLAG_ZLIB) {
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
        g_w_dec_us += (unsigned long)dec_us;
        if ((unsigned long)dec_us > g_w_dec_max) g_w_dec_max = (unsigned long)dec_us;
        g_w_c0_us += (unsigned long)core0_us;
        g_w_c1_us += (unsigned long)core1_us;
        g_w_dec_n++;
        ema_add(&g_ema_dec_us, dec_us);
        /* 服务时间 = **收帧体 + 解码**。故意不含两段:
         *  - gate 自己等的那段 (否则自我实现: 等得越久 svc 越大, 越容易被
         *    保险 2 误判成"装不下"而把锁关掉)。
         *  - t_hdr, 也就是阻塞在"等下一帧帧头"上的时间 —— 那是**发送端还没发**
         *    的空闲, 不是板子的活。踩过: 发送端 --fps 10 时 hdr 自动补到
         *    ~(100ms - dec), svc 于是恒等于一圈, 保险 2 永远判 over, 锁永远
         *    不生效, 而且怎么调都看不出差别。 */
        ema_add(&g_ema_svc_us, t_body + dec_us);
        (void)t_gate; (void)t_hdr;

        uint32_t crc = 0;
        if (g_crc_on)
            crc = crc32(0L, g_wr.buf, h.raw_len);

        /* 发布给 flip 线程 + 记参考帧; 旧 ready 没被消费就顶替 (丢帧计数) */
        g_wr.raw_len    = h.raw_len;
        g_wr.n_slices   = n_slices;
        g_wr.stride     = stride;
        g_wr.face_b_off = face_b_off;
        g_wr.fold_a     = (h.flags & PVS_FLAG_FOLD_A) ? 1u : 0u;
        g_wr.bpp3       = (h.flags & PVS_FLAG_3BIT) ? 1u : 0u;
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
                  dec_tag(nstr, face_b_off));
        else
            logts("FRAME seq=%u n=%u comp=%u flags=0x%x dec=%.1fms%s",
                  seq++, n_slices, h.comp_len, h.flags, dec_us / 1000.0,
                  dec_tag(nstr, face_b_off));
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
        "  --fake-slices N 引擎每圈片数, 只在 --fake 下有意义 (default %d;\n"
        "                 3-bit 的 60 槽几何在台面上试时要改它)\n"
        "  --oe-w W1,W2   3-bit BCM 的中位/高位 OE 沿数 (default %u,%u; 低位平面\n"
        "                 用 0x0C sub10 的 oe_window, 由 pov_boot.sh 固化成 %u)\n"
        "  --crc          crc32 every decoded frame + log it (costs 11-18 ms\n"
        "                 per frame on the A9; debug only, default off)\n"
        "  --flip-window  single = flip near slice 0 only (default);\n"
        "                 dual   = also near slice 180 (dual-panel, 26 pps)\n"
        "  --idle-anim F  无客户端时循环播放 F (anim.pvs 容器); 有推流自动让位\n"
        "  --idle-fps N   空闲动画帧率 (default 8)\n"
        "  --decode       parallel = 双面两条流分别解到 CPU0/CPU1 (default);\n"
        "                 serial   = 单核串行解 (对拍 / 出问题时退回)\n"
        "  --diag on|off  每秒多打一行 DIAG (窗内瞬时 rx/flip 帧率 + 一帧开销\n"
        "                 拆成 dec/A/B/cpy/wait), default on\n"
        "  --diag-nocopy  消融实验: 跳过 staging->DDR bank 的 memcpy (画面不再\n"
        "  --diag-rxonly  消融实验: 收完帧体立刻 ACK, 不解码/不发布/不翻页。\n"
        "  --rcvbuf N     SO_RCVBUF 字节数。**默认 0 = 不设**, 让内核自动调窗;\n"
        "                 设了反而会关掉 DRS 把接收窗钉死 (见 g_rcvbuf 注释),\n"
        "                 实测吞吐掉到 1/3。非 0 只用于 A/B 对照。\n"
        "                 用来把 DIAG 的 body 拆成「链路本身」与「板端处理挤慢的」\n"
        "                 更新!), 量那次拷贝对解码吞吐的真实代价。别在生产用\n"
        "  --phase-lock   on = 收完帧体后按转子相位停一小会儿再开解, 让\n"
        "                 dec+cpy 正好赶在翻页窗前结束 (每圈翻中一次)。\n"
        "                 引擎没转 / 一帧装不进一圈时自动放行, 见 PHASE 行的\n"
        "                 lock= 字段 (off/nospin/over/late/wait/tmo)。\n"
        "                 ⚠ default off, 而且**实测没用**: 本板的损失全在收包,\n"
        "                 相位可回收的量实测为 0 (drop=0/668 帧)。细节见\n"
        "                 pov_rxd.c 里 phase_gate 上方那段注释\n"
        "  --no-rmem-fix  对照实验: 不抬 net.core.rmem_max (生产别用)\n",
        argv0, PVS_PORT, FRAME_PHYS_DEFAULT, REG_PHYS_DEFAULT, PVS_N_SLICES,
        OE_W1_DEFAULT, OE_W2_DEFAULT, OE_W0_3BIT_HINT);
}

int main(int argc, char **argv)
{
    int port = PVS_PORT;
    uint32_t frame_phys = FRAME_PHYS_DEFAULT;
    uint32_t reg_phys = REG_PHYS_DEFAULT;
    double fake_rps = 0.0;
    uint32_t fake_slices = PVS_N_SLICES;    /* --fake 时写进 0x10[31:16] 的片数 */

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--base") && i + 1 < argc)
            frame_phys = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--regs") && i + 1 < argc)
            reg_phys = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--fake") && i + 1 < argc)
            fake_rps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--ring-bcm")) g_ring_bcm = 1;
        else if (!strcmp(argv[i], "--flip-timeout") && i + 1 < argc)
            g_flip_timeout_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fake-slices") && i + 1 < argc) {
            long v = strtol(argv[++i], NULL, 0);
            if (v < 1 || v > 65535) { usage(argv[0]); return 2; }
            fake_slices = (uint32_t)v;
        }
        else if (!strcmp(argv[i], "--crc"))
            g_crc_on = 1;
        else if (!strcmp(argv[i], "--swap-faces")) g_swap_faces = 1;
        else if (!strcmp(argv[i], "--oe-w") && i + 1 < argc) {
            /* --oe-w W1,W2: 3-bit BCM 的中位/高位平面 OE 沿数 (低位是 oe_window)。
             * RTL 内箝 [2,187], 这里先箝一遍免得写进去的和生效的不是一个数。 */
            unsigned w1 = OE_W1_DEFAULT, w2 = OE_W2_DEFAULT;
            if (sscanf(argv[++i], "%u,%u", &w1, &w2) != 2) { usage(argv[0]); return 2; }
            if (w1 < OE_W_MIN) w1 = OE_W_MIN;
            if (w1 > OE_W_MAX) w1 = OE_W_MAX;
            if (w2 < OE_W_MIN) w2 = OE_W_MIN;
            if (w2 > OE_W_MAX) w2 = OE_W_MAX;
            g_oe_w1 = w1; g_oe_w2 = w2;
        }
        else if (!strcmp(argv[i], "--idle-anim") && i + 1 < argc) idle_path = argv[++i];
        else if (!strcmp(argv[i], "--idle-fps")  && i + 1 < argc) g_idle_fps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--flip-window") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "dual")) g_win_dual = 1;
            else if (strcmp(m, "single")) { usage(argv[0]); return 2; }
        }
        else if (!strcmp(argv[i], "--diag-nocopy")) g_nocopy = 1;
        else if (!strcmp(argv[i], "--diag-rxonly")) g_rxonly = 1;
        else if (!strcmp(argv[i], "--rcvbuf") && i + 1 < argc) g_rcvbuf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--no-rmem-fix")) g_no_rmem_fix = 1;
        else if (!strcmp(argv[i], "--phase-lock") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "on")) g_lock = 1;
            else if (strcmp(m, "off")) { usage(argv[0]); return 2; }
        }
        else if (!strcmp(argv[i], "--diag") && i + 1 < argc) {
            const char *m = argv[++i];
            if (!strcmp(m, "off")) g_diag = 0;
            else if (strcmp(m, "on")) { usage(argv[0]); return 2; }
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
    /* v3.4 3-bit: 把"能收什么"和"谁负责 oe_w0"一次说清, 免得上板时靠猜 */
    logts("3-bit: PVS_FLAG_3BIT=0x%x -> stride 0x%x, n_slices 1..%u "
          "(1-bit: 0x%x / 1..%d); bpp_mode+oe_w1/oe_w2 -> 0x0C sub01, "
          "本进程按帧切; oe_w0 = 0x0C sub10 的 oe_window(**本进程不写**, "
          "3-bit 要 %u, pov_boot.sh 现固化的是 111)",
          (unsigned)PVS_FLAG_3BIT, (unsigned)PVS_SLICE_STRIDE_3BIT,
          (unsigned)PVS_N_SLICES_MAX_3BIT, (unsigned)PVS_SLICE_STRIDE,
          PVS_N_SLICES_MAX, OE_W0_3BIT_HINT);
    logts("BCM 权重: oe_w1=%u oe_w2=%u (默认 %u/%u, --oe-w 改); "
          "影子初值=无效 -> 第一帧无条件写 0x0C sub01, 之后每 %u 次重申一遍",
          g_oe_w1, g_oe_w2, OE_W1_DEFAULT, OE_W2_DEFAULT,
          (unsigned)BCM_REASSERT_EVERY);
    logts("engine STATUS=0x%08x POV_CTRL=0x%08x",
          reg_rd(REG_STATUS), reg_rd(REG_POV_CTRL));

    /* start on bank A, 面B 基址清 0 (= PL 回落到 0x18 的单面老行为);
     * POV_CTRL is left alone unless --fake */
    reg_wr(REG_SLICE_BASE_B, 0);
    reg_wr(REG_SLICE_BASE, g_bank_phys[0]);
    if (fake_rps > 0.0) {
        /* --fake-slices: 引擎每圈片数。默认 360 = 老行为逐位不变; 3-bit 的
         * 60 槽几何要在台面上无电机试, 就得能把它改掉 (帧里的 hdr.n_slices
         * 是"载荷有几片", 与引擎一圈几片是两回事, 见 0x10 的注释)。 */
        uint32_t period = (uint32_t)((double)ACLK_HZ / (fake_rps * fake_slices) + 0.5);
        reg_wr(REG_FAKE_PERIOD, period);
        /* pov_ctrl_write 会先保证 PHASE_B < n_slices 再落 0x10 */
        pov_ctrl_write(fake_slices, (1u << 1) | 1u);
        logts("fake-spin: %.2f rps x %u 片/圈 -> fake_period=%u ticks/slice, "
              "POV_CTRL set", fake_rps, fake_slices, period);
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
        /* ---- 接收缓冲: 默认**什么都不做** ----------------------------------
         * 🔴 2026-08-06 定案, 与 2026-08-05 的结论**相反** —— 老结论是错的:
         *   老代码为了"一帧 272 KB 得装得下"去 SO_RCVBUFFORCE 768 KB。但
         *   setsockopt(SO_RCVBUF) 的副作用是给 socket 置上 SOCK_RCVBUF_LOCK,
         *   内核的接收窗自动放大 (DRS, tcp_rcv_space_adjust) **从此整个关闭**,
         *   窗口被钉死在握手时按初始 rcvbuf 算出的 window_clamp 上, 再也长不大。
         *   吞吐 = 在途窗口 / RTT; 这条 WiFi 链路 RTT 约 30 ms, 钉死的窗正好
         *   给出 2-3 MB/s —— 就是"收包只有 3 MB/s"的全部真身。
         *   实测 (同一个 povstream, 同一条链路, 交错跑):
         *     设 768 KB (老行为)  2.76 / 2.12 / 1.73 MB/s
         *     不设   (本默认)     7.75 / 6.59 / 4.44 MB/s
         *   同一时刻, 一个什么 socket 选项都没设的 **Python** 最小 sink 也有
         *   6-8 MB/s, 比 C 写的 pov_rxd 快 3 倍 —— 差别只在这一段。
         * ⚠ "一帧比接收缓冲大" 从来不是问题: TCP 是字节流, recv_full 本来就
         *   分多次收。决定吞吐的是**在途窗口**, 不是能不能一次缓存下整帧。
         * 保留 --rcvbuf N 只是为了能把老行为 A/B 复现出来。 */
        if (g_rcvbuf > 0) {
            int rbuf = g_rcvbuf;
            const char *how = "SO_RCVBUF";
            /* 只有真要设 rcvbuf 时才有必要抬 net.core.rmem_max (出厂 176 KB,
             * 不抬的话 SO_RCVBUFFORCE 会失败而悄悄砍半)。默认路径不设 rcvbuf,
             * 也就不需要动这个 sysctl。 */
            if (!g_no_rmem_fix) {
                long want = (long)g_rcvbuf * 2L;   /* 内核记账要 x2 */
                FILE *f = fopen("/proc/sys/net/core/rmem_max", "r+");
                long cur = 0;
                if (f && fscanf(f, "%ld", &cur) == 1 && cur < want) {
                    rewind(f);
                    if (fprintf(f, "%ld", want) <= 0)
                        logts("WARN: 抬 net.core.rmem_max 失败 (%s), 当前 %ld B",
                              strerror(errno), cur);
                }
                if (f) fclose(f);
            }
            if (setsockopt(cfd, SOL_SOCKET, SO_RCVBUFFORCE, &rbuf, sizeof rbuf) == 0)
                how = "SO_RCVBUFFORCE";
            else if (setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof rbuf) != 0)
                logts("WARN: setsockopt SO_RCVBUF %d failed (%s)", rbuf,
                      strerror(errno));
            int rbuf_eff = 0;
            socklen_t rlen = sizeof rbuf_eff;
            if (getsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf_eff, &rlen) == 0)
                logts("SO_RCVBUF: requested %d B via %s -> effective %d B "
                      "⚠ 这会关掉内核接收窗自动放大, 只该用于 A/B 对照",
                      rbuf, how, rbuf_eff);
        } else {
            int rbuf_eff = 0;
            socklen_t rlen = sizeof rbuf_eff;
            getsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf_eff, &rlen);
            logts("接收缓冲: 不设 SO_RCVBUF, 交给内核 DRS 自动放大 "
                  "(此刻 %d B, 上限见 net.ipv4.tcp_rmem[2])", rbuf_eff);
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
