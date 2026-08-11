/* oeprobe.c — FS03 只读: 用 OE 行节拍反推面板跑在 50Mbps 还是 25Mbps 降级。
 *
 * 为什么要用 C 重写 (Python 版拿到 INCONCLUSIVE):
 *   Python 每次读 AXI-Lite 要 ~1.7 us, 而行周期本身才 ~8 us
 *   => 每段只有 3-5 个样本, 量化误差 ±20-33%, 量具量不了要量的东西。
 *
 * 两处比 Python 版硬:
 *   1) 主判据改成「固定时间窗内数跳变」: row = 2*T/n_edges。
 *      对成千上万行取平均, 不受单段小整数量化影响。
 *      (Python 版用的是 run-length 中位数, 3-5 个样本的中位数噪声极大。)
 *   2) 自检门槛从「每段 >=3 样本」提到 **>=10 样本**。不够就拒答 ——
 *      采样过慢会漏掉跳变, 而这个误差是**单向的**(把 fast 报成 slow),
 *      正好偏向我要证的结论, 所以宁可不答。
 *
 * 全程只读: O_RDONLY + PROT_READ, 只读状态口 0x00。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

#define BASE   0x40010000UL
static double SECS = 2.0;   /* argv[1] 可覆盖 (扫描时用短窗) */

static double fabs_(double x) { return x < 0 ? -x : x; }

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
    if (argc > 1) SECS = atof(argv[1]);
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile unsigned char *p = mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, BASE);
    if (p == MAP_FAILED) { perror("mmap"); return 1; }

    unsigned st0 = *(volatile unsigned *)p;
    printf("status(0x00) = 0x%08X  bit7 ddr_slow=%u  bit4 auto_en=%u  bit9 pov_en=%u\n",
           st0, (st0 >> 7) & 1, (st0 >> 4) & 1, (st0 >> 9) & 1);
    if (!((st0 >> 4) & 1)) {
        printf("VERDICT: INCONCLUSIVE -- auto_en=0, OE not moving\n");
        return 0;
    }

    /* 采样: 尽可能快地读 byte0, 记录跳变数与低电平占比 */
    size_t cap = 8u << 20, n = 0, edges = 0, nlow = 0;
    unsigned char *buf = malloc(cap);
    if (!buf) { perror("malloc"); return 1; }

    double t0 = now(), t1;
    do {
        for (int i = 0; i < 4096 && n < cap; i++) buf[n++] = p[0];
        t1 = now();
    } while (t1 - t0 < SECS && n < cap);

    double dt = (t1 - t0) / n;                    /* s / sample */
    int prev = (buf[0] >> 3) & 1;
    for (size_t i = 0; i < n; i++) {
        int b = (buf[i] >> 3) & 1;
        if (b != prev) { edges++; prev = b; }
        if (!b) nlow++;
    }

    printf("samples=%zu  dt=%.4f us  window=%.3f s  edges=%zu\n",
           n, dt * 1e6, t1 - t0, edges);
    if (edges < 2000) {
        printf("VERDICT: INCONCLUSIVE -- too few edges (%zu)\n", edges);
        return 0;
    }

    /* 主判据: 一行 = 一低 + 一高 = 2 次跳变 */
    double row_us  = 2.0 * (t1 - t0) / edges * 1e6;
    double duty_lo = (double)nlow / n;
    double low_us  = row_us * duty_lo;
    double spr     = row_us / (dt * 1e6);         /* samples per row */

    printf("  samples/row = %.1f   OE-low duty = %.3f\n", spr, duty_lo);
    /* 🔴 仪器自检: 每段至少 10 个样本, 否则拒答 (误差单向, 见文件头) */
    if (spr * duty_lo < 10.0 || spr * (1 - duty_lo) < 10.0) {
        printf("VERDICT: INCONCLUSIVE -- sampling too slow "
               "(%.1f lo / %.1f hi samples per run, need >=10)\n",
               spr * duty_lo, spr * (1 - duty_lo));
        return 0;
    }

    printf("  => row %.2f us, OE-low %.2f us, screen(54 rows) %.1f us, 2D %.0f Hz\n",
           row_us, low_us, row_us * 54, 1e6 / (row_us * 54));

    /* aclk=50MHz, rows=54; fast: 195+max(0,oe-111) / slow: 387+max(0,2oe-303) */
    const char *nm[] = {"FAST(50Mbps)+oe=111", "FAST(50Mbps)+oe=187",
                        "SLOW(25Mbps)+oe=111", "SLOW(25Mbps)+oe=187"};
    const double er[] = {3.90, 5.42, 7.74, 9.16};
    const double el[] = {2.22, 3.74, 4.44, 7.48};
    int best = 0; double berr = 1e9;
    for (int i = 0; i < 4; i++) {
        double e = fabs_(er[i] - row_us) / er[i] + fabs_(el[i] - low_us) / el[i];
        if (e < berr) { berr = e; best = i; }
    }
    double rerr = (er[best] > row_us ? er[best] - row_us : row_us - er[best]) / er[best];
    printf("  closest: %s (expect row %.2f / low %.2f), rel err %.1f%%\n",
           nm[best], er[best], el[best], rerr * 100);
    printf("VERDICT: %s\n", rerr < 0.20 ? nm[best] :
           "INCONCLUSIVE -- matches no known combination");
    return 0;
}
