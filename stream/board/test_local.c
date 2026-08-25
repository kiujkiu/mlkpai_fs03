/*
 * test_local.c - x86 loopback test for pov_rxd v2 (SIM_NO_DEVMEM build).
 *
 * Spawns ./pov_rxd_sim (--crc --flip-window dual) on a local port, connects,
 * and exercises:
 *   - all four codecs (raw / zero-run RLE / zlib / LZ4 raw block), ACK-paced
 *   - DELTA chain: zlib keyframe + 3x DELTA|ZLIB, asserts the daemon's
 *     logged crc32 of every reconstructed raw frame == crc of what the
 *     PC-side raw was (proves prev_acked_raw ^ decoded reconstruction)
 *   - two frames back-to-back with no ACK wait (ready-buffer overwrite /
 *     drop path; both must still ACK - RX is decoupled from the flip)
 *   - DELTA|LZ4: 压缩位与 DELTA 位正交, 组合必须与 DELTA|ZLIB 一样成立
 *   - NAK cases, each on a fresh connection (daemon closes after NAK):
 *     garbage magic / unknown flag bit / ZLIB|LZ4 同时置位 (压缩位互斥) /
 *     DELTA as first frame (no reference) / DELTA first after reconnect
 *     (reference must reset)
 *   - at least one FLIP line (flip thread alive, window logic ran)
 *
 * Exit 0 = pass.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <zlib.h>
#include <lz4.h>
#include <lz4hc.h>

#include "../protocol.h"

#define LZ4_HC_LEVEL 9        /* = povstream.py 的 DEFAULT_LZ4_LEVEL */

#define TEST_PORT 9517
#define MAX_FRAMES 32

static uint32_t sent_crc[MAX_FRAMES];
static int n_sent = 0;

/* deterministic frame: mostly zeros (like real slice data) + seeded noise */
static void gen_frame_n(uint8_t *f, uint32_t seed, uint32_t len)
{
    memset(f, 0, len);
    uint32_t s = seed * 2654435761u + 1;
    for (int i = 0; i < 40000; i++) {
        s = s * 1664525u + 1013904223u;
        uint32_t pos = (s >> 8) % len;
        uint8_t val = (uint8_t)(s & 0xff);
        f[pos] = val ? val : 0x5a;   /* keep literals nonzero for RLE */
    }
}

static void gen_frame(uint8_t *f, uint32_t seed) { gen_frame_n(f, seed, PVS_FRAME_RAW); }

/* v3.4: 期望的 (面A 字节数, 面B 字节数) 集合。板端把 nA 从 n_slices 推出来
 * (FOLD_A ? n/3 : n/2), 推错了整帧 crc 照样对得上 —— 只有 SIM 的 bankcrc 行
 * 里的两个 len 会露馅, 所以这里把每一帧的期望面长记下来, 收尾时逐行核对。 */
#define MAX_PAIRS 64
static uint32_t exp_pair[MAX_PAIRS][2];
static int n_pairs = 0;
static void want_faces(uint32_t la, uint32_t lb)
{
    for (int i = 0; i < n_pairs; i++)
        if (exp_pair[i][0] == la && exp_pair[i][1] == lb) return;
    if (n_pairs < MAX_PAIRS) { exp_pair[n_pairs][0] = la; exp_pair[n_pairs][1] = lb; n_pairs++; }
}

/* zero-run RLE per protocol.md: 0x00 escape + run:u16le zeros; literals raw */
static size_t rle_encode(const uint8_t *src, size_t n, uint8_t *dst)
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

static int send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) { if (n < 0 && errno == EINTR) continue; return -1; }
        p += n; len -= (size_t)n;
    }
    return 0;
}

/* send one frame; `prev` != NULL + DELTA flag -> payload = zlib(prev^raw)
 * (or bare prev^raw). Records crc of RAW (what the daemon must rebuild). */
static int send_frame_n(int fd, const uint8_t *raw, const uint8_t *prev,
                        uint16_t flags, uint32_t n_slices,
                        uint8_t *xbuf, uint8_t *scratch)
{
    pvs_hdr_t h;
    uint32_t raw_len = n_slices * PVS_STRIDE(flags);   /* v3.4: 片距随色深 */
    memcpy(h.magic, PVS_MAGIC, 4);
    h.raw_len = raw_len;
    h.n_slices = (uint16_t)n_slices;
    h.flags = flags;

    const uint8_t *body = raw;               /* what the codec layer sees */
    if (flags & PVS_FLAG_DELTA) {
        if (!prev) { fprintf(stderr, "test bug: DELTA without prev\n"); return -1; }
        for (size_t i = 0; i < raw_len; i++)
            xbuf[i] = raw[i] ^ prev[i];
        body = xbuf;
    }

    const uint8_t *payload = body;
    if (flags & PVS_FLAG_LZ4) {
        /* 🔴 raw block (LZ4_compress_HC), **不是** .lz4 帧格式 —— 板端的
         * LZ4_decompress_safe 只认 raw block, 见 protocol.h。 */
        int clen = LZ4_compress_HC((const char *)body, (char *)scratch,
                                   (int)raw_len,
                                   LZ4_compressBound((int)raw_len),
                                   LZ4_HC_LEVEL);
        if (clen <= 0) { fprintf(stderr, "LZ4_compress_HC rc=%d\n", clen); return -1; }
        h.comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (flags & PVS_FLAG_ZLIB) {
        uLongf clen = compressBound(raw_len);
        if (compress2(scratch, &clen, body, raw_len, 6) != Z_OK) return -1;
        h.comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (flags & PVS_FLAG_RLE) {
        h.comp_len = (uint32_t)rle_encode(body, raw_len, scratch);
        payload = scratch;
    } else {
        h.comp_len = raw_len;
    }

    if (send_all(fd, &h, sizeof h) || send_all(fd, payload, h.comp_len))
        return -1;
    want_faces(raw_len, 0);
    sent_crc[n_sent++] = crc32(0L, raw, raw_len);
    printf("test: sent frame %d flags=0x%x n=%u raw=%u comp=%u crc=%08x\n",
           n_sent - 1, flags, n_slices, raw_len, h.comp_len, sent_crc[n_sent - 1]);
    return 0;
}

static int send_frame(int fd, const uint8_t *raw, const uint8_t *prev,
                      uint16_t flags, uint8_t *xbuf, uint8_t *scratch)
{
    return send_frame_n(fd, raw, prev, flags, PVS_N_SLICES, xbuf, scratch);
}

/* DUAL_FACE: [u32 LE comp_len_A][面A 流][面B 流] —— 板端必须把 nA 从 n_slices
 * 推出来 (FOLD_A ? n/3 : n/2), 面边界 = nA*stride。 */
static int send_dual(int fd, const uint8_t *raw, uint16_t flags,
                     uint32_t n_slices, uint8_t *scratch)
{
    pvs_hdr_t h;
    uint32_t stride  = PVS_STRIDE(flags);
    uint32_t raw_len = n_slices * stride;
    uint32_t n_a     = (flags & PVS_FLAG_FOLD_A) ? n_slices / 3u : n_slices / 2u;
    uint32_t la      = n_a * stride, lb = raw_len - la;
    flags |= PVS_FLAG_DUAL_FACE;
    if (!(flags & PVS_FLAG_LZ4)) flags |= PVS_FLAG_ZLIB;

    /* 压缩位跟着 caller 走: 老用例给 0 -> zlib (逐字节不变); 给 PVS_FLAG_LZ4
     * -> 两条 raw block, 供 v3.5 的 PL 路径用 (PL 只认 lz4)。 */
    uLongf ca = compressBound(la), cb = compressBound(lb);
    uint8_t *pa = scratch, *pb = scratch + ca;
    if (flags & PVS_FLAG_LZ4) {
        int a = LZ4_compress_HC((const char *)raw, (char *)pa, (int)la,
                                LZ4_compressBound((int)la), LZ4_HC_LEVEL);
        int b = LZ4_compress_HC((const char *)raw + la, (char *)pb, (int)lb,
                                LZ4_compressBound((int)lb), LZ4_HC_LEVEL);
        if (a <= 0 || b <= 0) { fprintf(stderr, "LZ4_compress_HC dual rc=%d/%d\n", a, b); return -1; }
        ca = (uLongf)a; cb = (uLongf)b;
    } else if (compress2(pa, &ca, raw, la, 6) != Z_OK ||
               compress2(pb, &cb, raw + la, lb, 6) != Z_OK) return -1;

    memcpy(h.magic, PVS_MAGIC, 4);
    h.raw_len = raw_len;
    h.n_slices = (uint16_t)n_slices;
    h.flags = flags;
    h.comp_len = 4u + (uint32_t)ca + (uint32_t)cb;
    uint8_t pfx[4] = { (uint8_t)ca, (uint8_t)(ca >> 8), (uint8_t)(ca >> 16),
                       (uint8_t)(ca >> 24) };
    if (send_all(fd, &h, sizeof h) || send_all(fd, pfx, 4) ||
        send_all(fd, pa, ca) || send_all(fd, pb, cb)) return -1;
    want_faces(la, lb);
    sent_crc[n_sent++] = crc32(0L, raw, raw_len);
    printf("test: sent frame %d DUAL flags=0x%x n=%u nA=%u faces %u+%u crc=%08x\n",
           n_sent - 1, flags, n_slices, n_a, la, lb, sent_crc[n_sent - 1]);
    return 0;
}

/* MSTREAM: 把一整帧切成 nseg 段 (每段片数由 seg[] 给) 分别压成独立流,
 * 前面挂上流表 [u32 n][n×{u32 clen,u32 nsl}] —— 与 protocol.h / povstream.py
 * 的 stream_plan 出来的排布逐字节相同。 */
static int send_mstream(int fd, const uint8_t *raw, const uint8_t *prev,
                        uint16_t flags, const int *seg, int nseg,
                        uint8_t *xbuf, uint8_t *scratch)
{
    pvs_hdr_t h;
    uint32_t stride = PVS_STRIDE(flags);      /* v3.4: 3-bit 时每片 0x9000 */
    uint32_t n_slices = 0;
    for (int i = 0; i < nseg; i++) n_slices += (uint32_t)seg[i];
    uint32_t raw_len = n_slices * stride;
    memcpy(h.magic, PVS_MAGIC, 4);
    h.raw_len = raw_len;
    h.n_slices = (uint16_t)n_slices;
    h.flags = (uint16_t)(flags | PVS_FLAG_MSTREAM);

    const uint8_t *body = raw;
    if (flags & PVS_FLAG_DELTA) {
        if (!prev) { fprintf(stderr, "test bug: DELTA without prev\n"); return -1; }
        for (size_t i = 0; i < raw_len; i++) xbuf[i] = raw[i] ^ prev[i];
        body = xbuf;
    }
    uint32_t tbl_len = 4u + (uint32_t)nseg * 8u;
    uint8_t *p = scratch + tbl_len;         /* 流体从表后开始 */
    uint32_t clen[16];
    uint32_t soff = 0;
    for (int i = 0; i < nseg; i++) {
        uint32_t nb = (uint32_t)seg[i] * stride;
        int c;
        if (flags & PVS_FLAG_ZLIB) {          /* 压缩位说什么就压什么 */
            uLongf zc = compressBound(nb);
            if (compress2(p, &zc, body + soff, nb, 6) != Z_OK) {
                fprintf(stderr, "compress2 seg %d failed\n", i); return -1;
            }
            c = (int)zc;
        } else {
            c = LZ4_compress_HC((const char *)body + soff, (char *)p,
                                (int)nb, LZ4_compressBound((int)nb), LZ4_HC_LEVEL);
            if (c <= 0) { fprintf(stderr, "LZ4_compress_HC seg %d rc=%d\n", i, c); return -1; }
        }
        clen[i] = (uint32_t)c;
        p += c; soff += nb;
    }
    if (soff != raw_len) { fprintf(stderr, "test bug: seg sum != frame\n"); return -1; }
    uint32_t *t = (uint32_t *)scratch;      /* 小端机, 直接写 u32 */
    t[0] = (uint32_t)nseg;
    for (int i = 0; i < nseg; i++) { t[1 + i * 2] = clen[i]; t[2 + i * 2] = (uint32_t)seg[i]; }
    h.comp_len = (uint32_t)(p - scratch);
    if (send_all(fd, &h, sizeof h) || send_all(fd, scratch, h.comp_len)) return -1;
    want_faces(raw_len, 0);
    sent_crc[n_sent++] = crc32(0L, raw, raw_len);
    printf("test: sent frame %d MSTREAM n=%d flags=0x%x slices=%u comp=%u crc=%08x\n",
           n_sent - 1, nseg, h.flags, n_slices, h.comp_len, sent_crc[n_sent - 1]);
    return 0;
}

static int recv_ack(int fd)
{
    uint8_t b;
    ssize_t n;
    do n = recv(fd, &b, 1, 0); while (n < 0 && errno == EINTR);
    if (n != 1) { fprintf(stderr, "FAIL: no ACK byte (n=%zd)\n", n); return -1; }
    if (b != PVS_ACK) { fprintf(stderr, "FAIL: expected ACK got 0x%02x\n", b); return -1; }
    return 0;
}

static int connect_retry(int port)
{
    for (int i = 0; i < 150; i++) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in a = { .sin_family = AF_INET,
                                 .sin_port = htons((uint16_t)port) };
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) {
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            return fd;
        }
        close(fd);
        usleep(20000);
    }
    return -1;
}

/* send a header (+ optional extra bytes, e.g. a bad MSTREAM stream table),
 * expect NAK then close */
static int expect_nak_close2(int fd, const pvs_hdr_t *h, const void *extra,
                             size_t elen, const char *what)
{
    if (send_all(fd, h, sizeof *h)) { fprintf(stderr, "FAIL: send hdr (%s)\n", what); return -1; }
    if (elen && send_all(fd, extra, elen)) { fprintf(stderr, "FAIL: send extra (%s)\n", what); return -1; }
    uint8_t b; ssize_t n;
    do n = recv(fd, &b, 1, 0); while (n < 0 && errno == EINTR);
    if (n != 1 || b != PVS_NAK) {
        fprintf(stderr, "FAIL: %s: expected NAK, n=%zd b=0x%02x\n", what, n, n == 1 ? b : 0);
        return -1;
    }
    do n = recv(fd, &b, 1, 0); while (n < 0 && errno == EINTR);
    if (n != 0) { fprintf(stderr, "FAIL: %s: connection not closed after NAK\n", what); return -1; }
    printf("test: NAK + close on %s OK\n", what);
    return 0;
}

static int expect_nak_close(int fd, const pvs_hdr_t *h, const char *what)
{
    return expect_nak_close2(fd, h, NULL, 0, what);
}


/* ==== v3.5: --pl-lz4 数据通路 =============================================
 * x86 上没有 PL, 但 pov_rxd 的 SIM 构建里有一个按 lz4_axi_top.v 行为写的引擎
 * 模型 (见 pov_rxd.c 的 "SIM: PL lz4 引擎模型")。于是**整条 PL 软件路径**都能
 * 在这里跑真的: bank 认领与轮转、多引擎动态派活、等 done 的纪律、以及三条
 * 回退路径 (非 lz4 / DELTA / 引擎坏)。
 *
 * 判据比老用例强一档: PL 模式下 --crc 算的是**真正落进 DDR bank 的字节**
 * (pov_rxd 那边专门为此分了支), 所以 crc 对得上 = PL 把对的数据写到了对的
 * bank 的对的偏移上 —— 不只是"解码结果对"。
 */
#define PL_PORT_BASE 9530

struct plwant { const char *needle; const char *why; };

static int pl_pass(const char *name, int port, int engines, const char *fault,
                   const struct plwant *want, int nwant, int expect_delta_nak,
                   int serial)
{
    printf("\n=== PL pass: %s (engines=%d fault=%s) ===\n",
           name, engines, fault ? fault : "none");
    n_sent = 0; n_pairs = 0;

    int pfd[2];
    if (pipe(pfd)) { perror("pipe"); return -1; }
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); return -1; }
    if (pid == 0) {
        char ps[16], es[16];
        snprintf(ps, sizeof ps, "%d", port);
        snprintf(es, sizeof es, "%d", engines);
        dup2(pfd[1], 1);
        close(pfd[0]); close(pfd[1]);
        const char *argv[16];
        int a = 0;
        argv[a++] = "pov_rxd_sim";
        argv[a++] = "--port";  argv[a++] = ps;
        argv[a++] = "--fake";  argv[a++] = "20";
        argv[a++] = "--crc";
        argv[a++] = "--pl-lz4";
        argv[a++] = "--pl-engines"; argv[a++] = es;
        if (serial) argv[a++] = "--no-pipeline";
        if (fault) { argv[a++] = "--pl-fault"; argv[a++] = fault;
                     argv[a++] = "--pl-timeout"; argv[a++] = "150"; }
        argv[a] = NULL;
        execv("./pov_rxd_sim", (char *const *)argv);
        perror("execl pov_rxd_sim");
        _exit(127);
    }
    close(pfd[1]);

    int rc = -1, fd = -1;
    uint8_t *raw  = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *raw2 = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *xbuf = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *scratch = malloc(2 * compressBound(PVS_FRAME_RAW_MAX) + PVS_FRAME_RAW_MAX);
    if (!raw || !raw2 || !xbuf || !scratch) { perror("malloc"); goto done; }

    fd = connect_retry(port);
    if (fd < 0) { fprintf(stderr, "FAIL: %s: cannot connect\n", name); goto done; }

    /* 1) 单流 lz4 -> PL, 一个引擎在干活 */
    gen_frame(raw, 40);
    if (send_frame(fd, raw, NULL, PVS_FLAG_LZ4, xbuf, scratch) || recv_ack(fd)) goto done;
    /* 2) MSTREAM 3 条 -> 多引擎动态派活 (2 个引擎吃 3 条流 = 有一条要等) */
    {
        static const int s3[] = { 120, 120, 120 };
        gen_frame(raw, 41);
        if (send_mstream(fd, raw, NULL, PVS_FLAG_LZ4, s3, 3, xbuf, scratch)
            || recv_ack(fd)) goto done;
    }
    /* 3) DUAL_FACE 两条 lz4 流 -> 两个不同的 bank 偏移 (面拆分由 bankcrc 核对) */
    gen_frame_n(raw, 42, PVS_FRAME_RAW_MAX);
    if (send_dual(fd, raw, PVS_FLAG_LZ4, PVS_N_SLICES_MAX, scratch) || recv_ack(fd)) goto done;
    usleep(300000);                       /* 等 flip 线程把它翻上去 */
    /* 4) zlib 帧 -> PL 接不了, 必须回退 CPU 且照样正确 (bank 由 RX 自己写) */
    gen_frame(raw, 43);
    if (send_frame(fd, raw, NULL, PVS_FLAG_ZLIB, xbuf, scratch) || recv_ack(fd)) goto done;
    /* 5) 连翻几帧: bank 轮转 A->B->C->A 必须一直对 (crc 全部要能对上) */
    for (uint32_t seed = 44; seed < 48; seed++) {
        gen_frame(raw, seed);
        if (send_frame(fd, raw, NULL, PVS_FLAG_LZ4, xbuf, scratch) || recv_ack(fd)) goto done;
        usleep(120000);
    }

    if (expect_delta_nak) {
        /* 6) DELTA|LZ4: PL 帧不留参考帧 -> 这一帧必须 NAK+关连接, 并且**进程级**
         *    把 PL 关掉。重连后的 keyframe 走 CPU, 之后 DELTA 链就正常了
         *    —— 关键是 NAK 只该发生**一次**, 不能变成重连风暴。 */
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = PVS_FLAG_LZ4 | PVS_FLAG_DELTA;
        if (expect_nak_close(fd, &bad, "PL 模式下的 DELTA 帧")) goto done;
        close(fd);
        fd = connect_retry(port);
        if (fd < 0) { fprintf(stderr, "FAIL: %s: reconnect\n", name); goto done; }
        gen_frame(raw, 50);
        if (send_frame(fd, raw, NULL, PVS_FLAG_LZ4, xbuf, scratch) || recv_ack(fd)) goto done;
        memcpy(raw2, raw, PVS_FRAME_RAW);
        gen_frame(raw, 51);
        if (send_frame(fd, raw, raw2, PVS_FLAG_LZ4 | PVS_FLAG_DELTA, xbuf, scratch)
            || recv_ack(fd)) goto done;
        memcpy(raw2, raw, PVS_FRAME_RAW);   /* 参考帧 <- 刚 ACK 的那一帧 (seed 51) */
        gen_frame(raw, 52);                 /* 再来一个 DELTA: 不能再 NAK 一次 */
        if (send_frame(fd, raw, raw2, PVS_FLAG_LZ4 | PVS_FLAG_DELTA, xbuf, scratch)
            || recv_ack(fd)) goto done;
    }
    usleep(200000);
    close(fd); fd = -1;

    kill(pid, SIGINT);
    {
        FILE *lf = fdopen(pfd[0], "r");
        char line[512];
        uint32_t got[MAX_FRAMES];
        int n_got = 0, n_flips = 0, bad_faces = 0, guard = 0;
        int *seen = calloc((size_t)(nwant > 0 ? nwant : 1), sizeof(int));
        while (fgets(line, sizeof line, lf)) {
            if (!strstr(line, "SIM: reg[")) fputs(line, stdout);
            /* 🔴 流水线最危险的不变量: RX 绝不能往正在显示的 bank 里写。
             * daemon 在 SIM 构建里守着它并打 BANKGUARD FAIL, 这里只要见到
             * 一次就判失败 —— 这类撕裂在真板上是查不出来的。 */
            if (strstr(line, "BANKGUARD FAIL")) guard = 1;
            char *p = strstr(line, "crc=");
            if (p && strstr(line, "FRAME ") && n_got < MAX_FRAMES)
                got[n_got++] = (uint32_t)strtoul(p + 4, NULL, 16);
            if (strstr(line, "FLIP ")) n_flips++;
            for (int i = 0; i < nwant; i++)
                if (strstr(line, want[i].needle)) seen[i] = 1;
            if ((p = strstr(line, "bankcrc")) != NULL) {
                unsigned la = 0, lb = 0;
                char *q1 = strstr(p, "len="), *q2 = q1 ? strstr(q1 + 4, "len=") : NULL;
                if (q1 && q2 && sscanf(q1 + 4, "%u", &la) == 1 &&
                    sscanf(q2 + 4, "%u", &lb) == 1) {
                    int ok = 0;
                    for (int i = 0; i < n_pairs; i++)
                        if (exp_pair[i][0] == la && exp_pair[i][1] == lb) ok = 1;
                    if (!ok) {
                        fprintf(stderr, "FAIL: %s: bank 面拆分 A=%u B=%u 不在期望集合里\n",
                                name, la, lb);
                        bad_faces = 1;
                    }
                }
            }
        }
        fclose(lf);
        waitpid(pid, NULL, 0);
        pid = 0;
        if (guard) {
            fprintf(stderr, "FAIL: %s: RX 认领了正在显示的 bank (见 BANKGUARD FAIL)\n", name);
            free(seen); goto done;
        }
        if (bad_faces) { free(seen); goto done; }
        if (n_got != n_sent) {
            fprintf(stderr, "FAIL: %s: sent %d frames, daemon logged %d\n",
                    name, n_sent, n_got);
            free(seen); goto done;
        }
        for (int i = 0; i < n_sent; i++)
            if (got[i] != sent_crc[i]) {
                fprintf(stderr, "FAIL: %s: frame %d crc sent=%08x got=%08x "
                        "(PL 写进 bank 的字节不对?)\n", name, i, sent_crc[i], got[i]);
                free(seen); goto done;
            }
        if (n_flips < 1) {
            fprintf(stderr, "FAIL: %s: no flip happened\n", name);
            free(seen); goto done;
        }
        for (int i = 0; i < nwant; i++)
            if (!seen[i]) {
                fprintf(stderr, "FAIL: %s: 日志里没有 \"%s\" —— %s\n",
                        name, want[i].needle, want[i].why);
                free(seen); goto done;
            }
        free(seen);
        printf("test: PL pass %s OK (%d frames, %d flips, crc 全对)\n",
               name, n_got, n_flips);
    }
    rc = 0;
done:
    if (fd >= 0) close(fd);
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    free(raw); free(raw2); free(xbuf); free(scratch);
    return rc;
}

int main(void)
{
    signal(SIGPIPE, SIG_IGN);

    /* spawn the sim daemon with stdout -> pipe. --crc so every FRAME line
     * carries the reconstructed-raw crc; dual window to exercise §3.2. */
    int pfd[2];
    if (pipe(pfd)) { perror("pipe"); return 1; }
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); return 1; }
    if (pid == 0) {
        char portstr[16];
        snprintf(portstr, sizeof portstr, "%d", TEST_PORT);
        dup2(pfd[1], 1);
        close(pfd[0]); close(pfd[1]);
        execl("./pov_rxd_sim", "pov_rxd_sim", "--port", portstr,
              "--fake", "20", "--crc", "--flip-window", "dual", (char *)NULL);
        perror("execl pov_rxd_sim");
        _exit(127);
    }
    close(pfd[1]);

    int fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: cannot connect to sim daemon\n"); kill(pid, SIGKILL); return 1; }

    /* v3.1/v3.4: 帧长可变 (720 片 1-bit 与 240 片 3-bit 都是 8847360 B 上限),
     * 一律按最大帧分配; scratch 还要装下 send_dual 的两条流。 */
    uint8_t *raw  = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *raw2 = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *xbuf = malloc(PVS_FRAME_RAW_MAX);
    uint8_t *scratch = malloc(2 * compressBound(PVS_FRAME_RAW_MAX) + PVS_FRAME_RAW_MAX);
    if (!raw || !raw2 || !xbuf || !scratch) { perror("malloc"); return 1; }

    int rc = 1;

    /* 1-3: one frame per codec, ACK-paced (normal sender behaviour) */
    gen_frame(raw, 1);
    if (send_frame(fd, raw, NULL, 0, xbuf, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 2);
    if (send_frame(fd, raw, NULL, PVS_FLAG_RLE, xbuf, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 3);
    if (send_frame(fd, raw, NULL, PVS_FLAG_ZLIB, xbuf, scratch) || recv_ack(fd)) goto out;

    /* 4-5: LZ4 raw block keyframe, 再跟一个 DELTA|LZ4 —— DELTA 位与压缩位
     * 正交 (板端先按压缩位解码再 XOR), 所以这个组合必须与 DELTA|ZLIB 一样成立。*/
    gen_frame(raw, 8);
    if (send_frame(fd, raw, NULL, PVS_FLAG_LZ4, xbuf, scratch) || recv_ack(fd)) goto out;
    memcpy(raw2, raw, PVS_FRAME_RAW);              /* prev = seed 8 */
    gen_frame(raw, 9);
    if (send_frame(fd, raw, raw2, PVS_FLAG_LZ4 | PVS_FLAG_DELTA, xbuf, scratch)
        || recv_ack(fd)) goto out;

    /* 5b-5e: MSTREAM 流表 —— 1/2/3 条流 + 一个 DELTA|MSTREAM。
     * 单面 360 片, 所以 3 条流用 120/120/120, 2 条流用 180/180 (板端 dec_plan
     * 会把 3 条连续分组成 240/120 —— 分组正确性由 daemon 日志的 decode plan
     * 行体现, 这里保证的是「解出来逐字节没错」)。 */
    {
        static const int s1[] = { 360 }, s2[] = { 180, 180 }, s3[] = { 120, 120, 120 };
        gen_frame(raw, 20);
        if (send_mstream(fd, raw, NULL, PVS_FLAG_LZ4, s1, 1, xbuf, scratch)
            || recv_ack(fd)) goto out;
        gen_frame(raw, 21);
        if (send_mstream(fd, raw, NULL, PVS_FLAG_LZ4, s2, 2, xbuf, scratch)
            || recv_ack(fd)) goto out;
        gen_frame(raw, 22);
        if (send_mstream(fd, raw, NULL, PVS_FLAG_ZLIB, s3, 3, xbuf, scratch)
            || recv_ack(fd)) goto out;
        memcpy(raw2, raw, PVS_FRAME_RAW);          /* prev = seed 22 */
        gen_frame(raw, 23);
        if (send_mstream(fd, raw, raw2, PVS_FLAG_LZ4 | PVS_FLAG_DELTA, s3, 3,
                         xbuf, scratch) || recv_ack(fd)) goto out;
    }

    /* 6-8: DELTA|ZLIB chain vs the last ACKed frame (seed 23 是参考帧).
     * Daemon must rebuild raw = prev ^ decoded; crc check below proves it. */
    memcpy(raw2, raw, PVS_FRAME_RAW);              /* raw2 = prev (seed 23) */
    for (uint32_t seed = 10; seed < 13; seed++) {
        gen_frame(raw, seed);
        if (send_frame(fd, raw, raw2, PVS_FLAG_ZLIB | PVS_FLAG_DELTA,
                       xbuf, scratch) || recv_ack(fd)) goto out;
        memcpy(raw2, raw, PVS_FRAME_RAW);          /* prev <- this frame */
    }

    /* 7-8: precompress both, then send truly back-to-back with no ACK
     * wait -> newest frame overwrites the ready buffer (drop path); both
     * must ACK since RX is decoupled from the flip */
    {
        uint8_t *scratch2 = malloc(compressBound(PVS_FRAME_RAW));
        if (!scratch2) { perror("malloc"); goto out; }
        gen_frame(raw, 4);
        gen_frame(raw2, 5);
        uLongf c1 = compressBound(PVS_FRAME_RAW), c2 = c1;
        if (compress2(scratch,  &c1, raw,  PVS_FRAME_RAW, 6) != Z_OK ||
            compress2(scratch2, &c2, raw2, PVS_FRAME_RAW, 6) != Z_OK) goto out;
        pvs_hdr_t h1, h2;
        memcpy(h1.magic, PVS_MAGIC, 4);
        h1.raw_len = PVS_FRAME_RAW; h1.n_slices = PVS_N_SLICES;
        h1.flags = PVS_FLAG_ZLIB; h2 = h1;
        h1.comp_len = (uint32_t)c1; h2.comp_len = (uint32_t)c2;
        if (send_all(fd, &h1, sizeof h1) || send_all(fd, scratch,  c1) ||
            send_all(fd, &h2, sizeof h2) || send_all(fd, scratch2, c2)) goto out;
        sent_crc[n_sent++] = crc32(0L, raw,  PVS_FRAME_RAW);
        sent_crc[n_sent++] = crc32(0L, raw2, PVS_FRAME_RAW);
        printf("test: sent frames %d+%d back-to-back (comp=%lu,%lu)\n",
               n_sent - 2, n_sent - 1, c1, c2);
        free(scratch2);
        if (recv_ack(fd) || recv_ack(fd)) goto out;
    }

    /* ---- v3.4 3-bit + 非 360 槽几何 --------------------------------------
     * 这一组守的是两件事:
     *  (1) 片距从 flag 推 (3-bit = 0x9000), 不是常量;
     *  (2) 双面帧的 nA 从 n_slices 推 (FOLD_A ? n/3 : n/2), **不是写死 360/180**
     *      —— 老代码那两行会把 60 槽的 3-bit 双面帧直接 NAK 掉。
     * 整帧 crc 对不出 nA 拆错 (面A/面B 连在一起), 所以还额外核对 SIM 的
     * bankcrc 行给出的两个面长, 见收尾处的 exp_pair 检查。 */
    {
        const uint32_t n3 = 60;                       /* 05_3bit_bcm.md 的每面 60 槽 */
        uint32_t raw3 = n3 * PVS_SLICE_STRIDE_3BIT;   /* 2211840 */
        /* 3-bit 单面 60 片: zlib 关键帧 + LZ4|DELTA 跟一帧 */
        gen_frame_n(raw, 30, raw3);
        if (send_frame_n(fd, raw, NULL, PVS_FLAG_3BIT | PVS_FLAG_ZLIB, n3,
                         xbuf, scratch) || recv_ack(fd)) goto out;
        memcpy(raw2, raw, raw3);
        gen_frame_n(raw, 31, raw3);
        if (send_frame_n(fd, raw, raw2, PVS_FLAG_3BIT | PVS_FLAG_LZ4 | PVS_FLAG_DELTA,
                         n3, xbuf, scratch) || recv_ack(fd)) goto out;
        /* 3-bit + MSTREAM: 流表里的 n_slices_i 仍是**片数**, 解压长度按 0x9000 算 */
        {
            static const int s2[] = { 30, 30 };
            gen_frame_n(raw, 32, raw3);
            if (send_mstream(fd, raw, NULL, PVS_FLAG_3BIT | PVS_FLAG_LZ4, s2, 2,
                             xbuf, scratch) || recv_ack(fd)) goto out;
        }
        /* 3-bit 双面 120 片 -> nA = 60 (老代码在这里 NAK: 120 != 360+360) */
        gen_frame_n(raw, 33, 2 * raw3);
        if (send_dual(fd, raw, PVS_FLAG_3BIT, 2 * n3, scratch) || recv_ack(fd)) goto out;
        usleep(300000);                       /* 让 flip 线程把它翻上去 (查面长) */
        /* 1-bit 双面 720 -> nA = 360, 与写死时同值 (回归) */
        gen_frame_n(raw, 34, PVS_FRAME_RAW_MAX);
        if (send_dual(fd, raw, 0, PVS_N_SLICES_MAX, scratch) || recv_ack(fd)) goto out;
        usleep(300000);
        /* 1-bit 双面 + FOLD_A 540 -> nA = 180, 与写死时同值 (回归) */
        gen_frame_n(raw, 35, 540u * PVS_SLICE_STRIDE);
        if (send_dual(fd, raw, PVS_FLAG_FOLD_A, 540, scratch) || recv_ack(fd)) goto out;
        usleep(300000);
    }

    /* 9: one more paced frame to confirm daemon is still healthy */
    gen_frame(raw, 6);
    if (send_frame(fd, raw, NULL, PVS_FLAG_ZLIB, xbuf, scratch) || recv_ack(fd)) goto out;
    close(fd);

    /* NAK 1: reconnect, garbage magic -> expect 0x15 then close.
     * Also proves the DELTA reference reset: the previous connection ACKed
     * frames, and the *next* NAK case (first-frame DELTA) must still NAK. */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, "XXXX", 4);
        bad.comp_len = 16; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = 0;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "bad magic")) goto out;
        close(fd);
    }

    /* NAK 2: unknown flag bit (**bit8**) -> mask check must reject.
     * 注意别用已分配的位: bit3/bit4 = DUAL_FACE/FOLD_A (v3.1), bit7 = 3BIT
     * (v3.4)。用它们这个用例照样 NAK (被几何/长度校验拦下), 但测的就不是
     * 「未知 flag 位」了 —— 这条 2026-08-20 从 bit7 挪到 bit8。 */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = (1u << 8);
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "unknown flag bit")) goto out;
        close(fd);
    }

    /* NAK 2e: 3-bit 帧但 raw_len 按 1-bit 片距算 -> 长度自洽性必须拦下。
     * 这条守的是「片距从 flag 推」这件事本身: 用常量 0x3000 的老代码会放行。 */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024;
        bad.raw_len = 60u * PVS_SLICE_STRIDE;         /* 应该是 60*0x9000 */
        bad.n_slices = 60; bad.flags = PVS_FLAG_3BIT | PVS_FLAG_ZLIB;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "3BIT raw_len 按 1-bit 片距算")) goto out;
        close(fd);
    }

    /* NAK 2f: 3-bit 片数越界 (240 片 * 0x9000 已经吃满一个 bank) */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024;
        bad.n_slices = PVS_N_SLICES_MAX_3BIT + 1;
        bad.flags = PVS_FLAG_3BIT | PVS_FLAG_ZLIB;
        bad.raw_len = (uint32_t)bad.n_slices * PVS_SLICE_STRIDE_3BIT;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "3BIT n_slices 越界 (>240)")) goto out;
        close(fd);
    }

    /* NAK 2g: 双面帧片数不是 2 的倍数 -> nA 推不出来, 必须 NAK 而不是拆歪 */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.n_slices = 361;
        bad.flags = PVS_FLAG_DUAL_FACE | PVS_FLAG_ZLIB;
        bad.raw_len = 361u * PVS_SLICE_STRIDE;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "DUAL_FACE 片数为奇数")) goto out;
        close(fd);
    }

    /* NAK 2b: 压缩位同时置 ZLIB|LZ4 -> 互斥 (protocol.h: 一帧只能有一个
     * 压缩位)。板端按「codec_bits 是不是 2 的幂」判, 这条守着它。 */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = PVS_FLAG_ZLIB | PVS_FLAG_LZ4;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "ZLIB|LZ4 both set")) goto out;
        close(fd);
    }

    /* NAK 2c/2d: MSTREAM 流表的两个求和自校验。这两条是流表格式的命根子 ——
     * 少了它们, 一条被截断/错位的载荷会解出半帧垃圾还照样 ACK。 */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.raw_len = PVS_FRAME_RAW; bad.n_slices = PVS_N_SLICES;
        bad.flags = (uint16_t)(PVS_FLAG_LZ4 | PVS_FLAG_MSTREAM);
        /* [u32 n=2][clen=100,nsl=180][clen=100,nsl=100] -> Σn_slices=280 != 360 */
        uint32_t tbl[5] = { 2, 100, 180, 100, 100 };
        bad.comp_len = 4 + 16 + 200;                  /* Σclen 对得上, 片数对不上 */
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close2(fd, &bad, tbl, sizeof tbl,
                              "MSTREAM Σn_slices mismatch")) goto out;
        close(fd);

        tbl[2] = 180; tbl[4] = 180;                   /* 片数改对 (360) */
        bad.comp_len = 4 + 16 + 199;                  /* Σcomp_len 差 1 字节 */
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close2(fd, &bad, tbl, sizeof tbl,
                              "MSTREAM Σcomp_len mismatch")) goto out;
        close(fd);

        tbl[0] = PVS_MAX_STREAMS + 1;                 /* 流数越界 */
        bad.comp_len = 4 + 16 + 200;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close2(fd, &bad, tbl, 4, "MSTREAM n_streams out of range")) goto out;
        close(fd);
    }

    /* NAK 3: DELTA as first frame of a fresh connection (no reference,
     * and reference from earlier connections must NOT survive) */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = PVS_FLAG_ZLIB | PVS_FLAG_DELTA;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "first-frame DELTA")) goto out;
        close(fd);
    }

    /* NAK 4: keyframe then reconnect then DELTA -> reference reset check */
    {
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        gen_frame(raw, 7);
        if (send_frame(fd, raw, NULL, PVS_FLAG_ZLIB, xbuf, scratch) || recv_ack(fd)) goto out;
        close(fd);                                  /* reference now stale */
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = PVS_FLAG_ZLIB | PVS_FLAG_DELTA;
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "DELTA after reconnect")) goto out;
        close(fd);
        fd = -1;
    }

    /* stop daemon, harvest its log, verify per-frame CRCs in order */
    usleep(100000);   /* let the flip thread drain the last ready frame */
    kill(pid, SIGINT);
    {
        FILE *lf = fdopen(pfd[0], "r");
        char line[512];
        uint32_t got_crc[MAX_FRAMES];
        int n_got = 0, n_flips = 0, n_dualflip = 0, bad_faces = 0;
        while (fgets(line, sizeof line, lf)) {
            fputs(line, stdout);
            char *p = strstr(line, "crc=");
            if (p && strstr(line, "FRAME "))
                got_crc[n_got++] = (uint32_t)strtoul(p + 4, NULL, 16);
            if (strstr(line, "FLIP "))
                n_flips++;
            /* SIM: bankcrc A@0x… len=LA crc=… B@0x… len=LB crc=… —— 真正落进
             * DDR bank 的面拆分。整帧 crc 看不出 nA 拆错, 这两个长度看得出。 */
            if ((p = strstr(line, "bankcrc")) != NULL) {
                unsigned la = 0, lb = 0;
                char *q1 = strstr(p, "len="), *q2 = q1 ? strstr(q1 + 4, "len=") : NULL;
                if (q1 && q2 && sscanf(q1 + 4, "%u", &la) == 1 &&
                    sscanf(q2 + 4, "%u", &lb) == 1) {
                    int ok = 0;
                    for (int i = 0; i < n_pairs; i++)
                        if (exp_pair[i][0] == la && exp_pair[i][1] == lb) ok = 1;
                    if (!ok) {
                        fprintf(stderr, "FAIL: 面拆分不对: bank 里 A=%u B=%u "
                                "不在期望集合里 (nA 是不是又写死了?)\n", la, lb);
                        bad_faces = 1;
                    }
                    if (lb) n_dualflip++;
                }
            }
        }
        fclose(lf);
        int status;
        waitpid(pid, &status, 0);
        pid = 0;

        if (n_got != n_sent) {
            fprintf(stderr, "FAIL: sent %d frames, daemon logged %d\n", n_sent, n_got);
            goto out;
        }
        for (int i = 0; i < n_sent; i++) {
            if (got_crc[i] != sent_crc[i]) {
                fprintf(stderr, "FAIL: frame %d crc mismatch sent=%08x got=%08x\n",
                        i, sent_crc[i], got_crc[i]);
                goto out;
            }
        }
        if (n_flips < 1) { fprintf(stderr, "FAIL: no frame ever flipped\n"); goto out; }
        if (bad_faces) goto out;
        if (n_dualflip < 1) {
            fprintf(stderr, "FAIL: 没有任何双面帧翻上去过, 面拆分没被验到\n");
            goto out;
        }
        printf("test: 面拆分核对通过 (%d 次双面翻页, %d 组期望面长)\n",
               n_dualflip, n_pairs);
        printf("test: %d frames, all CRCs match (delta rebuilt OK), %d flips\n",
               n_got, n_flips);
    }

    /* ---- v3.5: --pl-lz4 数据通路 (三趟) --------------------------------- */
    {
        static const struct plwant w_ok[] = {
            { "PL 自检: 引擎0", "开机自检必须真的点名每个引擎跑一遍" },
            { "PL 自检: 引擎1", "多引擎时每个都要单测, 否则'两个引擎其实是同一个'查不出来" },
            { "PL lz4 自检通过", "自检结论要落到日志里" },
            { "PL 回退 CPU: PL 只认 lz4", "zlib 帧必须**响亮地**说明为什么没走 PL" },
            { "PLDIAG", "PL 的墙钟/B-per-clk 要能在运行中看到" },
            { "退回 CPU 解码", "DELTA 帧必须当场说清 PL 为什么被关掉" },
        };
        static const struct plwant w_err[] = {
            { "PL 解码失败", "STATUS[1]=error 必须原样打出来" },
            { "**PL 引擎有问题**", "CPU 解得出来而 PL 解不出 = 引擎的锅, 必须点名" },
        };
        /* 🔴 引擎"卡死"是 BD 交付时确认的**安全问题**: 一条流长度不对时引擎
         * busy 恒 1、不 done 不 error, 而 RTL 没有软复位。所以这两趟守的是:
         *  (a) 3 个挂 1 个 -> 摘掉它, 剩下 2 个**继续跑**, 不能一路退化;
         *  (b) 全挂了 -> 永久关 PL, 而且日志要说是"卡死"不是"解码出错"。 */
        static const struct plwant w_hang1[] = {
            { "卡死", "必须点名是引擎卡住, 不是解码出错" },
            { "摘出派发池", "卡死的引擎不摘掉, 下一帧派给它又超时, 会一路退化" },
            { "PL 降级运行: 还有 2/3", "3 挂 1 必须还剩 2 个在跑, 不是整个关掉" },
        };
        static const struct plwant w_hangall[] = {
            { "全部卡死", "全挂时要明确区分'硬件卡住'和'解码出错'" },
            { "永久关闭", "全挂了才该关 PL" },
        };
        static const struct plwant w_st[] = {
            { "PL lz4 自检没过", "自检失败必须是一条醒目结论" },
            { "全部走 CPU 解码", "自检没过就该自动关掉 --pl-lz4, 而不是带病上路" },
        };
        static const struct plwant w_ser[] = {
            { "PL lz4 串行 (--no-pipeline)", "退回开关必须在启动日志里说清楚" },
            { "pipe=off", "PLDIAG 要能一眼看出跑的是哪种模式" },
        };
        if (pl_pass("正常 2 引擎 (流水线)", PL_PORT_BASE, 2, NULL,
                    w_ok, (int)(sizeof w_ok / sizeof w_ok[0]), 1, 0)) goto out;
        /* 同一组帧走串行路径: 退回开关必须逐帧结果一致 (crc 全对) */
        if (pl_pass("正常 2 引擎 (--no-pipeline)", PL_PORT_BASE + 4, 2, NULL,
                    w_ser, (int)(sizeof w_ser / sizeof w_ser[0]), 1, 1)) goto out;
        /* 1 个引擎 => 自检只 start 一次 => fault at 2 正好打在第一帧上 */
        if (pl_pass("引擎报 error", PL_PORT_BASE + 1, 1, "error:2",
                    w_err, (int)(sizeof w_err / sizeof w_err[0]), 0, 0)) goto out;
        /* 3 引擎: 自检占掉 start 1..3, 第 4 次 start 落在引擎0 的第一帧上 ->
         * 只有引擎0 卡死, 1/2 照常 -> 必须降级到 2 个引擎继续跑 */
        if (pl_pass("3 引擎挂 1 -> 降级继续", PL_PORT_BASE + 5, 3, "hang:4",
                    w_hang1, (int)(sizeof w_hang1 / sizeof w_hang1[0]), 0, 0)) goto out;
        /* 1 引擎: 唯一的引擎卡死 = 全挂 -> 永久关 PL */
        if (pl_pass("引擎全卡死", PL_PORT_BASE + 2, 1, "hang:2",
                    w_hangall, (int)(sizeof w_hangall / sizeof w_hangall[0]), 0, 0)) goto out;
        /* fault at 1 => 自检第一次 start 就挂 => 开机就该把 PL 关掉 */
        if (pl_pass("自检就挂", PL_PORT_BASE + 3, 1, "hang:1",
                    w_st, (int)(sizeof w_st / sizeof w_st[0]), 0, 0)) goto out;
    }

    printf("PASS\n");
    rc = 0;
out:
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    return rc;
}
