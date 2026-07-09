/*
 * test_local.c - x86 loopback test for pov_rxd (SIM_NO_DEVMEM build).
 *
 * Spawns ./pov_rxd_sim (--crc --verify) on a local port and drives it as a
 * sender. Sections:
 *
 *  A. PVS1 regression (old-style sender): raw / zero-run RLE / zlib frames,
 *     two zlib frames back-to-back with no ACK wait (drop/no-flip path),
 *     garbage header -> NAK + close.
 *
 *  B. v2 delta suite: 100 synthetic evolving frames with GOP-10 keyframes;
 *     - delta before any keyframe -> NAK, connection stays open;
 *     - reconnect mid-stream (frame 50): delta -> NAK (anchor lost),
 *       keyframe resend -> ACK, stream continues;
 *     - two deltas back-to-back with no ACK wait (drop path with dirty
 *       union across a non-flip);
 *     - one DELTA|ZLIB and one bare DELTA (raw XOR mask) frame.
 *
 * The daemon runs with --verify: after EVERY frame it memcmps the freshly
 * synced inactive bank against the full shadow - this is the bank-union
 * invariant (inactive bank == current raw frame even though only dirty
 * spans of the last two deltas were written). Any "VERIFY FAIL" line fails
 * the test. CRCs of all ACKed frames are checked in order against what was
 * sent (proves decode paths), and VERIFY OK count must equal frame count.
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

#include "../protocol.h"
#include "pvs_codec.h"

#define TEST_PORT 9517
#define MAX_FRAMES 160

static uint32_t sent_crc[MAX_FRAMES];
static int n_sent = 0;

/* deterministic frame: mostly zeros (like real slice data) + seeded noise */
static void gen_frame(uint8_t *f, uint32_t seed)
{
    memset(f, 0, PVS_FRAME_RAW);
    uint32_t s = seed * 2654435761u + 1;
    for (int i = 0; i < 40000; i++) {
        s = s * 1664525u + 1013904223u;
        uint32_t pos = (s >> 8) % PVS_FRAME_RAW;
        uint8_t val = (uint8_t)(s & 0xff);
        f[pos] = val ? val : 0x5a;   /* keep literals nonzero for RLE */
    }
}

/* evolve a frame in place: ~3000 byte mutations (some clear back to zero),
 * like adjacent frames of a real animation */
static void mutate_frame(uint8_t *f, uint32_t seed)
{
    uint32_t s = seed * 2246822519u + 3;
    for (int i = 0; i < 3000; i++) {
        s = s * 1664525u + 1013904223u;
        uint32_t pos = (s >> 6) % PVS_FRAME_RAW;
        f[pos] = (uint8_t)(s & 0xff);          /* 1/256 clears to zero */
    }
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

static int send_hdr_payload(int fd, uint16_t flags, const uint8_t *payload,
                            uint32_t comp_len)
{
    pvs_hdr_t h;
    memcpy(h.magic, PVS_MAGIC, 4);
    h.comp_len = comp_len;
    h.raw_len = PVS_FRAME_RAW;
    h.n_slices = PVS_N_SLICES;
    h.flags = flags;
    if (send_all(fd, &h, sizeof h) || send_all(fd, payload, comp_len))
        return -1;
    return 0;
}

/* full frame in the given codec; records CRC as an expected-ACK frame */
static int send_frame(int fd, const uint8_t *raw, uint16_t flags,
                      uint8_t *scratch)
{
    uint32_t comp_len;
    const uint8_t *payload = raw;
    if (flags & PVS_FLAG_ZLIB) {
        uLongf clen = compressBound(PVS_FRAME_RAW);
        if (compress2(scratch, &clen, raw, PVS_FRAME_RAW, 6) != Z_OK) return -1;
        comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (flags & PVS_FLAG_RLE) {
        comp_len = (uint32_t)pvs_rle_encode(raw, PVS_FRAME_RAW, scratch);
        payload = scratch;
    } else {
        comp_len = PVS_FRAME_RAW;
    }
    if (send_hdr_payload(fd, flags, payload, comp_len)) return -1;
    sent_crc[n_sent++] = crc32(0L, raw, PVS_FRAME_RAW);
    printf("test: sent frame %d flags=0x%x comp=%u crc=%08x\n",
           n_sent - 1, flags, comp_len, sent_crc[n_sent - 1]);
    return 0;
}

/* delta frame: mask = raw ^ prev, encoded per `flags` codec bits.
 * `expect_ack` controls whether the CRC is recorded. */
static int send_delta(int fd, const uint8_t *raw, const uint8_t *prev,
                      uint16_t codec, uint8_t *mask, uint8_t *scratch,
                      int expect_ack)
{
    for (size_t i = 0; i < PVS_FRAME_RAW; i++) mask[i] = raw[i] ^ prev[i];
    uint16_t flags = PVS_FLAG_DELTA | codec;
    uint32_t comp_len;
    const uint8_t *payload = mask;
    if (codec == (PVS_FLAG_RLE | PVS_FLAG_ZLIB)) {
        /* normal v2 wire form: zlib-wrapped RLE stream */
        size_t rlen = pvs_rle_encode(mask, PVS_FRAME_RAW, scratch);
        uint8_t *tmp = malloc(compressBound(rlen));
        uLongf clen = compressBound(rlen);
        if (!tmp || compress2(tmp, &clen, scratch, rlen, 6) != Z_OK) return -1;
        memcpy(scratch, tmp, clen);
        free(tmp);
        comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (codec == PVS_FLAG_RLE) {
        comp_len = (uint32_t)pvs_rle_encode(mask, PVS_FRAME_RAW, scratch);
        payload = scratch;
    } else if (codec == PVS_FLAG_ZLIB) {
        uLongf clen = compressBound(PVS_FRAME_RAW);
        if (compress2(scratch, &clen, mask, PVS_FRAME_RAW, 6) != Z_OK) return -1;
        comp_len = (uint32_t)clen;
        payload = scratch;
    } else {
        comp_len = PVS_FRAME_RAW;
    }
    if (send_hdr_payload(fd, flags, payload, comp_len)) return -1;
    if (expect_ack) {
        sent_crc[n_sent++] = crc32(0L, raw, PVS_FRAME_RAW);
        printf("test: sent delta %d flags=0x%x comp=%u crc=%08x\n",
               n_sent - 1, flags, comp_len, sent_crc[n_sent - 1]);
    } else {
        printf("test: sent delta (expect NAK) flags=0x%x comp=%u\n",
               flags, comp_len);
    }
    return 0;
}

static int recv_byte(int fd, uint8_t *b)
{
    ssize_t n;
    do n = recv(fd, b, 1, 0); while (n < 0 && errno == EINTR);
    return (int)n;
}

static int recv_ack(int fd)
{
    uint8_t b;
    if (recv_byte(fd, &b) != 1) { fprintf(stderr, "FAIL: no ACK byte\n"); return -1; }
    if (b != PVS_ACK) { fprintf(stderr, "FAIL: expected ACK got 0x%02x\n", b); return -1; }
    return 0;
}

static int recv_nak_conn_open(int fd)
{
    uint8_t b;
    if (recv_byte(fd, &b) != 1) { fprintf(stderr, "FAIL: no NAK byte\n"); return -1; }
    if (b != PVS_NAK) { fprintf(stderr, "FAIL: expected NAK got 0x%02x\n", b); return -1; }
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

int main(void)
{
    signal(SIGPIPE, SIG_IGN);

    /* spawn the sim daemon with stdout -> pipe */
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
              "--fake", "20", "--crc", "--verify", (char *)NULL);
        perror("execl pov_rxd_sim");
        _exit(127);
    }
    close(pfd[1]);

    int fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: cannot connect to sim daemon\n"); kill(pid, SIGKILL); return 1; }

    uint8_t *raw     = malloc(PVS_FRAME_RAW);
    uint8_t *prev    = malloc(PVS_FRAME_RAW);   /* sender-side shadow */
    uint8_t *mask    = malloc(PVS_FRAME_RAW);
    uint8_t *scratch = malloc(compressBound(PVS_FRAME_RAW) + PVS_FRAME_RAW);
    if (!raw || !prev || !mask || !scratch) { perror("malloc"); return 1; }

    int rc = 1;

    /* ================= section A: PVS1 regression ======================= */

    /* A1-A3: one frame per codec, ACK-paced (old-style sender) */
    gen_frame(raw, 1);
    if (send_frame(fd, raw, 0, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 2);
    if (send_frame(fd, raw, PVS_FLAG_RLE, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 3);
    if (send_frame(fd, raw, PVS_FLAG_ZLIB, scratch) || recv_ack(fd)) goto out;

    /* A4-A5: precompress both, then send truly back-to-back with no ACK
     * wait -> exercises the sender-faster drop/no-flip path */
    {
        uint8_t *raw2 = malloc(PVS_FRAME_RAW);
        uint8_t *scratch2 = malloc(compressBound(PVS_FRAME_RAW));
        if (!raw2 || !scratch2) { perror("malloc"); goto out; }
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
        free(raw2); free(scratch2);
        if (recv_ack(fd) || recv_ack(fd)) goto out;
    }

    /* A6: one more paced frame to confirm daemon is still healthy */
    gen_frame(raw, 6);
    if (send_frame(fd, raw, PVS_FLAG_ZLIB, scratch) || recv_ack(fd)) goto out;
    close(fd);

    /* A7 NAK path: reconnect, garbage magic -> expect 0x15 then close */
    fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, "XXXX", 4);
        bad.comp_len = 16; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = 0;
        if (send_all(fd, &bad, sizeof bad)) { fprintf(stderr, "FAIL: send bad hdr\n"); goto out; }
        uint8_t b;
        if (recv_byte(fd, &b) != 1 || b != PVS_NAK) {
            fprintf(stderr, "FAIL: expected NAK on bad header\n"); goto out;
        }
        if (recv_byte(fd, &b) != 0) {
            fprintf(stderr, "FAIL: connection not closed after bad-header NAK\n"); goto out;
        }
        printf("test: NAK + close on bad header OK\n");
        close(fd);
    }

    /* ================= section B: v2 delta suite ======================== */

    fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: connect for delta suite\n"); goto out; }

    gen_frame(raw, 100);
    memcpy(prev, raw, PVS_FRAME_RAW);

    /* B0: delta with no anchor on a fresh connection -> NAK, conn OPEN */
    if (send_delta(fd, raw, prev, PVS_FLAG_RLE, mask, scratch, 0) ||
        recv_nak_conn_open(fd)) goto out;
    printf("test: unanchored delta NAKed, connection kept\n");

    /* B1..B100: 100 evolving frames, GOP 10, reconnect at 50,
     * back-to-back deltas at 70/71, delta-zlib at 80, raw-mask at 81 */
    for (int i = 0; i < 100; i++) {
        int keyframe = (i % 10 == 0);
        if (i > 0) mutate_frame(raw, 1000 + i);

        if (i == 50) {   /* reconnect mid-stream: receiver anchor is lost */
            close(fd);
            fd = connect_retry(TEST_PORT);
            if (fd < 0) { fprintf(stderr, "FAIL: mid-stream reconnect\n"); goto out; }
            /* sender wrongly continues with a delta -> NAK, conn open */
            if (send_delta(fd, raw, prev, PVS_FLAG_RLE, mask, scratch, 0) ||
                recv_nak_conn_open(fd)) goto out;
            printf("test: post-reconnect delta NAKed (anchor lost)\n");
            keyframe = 1;                       /* NAK-triggers-keyframe */
        }

        if (i == 74) {   /* two deltas back-to-back, no ACK wait (drop path) */
            uint8_t *raw75 = malloc(PVS_FRAME_RAW);
            uint8_t *scr2 = malloc(3 * PVS_FRAME_RAW);
            if (!raw75 || !scr2) { perror("malloc"); goto out; }
            memcpy(raw75, raw, PVS_FRAME_RAW);
            mutate_frame(raw75, 1000 + 75);
            /* frame 74 delta vs prev, frame 75 delta vs frame 74 */
            for (size_t k = 0; k < PVS_FRAME_RAW; k++) mask[k] = raw[k] ^ prev[k];
            uint32_t c74 = (uint32_t)pvs_rle_encode(mask, PVS_FRAME_RAW, scratch);
            for (size_t k = 0; k < PVS_FRAME_RAW; k++) mask[k] = raw75[k] ^ raw[k];
            uint32_t c75 = (uint32_t)pvs_rle_encode(mask, PVS_FRAME_RAW, scr2);
            if (send_hdr_payload(fd, PVS_FLAG_DELTA | PVS_FLAG_RLE, scratch, c74) ||
                send_hdr_payload(fd, PVS_FLAG_DELTA | PVS_FLAG_RLE, scr2, c75))
                goto out;
            sent_crc[n_sent++] = crc32(0L, raw,   PVS_FRAME_RAW);
            sent_crc[n_sent++] = crc32(0L, raw75, PVS_FRAME_RAW);
            printf("test: sent deltas %d+%d back-to-back (comp=%u,%u)\n",
                   n_sent - 2, n_sent - 1, c74, c75);
            if (recv_ack(fd) || recv_ack(fd)) goto out;
            memcpy(raw, raw75, PVS_FRAME_RAW);   /* ground truth = frame 75 */
            memcpy(prev, raw, PVS_FRAME_RAW);
            free(raw75); free(scr2);
            i++;                                  /* consumed 74 and 75 */
            continue;
        }

        if (keyframe) {
            if (send_frame(fd, raw, PVS_FLAG_ZLIB, scratch) || recv_ack(fd))
                goto out;
        } else {
            uint16_t codec = PVS_FLAG_RLE;
            if (i == 82) codec = PVS_FLAG_ZLIB;   /* zlib'd full mask */
            if (i == 83) codec = 0;               /* bare raw XOR mask */
            if (i == 84) codec = PVS_FLAG_RLE | PVS_FLAG_ZLIB; /* 0x7 wire form */
            if (send_delta(fd, raw, prev, codec, mask, scratch, 1) ||
                recv_ack(fd)) goto out;
        }
        memcpy(prev, raw, PVS_FRAME_RAW);
    }
    printf("test: delta suite: 100 frames streamed OK\n");
    close(fd);
    fd = -1;

    /* stop daemon, harvest its log, verify per-frame CRCs in order */
    kill(pid, SIGINT);
    {
        FILE *lf = fdopen(pfd[0], "r");
        char line[512];
        uint32_t got_crc[MAX_FRAMES];
        int n_got = 0, n_flipped = 0, n_dropped = 0;
        int n_verify_ok = 0, n_verify_fail = 0, n_nak_anchor = 0;
        while (fgets(line, sizeof line, lf)) {
            fputs(line, stdout);
            char *p = strstr(line, "crc=");
            if (p && strstr(line, "FRAME ")) {
                if (n_got < MAX_FRAMES)
                    got_crc[n_got] = (uint32_t)strtoul(p + 4, NULL, 16);
                n_got++;
                if (strstr(line, "flipped")) n_flipped++;
                if (strstr(line, "DROPPED")) n_dropped++;
            }
            if (strstr(line, "VERIFY OK"))   n_verify_ok++;
            if (strstr(line, "VERIFY FAIL")) n_verify_fail++;
            if (strstr(line, "delta frame without anchor")) n_nak_anchor++;
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
        if (n_verify_fail) {
            fprintf(stderr, "FAIL: %d bank-union VERIFY FAIL\n", n_verify_fail);
            goto out;
        }
        if (n_verify_ok != n_sent) {
            fprintf(stderr, "FAIL: VERIFY OK %d != %d frames\n", n_verify_ok, n_sent);
            goto out;
        }
        if (n_nak_anchor != 2) {
            fprintf(stderr, "FAIL: expected 2 unanchored-delta NAKs, got %d\n",
                    n_nak_anchor);
            goto out;
        }
        if (n_flipped < 1) { fprintf(stderr, "FAIL: no frame ever flipped\n"); goto out; }
        printf("test: %d frames, all CRCs match (%d flipped, %d dropped), "
               "bank-union verified %d/%d, %d anchor NAKs\n",
               n_got, n_flipped, n_dropped, n_verify_ok, n_got, n_nak_anchor);
    }

    printf("PASS\n");
    rc = 0;
out:
    if (fd >= 0) close(fd);
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    return rc;
}
