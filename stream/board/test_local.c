/*
 * test_local.c - x86 loopback test for pov_rxd v2 (SIM_NO_DEVMEM build).
 *
 * Spawns ./pov_rxd_sim (--crc --flip-window dual) on a local port, connects,
 * and exercises:
 *   - all three codecs (raw / zero-run RLE / zlib), ACK-paced
 *   - DELTA chain: zlib keyframe + 3x DELTA|ZLIB, asserts the daemon's
 *     logged crc32 of every reconstructed raw frame == crc of what the
 *     PC-side raw was (proves prev_acked_raw ^ decoded reconstruction)
 *   - two frames back-to-back with no ACK wait (ready-buffer overwrite /
 *     drop path; both must still ACK - RX is decoupled from the flip)
 *   - NAK cases, each on a fresh connection (daemon closes after NAK):
 *     garbage magic / unknown flag bit / DELTA as first frame (no
 *     reference) / DELTA first after reconnect (reference must reset)
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

#include "../protocol.h"

#define TEST_PORT 9517
#define MAX_FRAMES 32

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
static int send_frame(int fd, const uint8_t *raw, const uint8_t *prev,
                      uint16_t flags, uint8_t *xbuf, uint8_t *scratch)
{
    pvs_hdr_t h;
    memcpy(h.magic, PVS_MAGIC, 4);
    h.raw_len = PVS_FRAME_RAW;
    h.n_slices = PVS_N_SLICES;
    h.flags = flags;

    const uint8_t *body = raw;               /* what the codec layer sees */
    if (flags & PVS_FLAG_DELTA) {
        if (!prev) { fprintf(stderr, "test bug: DELTA without prev\n"); return -1; }
        for (size_t i = 0; i < PVS_FRAME_RAW; i++)
            xbuf[i] = raw[i] ^ prev[i];
        body = xbuf;
    }

    const uint8_t *payload = body;
    if (flags & PVS_FLAG_ZLIB) {
        uLongf clen = compressBound(PVS_FRAME_RAW);
        if (compress2(scratch, &clen, body, PVS_FRAME_RAW, 6) != Z_OK) return -1;
        h.comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (flags & PVS_FLAG_RLE) {
        h.comp_len = (uint32_t)rle_encode(body, PVS_FRAME_RAW, scratch);
        payload = scratch;
    } else {
        h.comp_len = PVS_FRAME_RAW;
    }

    if (send_all(fd, &h, sizeof h) || send_all(fd, payload, h.comp_len))
        return -1;
    sent_crc[n_sent++] = crc32(0L, raw, PVS_FRAME_RAW);
    printf("test: sent frame %d flags=0x%x comp=%u crc=%08x\n",
           n_sent - 1, flags, h.comp_len, sent_crc[n_sent - 1]);
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

/* send a bare header, expect NAK then close (daemon NAKs before payload) */
static int expect_nak_close(int fd, const pvs_hdr_t *h, const char *what)
{
    if (send_all(fd, h, sizeof *h)) { fprintf(stderr, "FAIL: send hdr (%s)\n", what); return -1; }
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

    uint8_t *raw  = malloc(PVS_FRAME_RAW);
    uint8_t *raw2 = malloc(PVS_FRAME_RAW);
    uint8_t *xbuf = malloc(PVS_FRAME_RAW);
    uint8_t *scratch = malloc(compressBound(PVS_FRAME_RAW) + PVS_FRAME_RAW);
    if (!raw || !raw2 || !xbuf || !scratch) { perror("malloc"); return 1; }

    int rc = 1;

    /* 1-3: one frame per codec, ACK-paced (normal sender behaviour) */
    gen_frame(raw, 1);
    if (send_frame(fd, raw, NULL, 0, xbuf, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 2);
    if (send_frame(fd, raw, NULL, PVS_FLAG_RLE, xbuf, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 3);
    if (send_frame(fd, raw, NULL, PVS_FLAG_ZLIB, xbuf, scratch) || recv_ack(fd)) goto out;

    /* 4-6: DELTA chain vs the last ACKed frame (seed 3 is the reference).
     * Daemon must rebuild raw = prev ^ decoded; crc check below proves it. */
    memcpy(raw2, raw, PVS_FRAME_RAW);              /* raw2 = prev (seed 3) */
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

    /* NAK 2: unknown flag bit (bit3) -> mask check must reject */
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, PVS_MAGIC, 4);
        bad.comp_len = 1024; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = (1u << 3);
        fd = connect_retry(TEST_PORT);
        if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
        if (expect_nak_close(fd, &bad, "unknown flag bit")) goto out;
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
        int n_got = 0, n_flips = 0;
        while (fgets(line, sizeof line, lf)) {
            fputs(line, stdout);
            char *p = strstr(line, "crc=");
            if (p && strstr(line, "FRAME "))
                got_crc[n_got++] = (uint32_t)strtoul(p + 4, NULL, 16);
            if (strstr(line, "FLIP "))
                n_flips++;
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
        printf("test: %d frames, all CRCs match (delta rebuilt OK), %d flips\n",
               n_got, n_flips);
    }

    printf("PASS\n");
    rc = 0;
out:
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    return rc;
}
