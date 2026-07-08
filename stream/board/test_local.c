/*
 * test_local.c - x86 loopback test for pov_rxd (SIM_NO_DEVMEM build).
 *
 * Spawns ./pov_rxd_sim on a local port, connects, streams frames in all
 * three codecs (raw / zero-run RLE / zlib), verifies the ACK bytes, then
 * checks the CRC32 the daemon logged for every frame against the CRC of
 * what was sent (proves header parse + decompress + double-buffer copy).
 * Also sends two frames back-to-back with no ACK wait (exercises the
 * drop/no-flip path) and a garbage header (expects NAK + close).
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
#define MAX_FRAMES 16

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

static int send_frame(int fd, const uint8_t *raw, uint16_t flags,
                      uint8_t *scratch)
{
    pvs_hdr_t h;
    memcpy(h.magic, PVS_MAGIC, 4);
    h.raw_len = PVS_FRAME_RAW;
    h.n_slices = PVS_N_SLICES;
    h.flags = flags;

    const uint8_t *payload = raw;
    if (flags & PVS_FLAG_ZLIB) {
        uLongf clen = compressBound(PVS_FRAME_RAW);
        if (compress2(scratch, &clen, raw, PVS_FRAME_RAW, 6) != Z_OK) return -1;
        h.comp_len = (uint32_t)clen;
        payload = scratch;
    } else if (flags & PVS_FLAG_RLE) {
        h.comp_len = (uint32_t)rle_encode(raw, PVS_FRAME_RAW, scratch);
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
              "--fake", "20", (char *)NULL);
        perror("execl pov_rxd_sim");
        _exit(127);
    }
    close(pfd[1]);

    int fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: cannot connect to sim daemon\n"); kill(pid, SIGKILL); return 1; }

    uint8_t *raw = malloc(PVS_FRAME_RAW);
    uint8_t *scratch = malloc(compressBound(PVS_FRAME_RAW) + PVS_FRAME_RAW);
    if (!raw || !scratch) { perror("malloc"); return 1; }

    int rc = 1;

    /* 1-3: one frame per codec, ACK-paced (normal sender behaviour) */
    gen_frame(raw, 1);
    if (send_frame(fd, raw, 0, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 2);
    if (send_frame(fd, raw, PVS_FLAG_RLE, scratch) || recv_ack(fd)) goto out;
    gen_frame(raw, 3);
    if (send_frame(fd, raw, PVS_FLAG_ZLIB, scratch) || recv_ack(fd)) goto out;

    /* 4-5: precompress both, then send truly back-to-back with no ACK
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

    /* 6: one more paced frame to confirm daemon is still healthy */
    gen_frame(raw, 6);
    if (send_frame(fd, raw, PVS_FLAG_ZLIB, scratch) || recv_ack(fd)) goto out;
    close(fd);

    /* NAK path: reconnect, garbage magic -> expect 0x15 then close */
    fd = connect_retry(TEST_PORT);
    if (fd < 0) { fprintf(stderr, "FAIL: reconnect failed\n"); goto out; }
    {
        pvs_hdr_t bad;
        memcpy(bad.magic, "XXXX", 4);
        bad.comp_len = 16; bad.raw_len = PVS_FRAME_RAW;
        bad.n_slices = PVS_N_SLICES; bad.flags = 0;
        if (send_all(fd, &bad, sizeof bad)) { fprintf(stderr, "FAIL: send bad hdr\n"); goto out; }
        uint8_t b; ssize_t n;
        do n = recv(fd, &b, 1, 0); while (n < 0 && errno == EINTR);
        if (n != 1 || b != PVS_NAK) { fprintf(stderr, "FAIL: expected NAK, n=%zd b=0x%02x\n", n, n == 1 ? b : 0); goto out; }
        do n = recv(fd, &b, 1, 0); while (n < 0 && errno == EINTR);
        if (n != 0) { fprintf(stderr, "FAIL: connection not closed after NAK\n"); goto out; }
        printf("test: NAK + close on bad header OK\n");
    }

    /* stop daemon, harvest its log, verify per-frame CRCs in order */
    kill(pid, SIGINT);
    {
        FILE *lf = fdopen(pfd[0], "r");
        char line[512];
        uint32_t got_crc[MAX_FRAMES];
        int n_got = 0, n_flipped = 0, n_dropped = 0;
        while (fgets(line, sizeof line, lf)) {
            fputs(line, stdout);
            char *p = strstr(line, "crc=");
            if (p && strstr(line, "FRAME ")) {
                got_crc[n_got++] = (uint32_t)strtoul(p + 4, NULL, 16);
                if (strstr(line, "flipped")) n_flipped++;
                if (strstr(line, "DROPPED")) n_dropped++;
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
        if (n_flipped < 1) { fprintf(stderr, "FAIL: no frame ever flipped\n"); goto out; }
        printf("test: %d frames, all CRCs match (%d flipped, %d dropped)\n",
               n_got, n_flipped, n_dropped);
    }

    printf("PASS\n");
    rc = 0;
out:
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    return rc;
}
