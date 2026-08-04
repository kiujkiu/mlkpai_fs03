# pov_rxd — board-side PVS1 stream receiver (v2, 26 页/秒)

TCP daemon for the Zynq-7020 (MLKPAI-FS03, Debian buster, kernel 6.6) that
receives compressed POV frames from the PC sender
(`stream/pc/povstream.py`), decompresses them (LZ4 / zlib / RLE / raw, 可选
DELTA 帧间差分), and flips the PL POV engine (`0x40010000`) between two
reserved-DDR banks inside a flip window so the display never glitches or
blocks.

Protocol: [`stream/protocol.h`](../protocol.h) +
[`stream/pc/protocol.md`](../pc/protocol.md) (PVS1: 16 B header, LZ4 /
zlib / zero-run RLE / raw, flags bit2 = DELTA, 1-byte ACK per frame).

## LZ4 (2026-08-04, v3.3): 12 fps → 48 fps

瓶颈一直在**板端解压**，不在链路。A9 单核实测 (`anime_dual720.bin`,
720 片双面 8,847,360 B): zlib-6 = 376,780 B / 163.5 ms / 51.6 MB/s，
lz4-HC9 = 388,166 B / 41.2 ms / 204.6 MB/s —— 压缩比只掉 3%，解压快 4×。
`DUAL_FACE` 的两条流照旧双核并行 ⇒ ≈20.6 ms/帧 ⇒ **48 fps**。
发送端: `povstream.py stream --codec lz4`（默认仍是 zlib，老的 `frames_*`
目录行为逐字节不变）。级别默认 **HC12**（比 HC9 小 4.5%，甚至比 zlib-6 还小）；
⚠ **HC10 实测比 HC9 差 6.4%，跳过 10**；HC12 编码 ≈1 s/帧 ⇒ 只能走 `--dir`
离线预压缩，现渲直推会被编码封顶。

## MSTREAM (bit6): 按工作量切流，不是按面切

`DUAL_FACE|FOLD_A`（面A 折 180 + 面B 360）按面切时两条流是 1:2，双核压不平：
makespan 被面B 的 360 片封顶 = 20.6 ms。切成 **180/90/270 三条**（`dec_plan` 连续
分组 → 核0 拿前两条 = 270 片，核1 拿第三条 = 270 片）降到 **15.47 ms**，
代价只有 +480 B（+0.18%）。
⚠ 顺带纠正一个旧说法：**`FOLD_A` 只省链路（−31%），解码一分钱不省** —— 按面切的
makespan 由面B 封顶，折不折叠都一样。

载荷 = `[u32 n][n×{u32 comp_len_i, u32 n_slices_i}][流…]`，两个求和自校验
（`Σcomp_len` / `Σn_slices`）不符一律 NAK。**分组必须连续不能轮转**（轮转会得到
450/90，比按面切还差）。没置 `MSTREAM` 的帧照旧走单流/按面两流老分支。
发送端: `povstream.py stream --stream-split balanced`（默认 `face` = 老行为）。
日志里 `decode plan: 3 条流 -> core0=流[0..1] 270片, core1=流[2..2] 270片` 一行
就是派活结果，上板调参看它。

🔴 只认 **LZ4 raw block**（`LZ4_decompress_safe`），不认 `lz4` 命令行那种
`.lz4` 帧格式（带 `0x184D2204` 魔数 + xxhash，会被当 token 解出负数）。
raw block 不带原长，`dstCapacity` 由 `hdr.raw_len` / 各面的 `nX*0x3000` 给。

**liblz4 是静态链进来的**（`deps/arm/liblz4.a`，lz4 1.10.0 交叉编译，与
`libz.a` 同一套做法）。板上虽然有 `liblz4.so.1.8.3`，但没有开发包，而且这里
的 ARM 产物是 `-static` 的：静态可执行文件不能 `-l:liblz4.so.1`，静态 glibc
下 `dlopen()` 也基本不可用。详见 `pov_rxd.c` 里 `#include <lz4.h>` 上方的注释。

v2 design: `docs/design_icnd2047/04_sw_stream_26fps.md` §3 — 三缓冲 +
RX/flip 双线程 (ACK 与翻页解耦), DELTA 重建, WC 映射 (povmem.ko), 半圈
双窗口翻页, crc32 挪到 `--crc` 后面。

## Memory / hardware assumptions

- Linux boots with `mem=256M` → phys `0x10000000..0x1FFFFFFF` is invisible
  to the kernel and reserved for frames (boot setup is handled elsewhere).
- DDR double buffer (v3.1, 权威表见 `pov_rxd.c` 文件头): bank A @
  `0x10000000`, bank B @ `0x11000000` (`BANK_STRIDE` = 16 MB), each using
  `0..0x870000` (`720 × 0x3000` = 8,847,360 B = 双面帧最大尺寸); bank C @
  `0x12000000` 为三缓冲预留但当前不映射。整窗 mmap 长度
  `FRAME_MAP_LEN = 0x1870000` (24.4 MB) —— 所以 `povmem.ko` 必须
  `size >= 0x1870000` (模块默认已是 `0x1900000`)。另有 3 个 cached malloc
  staging 缓冲 (写入/就绪/拷贝) 在 RX 线程和 flip 线程之间轮转。
- On start the daemon writes bank A to `slice_base` (0x18) and 0 to
  `slice_base_b` (0x28, 面B 基址; 0 = PL 两面都用 0x18). Dual-face frames
  set 0x28 = bank + nA×0x3000. It does NOT
  touch `POV_CTRL` (0x10) — the JTAG side owns mode config — unless you
  pass `--fake`.
- Frame-region mapping: `/dev/povmem` (povmem.ko, **write-combine**,
  memcpy ~300–800 MB/s) preferred; falls back to `/dev/mem`
  (strongly-ordered uncached, ~60–120 MB/s) when the module isn't loaded.
  The PL register page always uses `/dev/mem` (registers want SO). Either
  way nothing is cached in L1/L2, so the PL HP port needs no cache
  maintenance; WC being weakly ordered, the daemon issues a DSB after the
  bank memcpy and around the `slice_base` write.

## Build (WSL / dev machine)

```sh
cd stream/board
make            # cross ARM static binaries ./pov_rxd + ./bench_s0
make test       # x86 sim build + loopback protocol/delta/NAK/flip test
make bench      # bench_s0 (ARM) + bench_s0_x86 (local sanity run)
make ko         # povmem.ko - needs board kernel headers, see povmem/Makefile
```

The ARM binaries are fully **static** (glibc + `deps/arm/libz.a`, zlib
1.3.1 + `deps/arm/liblz4.a`, lz4 1.10.0, both cross-built with prebuilt
copies committed; `make deps` / `make deps-lz4` re-fetch/rebuild them), so
the board needs no toolchain and no matching libz/liblz4. NEON is enabled
(`-mfpu=neon`, Zynq A9 has it) for the DELTA XOR loop.

x86 的 `make sim` / `make test` 用 `-l:liblz4.so.1` 直接点 soname，所以
**只要运行时包 `liblz4-1`**，不需要 `liblz4-dev`（`-llz4` 才要 dev 包装的
`liblz4.so` 符号链接）；头文件借 `deps/arm/lz4.h`（纯 C 头，与架构无关）。

`povmem.ko` cannot be built on the dev machine without the board kernel
tree (6.6.0-xilinx headers): `make -C povmem KDIR=~/mlkpai-kernel/linux-xlnx
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-` once it's around, or build
natively on the board if linux-headers are installed.

## Local test (no board, x86 loopback)

```sh
cd stream/board
make test
```

Spawns `pov_rxd_sim` (`--crc --flip-window dual --fake 20`) on
localhost:9517 and asserts:

- raw / RLE / zlib frames each ACK and the daemon-logged crc32 of the
  decoded frame matches the sender's raw crc;
- a zlib keyframe + 3 `DELTA|ZLIB` frames rebuild to the exact raw frames
  (`raw = prev_acked_raw ^ decoded`, crc-verified);
- two frames sent back-to-back with no ACK wait both ACK (RX decoupled
  from the flip; newest-frame-wins drop path);
- NAK + close on: garbage magic, unknown flag bit (0x8), DELTA as the
  first frame of a connection, and DELTA right after a reconnect (the
  delta reference must reset per connection);
- at least one FLIP happened (flip thread + window logic alive).

`PASS` + exit 0 = good.

## S0 microbenchmark (bench_s0)

Board-side numbers that decide the §3.5-7 后手 (zlib-ng / zstd / 180 片):

```sh
scp stream/board/bench_s0 uisrc@<board>:/home/uisrc/
ssh root@<board> /home/uisrc/bench_s0            # 10 loops, median, ms
```

Prints one number per line: `so_memcpy_ms` (4.4 MB → /dev/mem SO map),
`wc_memcpy_ms` (→ /dev/povmem, `skip` if module not loaded), `inflate_ms`
(typical ~130 KB frame → 4.4 MB), `crc32_ms`, `xor_ms`. It writes to bank
B (`base+0x1000000`); don't run it while streaming. `--loops N --base ADDR`
supported. `bench_s0_x86` runs the three CPU-only items locally.

## Deploy to the board

Board WiFi IP was `10.168.168.189` (DHCP — may change, check the router or
serial console). Users: `uisrc` / `root`.

```sh
scp stream/board/pov_rxd uisrc@10.168.168.189:/home/uisrc/
# optional (WC copy speedup), once povmem.ko is built for 6.6.0-xilinx:
scp stream/board/povmem/povmem.ko root@10.168.168.189:/root/
# 窗口必须 >= 0x1870000, 否则 pov_rxd 的 mmap 失败并静默回落到慢的 /dev/mem
ssh root@10.168.168.189 insmod /root/povmem.ko base=0x10000000 size=0x1900000
```

## Run (no systemd, just nohup)

`/dev/mem` requires root — run as root (or `sudo`):

```sh
ssh root@10.168.168.189
nohup /home/uisrc/pov_rxd > /var/log/pov_rxd.log 2>&1 &
tail -f /var/log/pov_rxd.log       # per-frame FRAME/FLIP + 1 Hz STAT lines
```

Stop with `kill -INT <pid>` (clean exit; SIGTERM also handled). The listen
socket uses `SO_REUSEADDR`, so an immediate restart never hits
`EADDRINUSE`.

Options:

```
--port N            TCP listen port                 (default 9500)
--base ADDR         frame region phys base          (default 0x10000000)
--regs ADDR         POV engine AXI base             (default 0x40010000)
--fake RPS          motor-less test: program fake_period (0x14) and
                    POV_CTRL (0x10) for RPS revolutions/sec fake spin
--crc               crc32 + log every decoded frame (costs 11-18 ms/frame
                    on the A9; debug only, default off)
--flip-window MODE  single = flip near slice 0 only (default, 现行为);
                    dual   = also near slice 180 (双屏对置, 26 页/秒)
```

Example — dual-panel bench without the motor, 13 rps:

```sh
nohup /home/uisrc/pov_rxd --fake 13 --flip-window dual > /var/log/pov_rxd.log 2>&1 &
```

Then stream from the PC: `python stream/pc/povstream.py stream --host
10.168.168.189 ...` (see `stream/pc/`).

## Behaviour notes

- Two threads: the **RX thread** (recv → inflate → DELTA XOR rebuild →
  publish → **ACK immediately**) and the **flip thread** (grab the newest
  ready staging buffer → memcpy into the idle DDR bank → wait for a flip
  window → write `slice_base`). ACK pacing = decode throughput; the sender
  (`--fps 26+`) auto-locks to the page rate via the ACK gate.
- Flip window: `slice_idx < 8`, plus `|slice_idx − 180| < 8` in dual mode.
  A window that was just used must be left before it can trigger again
  (no double flips in one pass). If no window shows up within 2 s (engine
  idle / not spinning) it flips anyway (`FORCED`, counted in STAT).
- If the sender outruns the display, the ready buffer is overwritten in
  place — frames are ACKed and **dropped** (STAT `drop=`), the display is
  never blocked, the newest frame wins.
- DELTA (`flags` bit2): `raw = prev_acked_raw ^ decoded`; the reference is
  the previous successfully-ACKed frame, kept in cached RAM (never read
  back from the uncached banks). It resets on every (re)connect — a DELTA
  frame with no reference is NAKed and the sender falls back to a keyframe.
- Bad header / unknown flag bits / failed decompress → 1-byte NAK (0x15)
  and the connection is closed; the sender reconnects.
- Logs: per-frame `FRAME seq=… comp=… flags=…[ crc=…] dec=…ms`, per-flip
  `FLIP gen=… bank=… win=…`, plus a 1 Hz
  `STAT rx=… flip=… drop=… forced=… dec_avg=…` line while active.
- One client at a time; on disconnect the daemon goes back to `accept()`
  (the active bank persists across connections).
