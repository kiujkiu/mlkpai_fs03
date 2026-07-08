# pov_rxd — board-side PVS1 stream receiver

TCP daemon for the Zynq-7020 (MLKPAI-FS03, Debian buster, kernel 6.6) that
receives compressed POV frames from the PC sender
(`stream/pc/povstream.py`), decompresses them into a reserved-DDR double
buffer, and flips the PL POV engine (`0x40010000`) to the new bank at the
slice-counter wrap so the display never glitches or blocks.

Protocol: [`stream/protocol.h`](../protocol.h) +
[`stream/pc/protocol.md`](../pc/protocol.md) (PVS1: 16 B header, zlib
default / zero-run RLE / raw, 1-byte ACK per frame).

## Memory / hardware assumptions

- Linux boots with `mem=256M` → phys `0x10000000..0x1FFFFFFF` is invisible
  to the kernel and reserved for frames (boot setup is handled elsewhere).
- Double buffer: bank A @ `0x10000000`, bank B @ `0x10500000`, each
  `360 × 0x3000 = 4,423,680` B.
- On start the daemon writes bank A to `slice_base` (0x18). It does NOT
  touch `POV_CTRL` (0x10) — the JTAG side owns mode config — unless you
  pass `--fake`.
- Cache coherency: on 32-bit ARM (kernel 6.6, `arch/arm/mm/mmu.c
  phys_mem_access_prot()`), an mmap of `/dev/mem` for a pfn with
  `!pfn_valid()` — true for both the frame region above `mem=256M` and the
  PL register page — is mapped **uncached** (`pgprot_noncached`). CPU
  stores therefore reach DRAM directly and the PL HP port sees them with
  no cache flushing. The daemon additionally issues `__sync_synchronize()`
  (DMB) after the frame memcpy and around the `slice_base` write so the
  data is globally observable before the engine can latch the new base.
  (If a future kernel were built with `CONFIG_IO_STRICT_DEVMEM` or the
  region became kernel-managed RAM, revisit this — symptom would be stale
  or torn frames.)

## Build (WSL / dev machine)

```sh
cd stream/board
make            # cross ARM static binary ./pov_rxd (arm-linux-gnueabihf-gcc)
make test       # x86 sim build + loopback protocol/double-buffer test
```

The ARM binary is fully **static** (glibc + `deps/arm/libz.a`, zlib 1.3.1
cross-built, prebuilt copy committed; `make deps` re-fetches/rebuilds it),
so the board needs no toolchain and no matching libz. The binary itself is
committed too — the board may have no compiler.

## Deploy to the board

Board WiFi IP was `10.168.168.189` (DHCP — may change, check the router or
serial console). Users: `uisrc` / `root`.

```sh
scp stream/board/pov_rxd uisrc@10.168.168.189:/home/uisrc/
```

## Run (no systemd, just nohup)

`/dev/mem` requires root — run as root (or `sudo`):

```sh
ssh root@10.168.168.189
nohup /home/uisrc/pov_rxd > /var/log/pov_rxd.log 2>&1 &
tail -f /var/log/pov_rxd.log       # timestamped per-frame log lines
```

Stop with `kill -INT <pid>` (clean exit; SIGTERM also handled). The listen
socket uses `SO_REUSEADDR`, so an immediate restart never hits
`EADDRINUSE`.

Options:

```
--port N       TCP listen port                 (default 9500)
--base ADDR    frame region phys base          (default 0x10000000)
--regs ADDR    POV engine AXI base             (default 0x40010000)
--fake RPS     motor-less test: program fake_period (0x14) and
               POV_CTRL (0x10) for RPS revolutions/sec fake spin
```

Example — bench test without the motor, 15 rps:

```sh
nohup /home/uisrc/pov_rxd --fake 15 > /var/log/pov_rxd.log 2>&1 &
```

Then stream from the PC: `python stream/pc/povstream.py stream --host
10.168.168.189 ...` (see `stream/pc/`).

## Behaviour notes

- One client at a time; on disconnect the daemon goes back to `accept()`
  (state, including the active bank, persists across connections).
- Frames are decompressed into a RAM staging buffer, then `memcpy`'d into
  the **inactive** bank; the flip (write to 0x18) waits until the engine's
  `slice_idx` (low 16 bits of 0x10) wraps below 8, so a frame swap always
  lands at a revolution boundary. If the counter never wraps within 2 s
  (engine idle / not spinning) it flips anyway with a warning.
- If the sender outruns the display (new data already pending on the
  socket before the flip), the just-received frame is ACKed and **dropped**
  (no flip) — the display is never blocked, the newest frame wins.
- Bad header / failed decompress → 1-byte NAK (0x15) and the connection is
  closed; the sender reconnects.
- Every frame logs `FRAME seq=… crc=… bank=… flipped|DROPPED` — the CRC32
  of the decompressed frame, handy for end-to-end payload verification
  against the sender.
