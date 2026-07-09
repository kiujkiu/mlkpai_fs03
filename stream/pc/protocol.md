# PVS — POV volumetric display streaming protocol (PVS1 + v2 delta)

PC sender (`stream/pc/povstream.py`) → board receiver over **TCP** (PC connects
to board, default port **9500**). C header for the receiver:
[`stream/protocol.h`](../protocol.h) — keep in sync with this file.

## Frame geometry

One model frame = **360** angular slices for the FS03 160×180 panel.
Each slice = 11664 packed data bytes padded to `0x3000` (12288); packing is the
hardware-verified `tools/pack_obs.py` DDR mirror contract
(`lane*1296 + row*24 + word*4`, LE u32 words) — **do not change**.

```
FRAME_RAW = 360 * 0x3000 = 4,423,680 bytes
```

## Wire format

Stream = repeated `[header | payload]`, connection close ends the stream.
All integers **little-endian**.

| off | size | field      | value                                          |
|-----|------|------------|------------------------------------------------|
| 0   | 4    | magic      | `'PVS1'` (also for v2 frames — no magic bump)  |
| 4   | 4    | comp_len   | payload bytes as transmitted                   |
| 8   | 4    | raw_len    | decompressed size, **must be 4423680**         |
| 12  | 2    | n_slices   | **360**                                        |
| 14  | 2    | flags      | bit0 = RLE, bit1 = zlib, bit2 = delta (v2), 0 = raw |
| 16  | comp_len | payload |                                                |

### Flow control (ACK)

After each frame is received, decompressed and verified (raw_len match), the
board sends **1 byte**: `0x06` (ACK, send next) or `0x15` (NAK, sender aborts;
v2 exception below). Sender does not transmit frame N+1 until frame N is
ACKed — this self-paces the link and bounds board-side buffering to one frame.

## v2: delta frames (flags bit2)

A delta frame's payload decodes to a **XOR mask against the previous raw
frame**. Accepted wire forms (`flags` values):

| flags | payload                                     | use                |
|-------|---------------------------------------------|--------------------|
| 0x7   | zlib-wrapped zero-run-RLE of the XOR mask   | **normal form** — sender picks the smaller of RLE / zlib(RLE) |
| 0x5   | bare zero-run-RLE of the XOR mask           | when zlib wrap doesn't shrink |
| 0x6   | zlib of the full 4.4 MB mask                | accepted, not sent |
| 0x4   | raw 4.4 MB mask (`comp_len == raw_len`)     | accepted, not sent |

The RLE stream must decode to exactly `raw_len` bytes. Zero runs mean "bytes
unchanged"; literals XOR into the frame — the receiver applies the stream
directly to its **cached shadow copy** of the current frame, so decode cost
scales with changed bytes, not frame size. `run == 0` escapes are legal no-ops.

### Keyframes, GOP, recovery

- A **keyframe** is any non-delta frame (normally zlib, i.e. a legal PVS1
  frame). The **first frame of every connection must be a keyframe** — the
  receiver's delta anchor is per-connection.
- Sender (`povstream.py --codec delta`) emits a keyframe every `--gop` N
  frames (default 30, `0` = only when required), after every reconnect, and
  whenever the delta degenerates (RLE payload ≥ raw/4, e.g. scene cut).
- **NAK-keyframe recovery**: a delta frame the receiver cannot anchor (fresh
  connection / lost state) is NAKed **without closing the connection**; the
  sender re-encodes the same frame as a keyframe and resends. All other NAKs
  keep PVS1 semantics (connection closed / sender aborts). Old PVS1 senders
  never set bit2 and are entirely unaffected.

### Receiver double-buffer: the bank-union invariant

The board flips between two DDR banks (A/B) at revolution boundaries, and the
`/dev/mem` mapping of the banks is **uncached** — full 4.4 MB writes cost
50–90 ms on the A9, so v2 writes only what changed. The receiver keeps:

- a cached malloc'd **shadow** = current raw frame (deltas applied here);
- per-bank **dirty accumulators** = set of 4 KiB pages where that bank
  differs from the shadow.

Each frame: this frame's dirty pages are OR-ed into *both* accumulators, then
the inactive bank's accumulated dirty **spans** are memcpy'd from the shadow
and its accumulator cleared. Because banks normally alternate, the inactive
bank is 2 frames stale and its accumulator holds the union of the last two
deltas — but the invariant `bank == shadow on every non-accumulated page`
holds by construction across *any* history: drops (same bank synced twice),
keyframes (all pages dirty), reconnects. `pov_rxd --verify` memcmps the full
bank against the shadow after every frame; the loopback test runs 100+ frames
of mixed keyframes/deltas/drops/reconnects with verify on.

### v2 measured numbers

x86 (WSL, `stream/board/bench_delta`, real adjacent frames), wire sizes:

| corpus (motion type)      | changed | 0x7 delta wire | zlib-6 full | delta/zlib |
|---------------------------|---------|----------------|-------------|------------|
| frames_palace (orbit+zoom)| 1.56 %  | **15.7 KB** (281×) | 33.8 KB | **2.1× smaller** |
| frames_globe (rotation)   | 1.27 %  | **55.0 KB** (80×)  | 98.8 KB | 1.8× smaller |
| frames_asol_idle (idle)   | ~2 %    | 85 KB          | 82 KB       | ≈ parity   |
| frames_spinpulse (dither) | 7.0 %   | 258 KB         | 233 KB      | ≈ parity   |
| static repeat             | 0 %     | 0.2 KB         | 30.6 KB     | 150×       |

Dithered sources (Bayer phase rotates per frame → whole-frame XOR noise) gain
nothing on the wire but still gain the fast decode. Board-side per-frame cost
(x86 measured → A9 estimated at 5–10×):

| step                | x86     | A9 est.   | PVS1 equivalent (A9 measured) |
|---------------------|---------|-----------|-------------------------------|
| inflate+apply (0x7) | 0.3–0.6 ms | **1.5–6 ms** | zlib inflate 4.4 MB: 100–200 ms |
| dirty-span bank write | 0.2 ms | ~0.9× of 50–90 ms (scales with dirty pages) | full 4.4 MB uncached: 50–90 ms |

Decode is no longer the bottleneck (≈ 60× faster). The remaining 30 fps wall
is the **uncached bank write**, which now scales with dirty pages: full-frame
motion (orbit/rotation) dirties ~90 % of pages → ~45–80 ms → 12–20 fps;
moderate/low motion → within the 33 ms budget. Wire is comfortable: 15–85 KB
per delta @ 3.5 MB/s WiFi = 40–200 fps. Ground truth: run the committed ARM
`pov_rxd --bench` on the board and read the `STAT` lines (avg/max recv /
decode / write µs every 30 frames), with the PC side
`povstream.py stream --codec delta --bench`.

## Compression — measured on a real rendered frame

Corpus: `tools/anime_slices.bin` (real 360-slice anime frame, 94.1% zero bytes),
WSL Python 3, single core:

| codec         | size      | ratio  | encode | decode | receiver code    |
|---------------|-----------|--------|--------|--------|------------------|
| raw           | 4,423,680 | 1.0×   | —      | —      | memcpy           |
| zero-run RLE  |   848,922 | 5.2×   | 46 ms  | 36 ms  | ~15 lines C      |
| **zlib -1**   |   476,928 | 9.3×   | 16 ms  | 9 ms   | `uncompress()` one-liner |
| **zlib -6**   |   341,850 | 12.9×  | 41 ms  | 7 ms   | same             |

**Choice: zlib (flags bit1), default level 6.** It beats the tuned zero-RLE
~2× on ratio *and* is simpler on both ends (Python `zlib.compress`, C
`uncompress()` from zlib1g-dev on Debian). RLE is kept specified (bit0) as a
zero-dependency fallback: `0x00` escape byte followed by `run:u16le` emits that
many zeros; any other byte is a literal (raw zero bytes never appear bare).

Projected model fps on a 9.4 MB/s link (payload-bound, header/ACK negligible):
`9.4 / (4.42/ratio)` → **~20 fps** @ zlib-1, **~27 fps** @ zlib-6, 1.1 fps raw.
Actual figures vary ±20% with scene density; povstream prints live stats.

## Sender pipeline

```
source (spinpulse GLB anim | procedural globe | dir of .bin)
  → per-frame: transform points → voxelize → 360 slice renders
  → 1-bit Bayer dither (phase varies per slice AND per frame)
  → pack_obs.pack_slice → 360×0x3000 frame
  → zlib → PVS1 frame → TCP, wait ACK, pace to --fps
```

Because live numpy rendering costs seconds/frame, the normal workflow is
`povstream.py render` (prerender N frames to `stream/pc/frames_<name>/`)
then `povstream.py stream --dir` at target fps. `stream --anim` live-renders.

`fake_board.py` is a loopback receiver implementing this spec (decompress,
verify, sha256, optional slice-0 PNG via `pack_obs.unpack_slice`) for
hardware-free end-to-end testing.
