# PVS1 — POV volumetric display streaming protocol

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
| 0   | 4    | magic      | `'PVS1'`                                       |
| 4   | 4    | comp_len   | payload bytes as transmitted                   |
| 8   | 4    | raw_len    | decompressed size, **must be 4423680**         |
| 12  | 2    | n_slices   | **360**                                        |
| 14  | 2    | flags      | bit0 = RLE, bit1 = zlib, bit2 = DELTA, 0 = raw |
| 16  | comp_len | payload |                                                |

**DELTA (bit2, PVS1.1)**: payload (after RLE/zlib decode) = `cur XOR prev_raw`,
where `prev_raw` is the previous successfully-ACKed raw frame of this
connection. Composes with zlib: `ZLIB|DELTA` = `zlib(prev ^ cur)`. The first
frame of a connection MUST NOT set DELTA (keyframe); the receiver NAKs a DELTA
frame with no reference. Keyframe cadence (default every 26 frames, and on
(re)connect) is sender policy, not protocol.

### Flow control (ACK)

After each frame is received, decompressed and verified (raw_len match), the
board sends **1 byte**: `0x06` (ACK, send next) or `0x15` (NAK, sender aborts).
Sender does not transmit frame N+1 until frame N is ACKed — this self-paces the
link and bounds board-side buffering to one frame.
Exception (PVS1.1, design doc §2.3): on a NAK for a **DELTA** frame the sender
does not abort — it immediately re-sends that frame as a keyframe and
continues (board restart / lost reference recovery).

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
