# PVS1 — POV volumetric display streaming protocol

PC sender (`stream/pc/povstream.py`) → board receiver over **TCP** (PC connects
to board, default port **9500**). C header for the receiver:
[`stream/protocol.h`](../protocol.h) — keep in sync with this file.

## Frame geometry

One model frame = **`n_slices`** angular slices for the FS03 160×180 panel.
Each slice = 11664 packed data bytes padded to `0x3000` (12288); packing is the
hardware-verified `tools/pack_obs.py` DDR mirror contract
(`lane*1296 + row*24 + word*4`, LE u32 words) — **do not change**.

```
raw_len = n_slices * 0x3000        # <= PVS_FRAME_RAW_MAX = 720*0x3000 = 8,847,360
传统单面帧: n_slices = 360 → 4,423,680 bytes
```

### 2026-07-31: 帧长度不再是常量 (机械 v3.1 偏心屏)

机械 v3.1 把双面屏模组整体偏心 6.7mm，两个 LED 面到转轴的垂距变成
**0mm（面 A，穿心）** 和 **13.4mm（面 B）** —— 两面不再关于转轴对称，
于是「屏B@θ ≡ 屏A@(θ+180)」**作废**，两面必须各渲一份数据。

**头里的 `n_slices` 是权威**，接收方按它算长度，不再硬校验 == 360。
老帧（`n_slices=360`、无新 flag）语义逐字节不变。

| flag | 含义 |
|---|---|
| `DUAL_FACE` (bit3) | 载荷 = `[面A 全部片][面B 全部片]` 顺序拼接 |
| `FOLD_A` (bit4) | 面 A 只送前半圈 180 片（θ=0..179°），后半圈由 PL 取 `idx-180` 再做镜像置换补齐 |

**合法组合（接收方强制校验，不符直接 NAK）**：

| 组合 | `n_slices` | 说明 |
|---|---|---|
| 无新 flag | 360 | 老的单面帧 |
| `FOLD_A` | 180 | 单面折叠 |
| `DUAL_FACE` | 720 | 双面各 360 |
| `DUAL_FACE\|FOLD_A` | 540 | 面A 折叠 180 + 面B 360 |

### DUAL_FACE 的载荷 = 两条独立压缩流

```
payload = [u32 LE comp_len_A][面A 压缩流][面B 压缩流]
comp_len_A = 面A 流字节数;  面B 流长度 = comp_len - 4 - comp_len_A
comp_len 含这 4 字节前缀。未压缩 (flags 无 RLE/ZLIB) 时前缀同样存在,
此时 comp_len_A == nA*0x3000 且 comp_len == 4 + raw_len。
```

**为什么拆两条**：两面可以**并行解压到两个 CPU 核**。实测拆流的压缩代价是
**−0.12% ~ +0.04%**（有时反而更小）—— 因为 zlib 窗口只有 32 KB，而两面在载荷里
相距 2.2–4.4 MB，本来就不存在可丢的跨面后向引用；连"两面数据完全相同"的极端
构造也只多 129 字节。基本是白拿。

⚠ **并行加速比取决于两面的大小比**。`DUAL_FACE|FOLD_A` 时面A 180 片 / 面B 360 片
= **1:2**，双核并行由大的那面封顶 ⇒ 上限 1.5×（实测 1.42×），**不是 2×**。
对称的 720 片才吃满 2×。

单面帧（无 `DUAL_FACE`）**没有这个前缀**，排布逐字节不变。

**`FOLD_A` 的合法前提**：仅当该面**穿心**（垂距 0）时，`slice_i ≡ mirror(slice_{i+180})`
才严格成立。已在打包域独立验证：RTL 的镜像置换在 9×54=486 格上是双射且对合，
且严格等于观察者域 `X → 159-X`；穿心面 8/8 满足恒等式，偏心面 0/8。
发送端置这个 flag 前**必须**先跑几何自检（`gen_anime_slices.check_meridian_mirror`）。

⚠ **折叠的代价**：Bayer 抖动相位是逐 slot 变的，折叠后 PL 拿 slot `i` 的数据镜像出
slot `i+180`，抖动相位一并被复制 → 每转的相位多样性从 360 种降到 180 种，
时域抖动平滑效果打折。这不是 bug，别试图让折叠输出与完整 360 片输出逐字节相同。

## Wire format

Stream = repeated `[header | payload]`, connection close ends the stream.
All integers **little-endian**.

| off | size | field      | value                                          |
|-----|------|------------|------------------------------------------------|
| 0   | 4    | magic      | `'PVS1'`                                       |
| 4   | 4    | comp_len   | payload bytes as transmitted                   |
| 8   | 4    | raw_len    | decompressed size, **must == n_slices * 0x3000** |
| 12  | 2    | n_slices   | 权威片数, 1..720 (老帧 = 360)                  |
| 14  | 2    | flags      | bit0 = RLE, bit1 = zlib, bit2 = DELTA, bit3 = DUAL_FACE, bit4 = FOLD_A, 0 = raw |
| 16  | comp_len | payload |                                                |

**DELTA (bit2, PVS1.1)**: payload (after RLE/zlib decode) = `cur XOR prev_raw`,
where `prev_raw` is the previous successfully-ACKed raw frame of this
connection. Composes with zlib: `ZLIB|DELTA` = `zlib(prev ^ cur)`. The first
frame of a connection MUST NOT set DELTA (keyframe); the receiver NAKs a DELTA
frame with no reference. Keyframe cadence (default every 26 frames, and on
(re)connect) is sender policy, not protocol.

🔴 **DELTA 不能跨几何变化 (2026-07-31)**：参考帧必须与当前帧**布局**一致 ——
不只是长度一致。只要 `n_slices` 或面布局 flags（`DUAL_FACE`/`FOLD_A`）相对上一帧
发生任何变化，这一帧**必须强制走关键帧**。

⚠ 为什么"只比长度"不够：**540 单面** 和 **540 双面折叠**（180+360）的 `raw_len`
完全相同，但面边界不同 —— 拿一个当另一个的参考帧做 XOR，会把两面的数据搅在一起，
而且**长度校验完全发现不了**。接收方比对的是 `(raw_len, face_b_off)` 两项，不符即 NAK；
但避免它是**发送端的责任**（发送端切换几何时应先排空发送窗口再发关键帧）。

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
