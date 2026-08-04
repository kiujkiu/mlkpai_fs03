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
| 14  | 2    | flags      | bit0 = RLE, bit1 = zlib, bit2 = DELTA, bit3 = DUAL_FACE, bit4 = FOLD_A, bit5 = LZ4, bit6 = MSTREAM, 0 = raw |
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

### 2026-08-04: LZ4 (bit5) 顶掉 zlib —— 板端 12 fps → 48 fps

上面那张表是在 **PC** 上测的，选 zlib 时看的是压缩比。真正的瓶颈后来被证明在
**板端解压**：A9 单核 inflate 只有 ~52 MB/s，双面 8.85 MB 一帧要 163 ms，双核也就
82 ms ⇒ 封顶 12 fps，链路根本没跑满。

板上真实内容 `anime_dual720.bin`（720 片偏心双面，8,847,360 B）在 **ARM Cortex-A9
单核**实测（**单流**压的，不是拆两条流）：

| codec   | size    | ratio  | 解压耗时 | 解压吞吐   |
|---------|---------|--------|----------|------------|
| zlib-6  | 376,780 | 23.5×  | 163.5 ms | 51.6 MB/s  |
| **lz4-HC9** | **388,166** | **22.8×** | **41.2 ms** | **204.6 MB/s** |

压缩比只差 3%（每帧多 11 KB，2.4G WiFi 上可忽略），解压快 **4×**。
`DUAL_FACE` 的两条流照旧双核并行 ⇒ 单帧 ≈ 20.6 ms ⇒ **48 fps**。

**`LZ4` (bit5) 与 `zlib` (bit1)、`RLE` (bit0) 互斥** —— 一帧最多置一个压缩位，
多于一位是非法帧，接收方 NAK。载荷排布与 zlib 完全一致（含 `DUAL_FACE` 的
`[u32 comp_len_A][A][B]` 双流前缀），`DELTA|LZ4` = `lz4(prev ^ cur)`。

🔴 **必须是 LZ4 raw block，不是 `.lz4` 帧格式。** 这是最容易踩的坑：

| | 产生方 | 内容 | 板端能不能解 |
|---|---|---|---|
| **raw block** ✅ | `LZ4_compress_HC()` / `LZ4_compress_default()`（`lz4.h`/`lz4hc.h`） | 纯 token 流，无头无尾无校验，**不带原长** | `LZ4_decompress_safe()`，`dstCapacity` 由调用方给（单面 = `raw_len`，双面 = 各面 `nX*0x3000`） |
| `.lz4` 帧格式 ❌ | `lz4` 命令行 / `lz4frame.h` / python `lz4.frame` | 魔数 `0x184D2204` + 帧描述符 + 分块头 + 可选 xxhash | **不能** —— `LZ4_decompress_safe()` 会把魔数当 token 解，返回负数 |

所以发送端不能 `lz4 -9 f.bin`，也不能 `lz4.frame.compress()`；povstream 用 ctypes
直接调 `liblz4.so.1` 的 `LZ4_compress_HC()`（`--codec lz4`，级别 `--lz4-level`，默认 9）。

拆两条流的压缩代价（x86 实测，同一个 `anime_dual720.bin`）：
lz4-HC9 单流 388,166 B → 双流 388,307 B（+141 B，+0.04%）；zlib-6 单流 377,009 B →
双流 377,093 B（+84 B，+0.02%）。与上面 §"为什么拆两条"的结论一致。

**HC 级别（x86 liblz4 1.10.0，整帧单流实测）**：

| level | size | ratio | encode |
|---|---|---|---|
| HC9  | 388,166 | 22.79× | 135 ms |
| HC10 | 413,178 | 21.41× | 138 ms |
| HC11 | 381,889 | 23.17× | 497 ms |
| **HC12** | **370,699** | **23.87×** | 977 ms |

🔴 **HC10 比 HC9 还差 6.4%，可复现，跳过 10。** 默认用 **HC12**：比 HC9 小 4.5%，
甚至比 zlib-6（377,009 B）还小 —— 于是 LZ4 在这份内容上**压缩比和解压速度双赢**。
代价全在 PC 编码侧（≈1 s/帧）⇒ **必须走 `--dir` 的离线预压缩缓存**，
现渲直推（`stream --anim` 或 `--no-precomp`）会被编码封顶，povstream 会打警告。
解压速度与级别无关（raw block 格式一样）。

### 2026-08-04: MSTREAM (bit6) —— 流数可变，按**工作量**切而不是按**面**切

并行解码的 makespan 由**最慢那条流**决定。`DUAL_FACE|FOLD_A`（面A 折 180 +
面B 360，共 540 片）按面切时两条流是 1:2，**双核完全压不平**：

| 切法 | 两核 makespan | @30fps 解码余量 |
|---|---|---|
| 按面切 A180 / B360 | 20.6 ms（被面B 的 360 片封顶） | 1.53× ❌ |
| 朴素三分 180/180/180 | 20.6 ms（3 条流放 2 个核 = 2+1，零收益，还白亏 489 B） | 1.53× ❌ |
| **均衡三分 A180 / B0-89 / B90-359** | **15.47 ms**（两核各 270 片） | **2.04×** ✅ |

⇒ **切点必须落在面B 的第 90 片**。代价：本仓库 `anime_dual720` 派生的 fold540 帧
实测 lz4-HC12 按面两流 265,850 B → 均衡三流 266,330 B（**+480 B，+0.18%**）。

🔴 **顺带纠正一个旧说法**：`FOLD_A` 曾被说成"链路和解码两头都省"。**解码那半是错的** ——
按面切时 makespan 由面B 封顶，折不折叠完全一样，**`FOLD_A` 只省链路（−31%）**。
正因为如此才必须重新切分负载。

**载荷格式**（置 `MSTREAM` 时，取代 `DUAL_FACE` 那个 4 B 前缀）：

```
[u32 n_streams]
[n_streams × { u32 comp_len_i, u32 n_slices_i }]      # 8 B/条
[流 0][流 1] … [流 n-1]
接收方必须校验:  comp_len == 4 + 8*n_streams + Σ comp_len_i
                Σ n_slices_i == hdr.n_slices
流 i 解压后落在 buf + (Σ_{j<i} n_slices_j) * 0x3000, 长度 n_slices_i * 0x3000
n_streams ∈ [1, PVS_MAX_STREAMS = 16]
```

- **表里必须带 `n_slices_i`**：压缩流里读不出原长，而 `LZ4_decompress_safe` 要先知道
  `dstCapacity` 和落点。只给压缩长度的话，流 i 的落点得等流 i-1 解完才知道 =
  退化成串行，并行的意义就没了。
- **末条的长度不省那 4 字节**：388 KB 载荷里 4 字节 = 0.001%，换来两个独立的求和
  自校验 —— 载荷截断/错位当场 NAK，而不是解出半帧垃圾还照样 ACK。
- **边界必须落在片边界**（长度都是 `0x3000` 的整数倍）：板端是按片给两个核派活的。
- 流之间**不能有跨流引用**，各压各的。
- 与 `DUAL_FACE` **正交**：`DUAL_FACE` 说"显示怎么分"（两个 DDR 基址，分界 `nA*0x3000`），
  `MSTREAM` 说"解码怎么分"（几条流）。`MSTREAM` 也可用在单面帧上（360 片切两条 =
  单面也吃满双核）。

**板端派活必须是连续分组，不能轮转**：三条流 180/90/270 轮转会得到
核0={流0,流2}=450 片 / 核1={流1}=90 片，makespan 450 —— 比按面切还差；
连续分组才能取到 {流0,流1}=270 / {流2}=270。

**向后兼容（两个方向）**：

- **老固件 + 新发送端**：默认 `--stream-split face` 不置 `MSTREAM`，逐字节还是老排布。
  且当均衡切分结果**恰好等于按面切**时（720 片双面 = 360/360 本来就平衡），发送端
  自动退回老的两流格式。真要用三流时置了 `MSTREAM`，老固件的未知 flag 掩码会直接
  NAK —— **响亮地失败**，不会静默解错。
- **新固件 + 老发送端**：没有 `MSTREAM` 位就走老的两流分支，逐字节兼容。

发送端开关：`povstream.py stream --stream-split {face|balanced}`（默认 `face`）。

## Sender pipeline

```
source (spinpulse GLB anim | procedural globe | dir of .bin)
  → per-frame: transform points → voxelize → 360 slice renders
  → 1-bit Bayer dither (phase varies per slice AND per frame)
  → pack_obs.pack_slice → 360×0x3000 frame
  → zlib (默认) / lz4 (--codec lz4) → PVS1 frame → TCP, wait ACK, pace to --fps
```

Because live numpy rendering costs seconds/frame, the normal workflow is
`povstream.py render` (prerender N frames to `stream/pc/frames_<name>/`)
then `povstream.py stream --dir` at target fps. `stream --anim` live-renders.

`fake_board.py` is a loopback receiver implementing this spec (decompress,
verify, sha256, optional slice-0 PNG via `pack_obs.unpack_slice`) for
hardware-free end-to-end testing.
