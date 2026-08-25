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

## PL 硬件 lz4 解码器 (`--pl-lz4`, v3.5) —— 一刀砍掉 dec + cpy

⚠ **默认关着**，因为 PL 里的解码器还没集成进比特流。开着它而 PL 不在，
开机自检会当场把它关掉并说清原因（见下面）——**不会**带病上路。

半屏 3-bit 双面（282 槽 = 10.47 MB/帧）转动实测：

```
dec 158 ms + cpy 80 ms = 238 ms/帧  =>  4.2 fps      (转速给的上限是 22 fps)
```

`dr1v90/lz4hw` 的 `lz4_axi_top` 把这两项**一起**消掉：解码交给 PL；而且它
片上有 64 KB 历史窗（= LZ4 offset 上限），全程不回读 DDR ⇒ 输出是**纯顺序流**
⇒ `DST_ADDR` 可以直接写最终的帧 bank，**不需要先解到 staging 再 memcpy**。

```
老:  收包 -> staging(cached) -> CPU 解压 -> memcpy 8-21 MB 进 bank -> 翻页
新:  收包 -> comp 缓冲(DDR, 几百 KB) -> PL 引擎直写 bank -> 翻页
```

🔴 **谁往 bank 里写，两种模式不一样**（改这块代码前先看懂这条）：
`--pl-lz4` 关着时是 flip 线程 memcpy；开着时是 **RX 线程**（PL 直写，或回退到
CPU 时 RX 自己 memcpy），flip 线程只写寄存器。两种写者绝不混用。

### 用法

```bash
./pov_rxd --pl-lz4                       # 1 个引擎 @0x40020000
./pov_rxd --pl-lz4 --pl-engines 3        # 当前 BD 的配置 (NENG=3)
./pov_rxd --pl-lz4 --pl-engines 3 --no-pipeline   # 退回串行, 二分用
```

⚠ **流数应当正好等于引擎数，而且要等分**：3 引擎 = **95/95/94 片**。
`DST_ADDR = bank + Σ(n_slices_j × 0x9000)`，天然 8B 对齐。
流数多于引擎数时，多出来的流要等引擎空出来才发得出去，而流水线模式下那会儿
RX 正阻塞在 `recv` 里没人 poll ⇒ 它们要等到下一帧收完才起跑。启动时会告警一次。

🔴 发送端现有的 `--stream-split balanced` 是当年给**两个 CPU 核**调的
（180/90/270 不等分），对 3 个**同构硬件引擎**是错的 —— 需要 N 等分模式。
板端的动态派活（谁先空谁接下一条）本来就支持任意切法，收到什么按 `DST_ADDR`
分派即可。

BD 已定稿（bitstream 已建 / 时序收敛 / 160 向量过）：

```
AXI-Lite   lz4_0/1/2 @ 0x40020000 / 0x40030000 / 0x40040000  各 64K, 挂 GP0
           panel 仍在 0x40010000
AXI4 主口  每引擎独占一个 HP —— 顺序 HP3 → HP1 → HP2 (HP0 留给面板取数)
时钟       FCLK_CLK0 50 MHz, 与 panel 同域
```

`--pl-engines` 默认就是 **3**。写多了也不怕：自检**逐引擎判决**，不存在的那些
会被单独判死并降级运行，不会一票否决掉整条 PL 通路。

集成好之后要把它加进 `systemd/povrxd.service.d-*.conf` 的 `ExecStart`，
和 `--half-scan --oe-w` 一样，**否则服务重启就回到 CPU 路径**。

### 并行度 = 载荷里的**流数**

PL **拆不开单条 lz4 流**（LZ4 是串行回溯格式）。所以 N 个引擎要 N 条流：

```
povstream.py stream --codec lz4 --stream-split balanced   # MSTREAM 多流
```

流数 < 引擎数时多余的引擎就是闲着（启动日志会提醒）。这正是当初为**双核并行**
切多流白捡的好处：换成硬件引擎，编码侧一行都不用改。

### 🔴 收包与解码必须流水线（v3.5b，默认开）

板子**只有 WiFi，而且是物理必然** —— Zynq 跟着 LED 屏一起以 11.1 rev/s 旋转，
插不了网线（`eth0 carrier=0`，全部流量走 `wlx*`）。实测收一帧（~300 KB）
55–80 ms = 30–44 Mbps，就是这块 USB WiFi 网卡的真实能力，换环境不会变好。

```
串行   recv(55-80) + PL(75) = 130-155 ms  =>  6.5-7.7 fps
流水线 max(55-80, 75)       =  75-80 ms   =>  12.5-13 fps  => 撞上转速上限 11.1 ✅
```

花 3 个引擎把 `dec+cpy` 的 238 ms 干掉，再让串行 recv 吃回去 55–80 ms，不值。
所以默认**流水线**：给 PL 发完车立刻 ACK，回去收下一帧；下一帧收完再回来收割。

**ACK 语义因此从「已显示」变成「已交给硬件」。** 这是有意的裁定：内容是实时
动画，出错的唯一后果是丢一帧，而丢帧本来就是既有策略（newest frame wins）。

⚠ **错误报告因此晚一帧**。日志会把两个 `seq` 都打出来：

```
PL 解码失败 (STATUS[1]=error): 出错的是**帧 seq=17**(不是刚收到的 seq=18); 引擎0 流#1 ...
```

线上那个 NAK 字节对应的是**刚收到的那一帧**，真正解坏的是**上一帧**；
发送端收到 NAK 会重连 + 重发 keyframe，结果一样。`--no-pipeline` 退回串行
（ACK = 已显示），出问题时二分用。

🔴 **bank 归属的关键次序**：**先把上一帧发布出去，再认领本帧的 bank**。
反过来 RX 会同时占两块（上一帧待翻 + 本帧在写），加上 active 就是 3 块全占，
而 flip 线程正等翻页窗（最坏一整圈 90 ms）期间 active 还没换 ⇒ 认领到的必然是
**正在显示的那块** = 边扫边写。按「先发布再认领」走，RX 任何时刻只占一块；
上一帧若还没被翻走，`bank_claim` 就地顶替它（= 既有的最新帧赢策略）。
SIM 构建里有个 `BANKGUARD` 哨兵守着这条不变量，`test_local` 见到一次就判失败
（已用"故意改坏次序"验证过它真的会响，不是死代码）。

### comp 缓冲**不需要**双缓冲（算过了）

直觉上流水线要双缓冲，实际不用 —— 因为 recv 落的是 `cbuf`（cached malloc），
**不是** comp 缓冲；comp 缓冲只在 `pl_stage_streams` 那一刻被写，而下一帧的
stage 必然发生在本帧收割之后（引擎是同一套硬件，`PL(N)` 本来就不能在 `PL(N-1)`
完成前开始）。唯一需要「上一帧压缩流还在」的场合是出错时的 CPU 交叉验证，
而那时 `cbuf` 已经装着下一帧了 ⇒ **交叉验证的源改成 comp 缓冲里那份**
（WC 读慢，但只在出错时读一次）。于是映射窗一个字节都不用涨：

```
双缓冲 (2*2MB) 需要 map 0x5910000 = 89.06 MiB  >  povmem 88 MiB   ✗ 装不下
单缓冲 (1*2MB)      map 0x5710000 = 87.06 MiB  <  povmem 88 MiB   ✓ 余 960 KB
```

（`pov_boot.sh` 里那个手写的 `size=0x5800000` 因此**一个字都不用改** —— 那个
常量以前漏改过一次造成静默越界写，这次特意绕开它。）

### 缓冲怎么放（算式，别再算一遍）

```
bank A/B/C   0x10000000 / 0x12000000 / 0x14000000, 间距 0x2000000, 各用 0x1500000
banks 区     2*0x2000000 + 0x1500000 = 0x5500000  (85.0 MiB)
comp 缓冲    0x15500000 + 0x200000   (2 MB; 单帧压缩流实测 ~300 KB = 33x, 留 6-7 倍)
自检落点     0x15700000 + 0x10000    (64 KB, 4 个引擎各 16 KB)
映射窗       --pl-lz4 off: 0x5500000    on: 0x5710000 (87.1 MiB)
povmem 现值  size=0x5800000 (88 MiB)  =>  **两种都装得下, pov_boot.sh 不用改**
```

映射长度是按需求的最小值取的：关着时不多要那 2.06 MB，免得把 `povmem size`
刚好卡在老值上的板子推进 `mmap -EINVAL` → 静默回退 SO 的坑。

🔴 每条流在 comp 缓冲里的落点**对齐到 64 B**：`lz4_axi_top` 的读侧
`rd_ptr <= src_addr` 之后每拍取 8 字节，**假设 src_addr 8 字节对齐**；而 MSTREAM
载荷里各流是紧挨着排的（第 2 条起偏移是任意字节）。反正压缩流本来就要 memcpy
一次进 PL 看得见的 DDR，顺手对齐，这一整类风险就没了。
（`vivado/hdl/lz4/lz4_engine_axi.v` 的文件头把这条写成了硬契约：
「SRC_ADDR / DST_ADDR 必须 8 字节对齐，硬件不做兜底」。DST 天然满足，
因为落点是 `bank_base + Σn_slices*stride`，而 0x3000/0x9000 都是 8 的倍数。）

### 什么帧走 PL，什么帧回退 CPU（回退**永远有一行日志**）

| 帧 | 走哪 |
|---|---|
| `LZ4` 且不带 `DELTA`，`comp_len <= 2 MB` | **PL** |
| `zlib` / `RLE` / `raw` | CPU（PL 只认 lz4） |
| 任何 `DELTA` 帧 | CPU，且**整个进程**从此退回 CPU |
| 自检没过 / 引擎被判死 | CPU |

`DELTA` 为什么不行：XOR 要把上一帧原样读回来，而 PL 的输出在写合并内存里，
**WC 读极慢**（读 10 MB 比省下的解码时间还多）。所以 PL 帧一律作废参考帧；
第一次见到 DELTA 帧会 NAK 一次（此刻确实没有参考帧），povstream 收到 delta 的
NAK 会自动重连 + 重发 keyframe，之后一路 CPU ⇒ **全程只 NAK 一次**。
这个开关是**进程级**的：做成连接级就是 NAK/重连风暴（NAK 会关连接）。

**要 PL 的速度就别开发送端的 `--delta`；要 DELTA 就别开 `--pl-lz4`。**

### 🔴 引擎会**静默挂死**，而且 RTL 没有软复位

BD 交付时确认的失效模式：**一条流的长度不对**（源字节耗尽而 `raw_len` 还没到）
时，引擎 **`busy` 恒 1、不置 `error`、不置 `done`** —— 纯粹卡住。唯一的出路是
整个 PL 复位（画面闪一下），守护进程不该干这事。

⚠ **`MSTREAM` 的两个求和自校验管不到单条流的 `raw_len`** —— 校验过了不代表
喂进去是安全的。所以 `--pl-timeout` 是安全网里唯一的一层。

处理方式（`pl_reap_stuck`）：

| 情况 | 行为 |
|---|---|
| 一个引擎超时 | **摘出派发池，永不再派活**；剩下的继续跑 |
| 3 个挂 1 个 | 降到 0.67x 能力，掉帧但**不黑屏**（`PLDIAG` 的 `eng=2/3`） |
| 全部挂掉 | 永久关 PL，日志明说是「**引擎全部卡死**」而不是「解码出错」 |

不摘掉的话，下一帧派给同一个引擎又超时，会**一路退化成「每帧等满一个超时」**。

超时后的宽限等待是**有界**的：真卡死的引擎永远等不到 `busy` 掉下去，
所以老的「等到不 busy 为止」在这个失效模式下就是第二个死等点。宽限只为
「只是慢」（DDR 争用）那种情况留的，超过就判死。

**软件侧还挡了一层**：单条流若 `comp_len × 255 + 64 < n_slices × stride`，
它**绝无可能**解出 `dst_len`（LZ4 raw block 的放大上限是 255×）。这种流不交给
硬件，改走 CPU —— `LZ4_decompress_safe` 不会卡死，由它给出权威结论。
代价是几行判断，收益是不用赔一个永久报废的引擎。

🔴 **超时的计时起点是「我们真正阻塞了多久」，不是「自发车以来」。**
流水线下发完车就去 `recv` 了（WiFi 55–80 ms，抖起来更长），按「自发车以来」算
会在第一次 poll 就判超时 —— 而超时的后果是**永久摘掉一个引擎**，假阳性的代价
极高：链路抖一下就报废一个引擎。硬件真正跑了多久由 `PLDIAG` 的 `pl` 字段单独记。

### 🔴 `done` 的"新鲜度"必须在**发车那一刻**确认（2026-08-25 上板打脸）

`done_r` **只在 `start` 那一拍清 0**，没有写 1 清零口。所以「我读到的 `done`
是这一次的，还是上一次残留的」必须有办法分辨。

**曾经的做法（错的）**：留给调度器的第一次 poll —— 先看见 `busy=1` 或 `done=0`
这个**瞬态**，才开始采信 `done`。串行时没问题（发车后立刻 poll，瞬态就在眼前）。
**流水线之后就错了**：发完车 RX 去 `recv` 下一帧（55–80 ms），而引擎 74 ms 就
干完了 ⇒ 回来第一次 poll 时 `done=1, busy=0`，**瞬态早就不存在**，两个条件一个
都不成立 ⇒ 永远 `continue` ⇒ 每帧等满 400 ms 超时。

上板实测的形态（`--pl-engines 3`，284 片 3-bit 双面，`comp≈555 KB`）：

```
自检:  引擎0/1/2 全过, 0.93 B/clk          ← 硬件没问题
推流:  24 帧只成 1 帧, 其余全部 400 ms 超时
超时后 devmem 读寄存器: 三个引擎 done=1 err=0 busy=0 CYCLES≈3.69M (=73.9 ms)
唯一成功那帧: pl 74.1ms 0.95B/clk         ← 与 CYCLES 完全吻合
```

⇒ 引擎 74 ms 就干完了，阈值 400 ms，**是软件没认**。
（那唯一成功的一帧，是恰好 `recv` 够快、poll 落在 `busy=1` 窗口内。）

**现在的做法**：`pl_start()` 写完 `CTRL` 后**当场**读 `STATUS`，确认看到
`done=0`（或 `busy=1`）。`done_r` 在 start 后 1–2 个 aclk（50 MHz = 40 ns）就
清掉，而一次 AXI-Lite 读要 ~µs ⇒ 紧接着读必然成立。确认之后，后面任何时候读到
的 `done` 都必然是本次的，**poll 侧不再需要任何新鲜度判断**，那套逻辑连同它的
坑一起删掉了。

> **教训**：瞬态只在发车后的几微秒内保证存在，就必须在那几微秒内去看。
> 把"确认"推迟到一个**时序不受控**的地方（这里是 recv 之后），等于没确认。

**为什么 SIM 全绿却漏掉了**：SIM 的 `busy` 是按「还能读几次」建模的
（每读一次 `STATUS` 减一），于是**无论隔多久去读，头几次一定读到 `busy=1`**，
瞬态永远抓得到。真硬件的 `busy` 是一段**时间**。已改成时间模型
（`SIM_PL_BUSY_US`，200 µs）：发车时的确认读一定落在窗内，调度器 poll（ms 级）
一定落在窗外 —— 与板上同形。改完后 `test_local` **当场复现**了板上的失败
（17 次超时、`PLDIAG n=0`），修完降到 0。

**顺带加的定性日志**：超时时如果发现引擎其实全是完成态，直接打
「**轮询逻辑漏检 done**，不是引擎卡死也不是 DDR 争用」——
这次是靠人去 `devmem` 读寄存器才定性的，下次一眼就能看出来。

### 出错怎么响亮（本项目吃过静默失败的亏）

1. **开机自检** — 每个引擎各解一段本进程现压的 raw block，与 liblz4 逐字节比对。
   不过就把 `--pl-lz4` 自动关掉并打出该查什么（比特流里有没有？基址对不对？
   AXI4 主口接 HP 了吗？）。“PL 没接上”必须在开机时炸，不是推流中间。
   每个引擎单测 ⇒ “两个引擎其实是同一个”这种 BD 接线错误也查得出来。
2. **`STATUS[1]=error`** — 先原样打出 `err_code` + src/dst/len，然后**用 CPU 把
   同一条流再解一遍交叉验证**：CPU 解得出来 ⇒ **引擎有问题**，永久关掉 PL 并
   点名（本帧照常上屏，一帧不丢）；CPU 也解不出来 ⇒ 流真的坏了 ⇒ NAK。
3. **超时** — `done` 一直不来 = 硬件层面就不对，直接判死 + 本帧改用 CPU 解。

🔴 软件必须等 `STATUS[0]=done`，**不是 core_done**：core 报完时最后几个字节可能
还压在写通道里。而且 `done_r` 只在 `start` 那一拍清 0 ⇒ 刚写完 `CTRL` 就读
`STATUS`，读到的可能是**上一次**的 done ⇒ 代码先确认引擎“动起来了”
（`busy=1` 或 `done=0`）才开始采信 `done`。

### 日志

```
PL 自检: 引擎0 @0x40020000 OK (16384 B / 18432 cyc = 0.89 B/clk)
PLDIAG pl 52.1/60.0ms n=19 1.94B/clk out=21600KB eng=2 fb=0 ok=1
PL 回退 CPU: PL 只认 lz4 raw block, 本帧不是
```

`PLDIAG` 的 **B/clk** 就是 DESIGN.md 想要的实测值（输出字节 / `CYCLES`），
用它决定到底要几个引擎。BD 给了一把现成的尺子：**95 片一条流应该是
3.63–3.77M aclk = 0.93–0.97 B/clk**；低于 **0.80** 会自动告警一次，意思是
有小事务或 DDR 争用。⚠ 想用 `pair_miss` 佐证 DDR 争用的话，它的读数早就
**饱和在 65535** 且 RTL 没有软件清零口（只有 PL 复位能清）⇒ 必须**冷启动后测
增长率**，绝对值没有意义。`DIAG` 行的 `cpy` 在 PL 模式下应当是 **0.0ms** ——
那正是"staging→bank 那次 memcpy 没有了"的证据。`fb` 是本窗内回退 CPU 的帧数。

`PLDIAG` 的 **`wait`** 是流水线的体检指标：RX 线程**真正阻塞**在等 `done` 上的
时间。流水线奏效时它应该接近 **0**（PL 整个藏在 recv 后面）；它一旦逼近 `pl`，
说明 recv 比解码快、瓶颈换到解码这边了 —— 那时候才该去加引擎或提频。
`pipe=on/off` 标明跑的是哪种模式。

⚠ `--pl-fault error:N` / `hang:N` 只有 **x86 SIM 构建**的引擎模型认（板上给了
无效）；`test_local` 用它把上面三条回退路径在 x86 上真跑一遍。

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
