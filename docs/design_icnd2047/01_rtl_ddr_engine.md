# ICND2047 双沿列驱引擎 RTL 设计 (icnd2047_panel_pov)

[2026-07-13 20:36 CST] 设计文档 v1。目标:照此文档可直接编码,不留架构级悬案。

## 0. 目标与总账

| 项 | 现役 (板1 v5, ICND2049) | 新引擎 (ICND2047) |
|---|---|---|
| 屏 | P0.9375 COB 160×180 共阴 | 同厂共阳版 |
| 列驱 | ICND2049, 单沿, DCLK 25MHz = 25Mbps/lane | ICND2047, **双沿**, DCLK 25MHz = 50Mbps/lane |
| 链结构 | 9 lane × 12 级联 × 16ch = 1728 列位 | 不变 |
| 行移位窗 | 192 DCLK = 384 aclk = 7.68µs | **192 沿 = 96 DCLK = 192 aclk = 3.84µs** |
| 扫描 | 54 扫 | 不变 |
| 整屏刷新 | 实测 2.37kHz (overlap, oe_window=48 DCLK, 占空比 25%) | 目标 **~4.75kHz** (oe_window=96 沿, 占空比 ~49%) |
| 行驱 | ICND3019 共阴低侧 | 未知共阳高侧芯片 → 抽象子模块 row_drv |
| aclk | FCLK0 50MHz | 不变 50MHz |
| DDR slice 格式 | pack_obs/gen_chess_obs 打包 | **不变** (fb 布局逐 bit 兼容) |

行周期预算 (aclk=20ns):FETCH 1 + LOAD 1 + SHIFT 192 + LWAIT 0 + DISP 1 = **195 aclk = 3.90µs**;
54 行 = 10530 aclk = 210.6µs → **4.75kHz**。行推进死区完全藏进移位尾 (条件见 §2.5)。
对比:现役同 overhead 结构单沿 2.37kHz,移位速率 ×2 → 数字自洽。

芯片时序输入 (ICND2047 V2.0 datasheet, VDD=3.3/5V 同):fMAX=25MHz;twCLK≥20ns (即 25MHz 时占空比必须 ~50%);tSETUP1/tHOLD1 (SIN-CLK) ≥5ns;tSETUP2/tHOLD2 (LE-CLK) ≥5ns;twLE≥20ns;twOE≥40ns;CLK 最大上升/下降时间 500ns。LE 长度 = LE 为高期间 CLK **上升沿+下降沿总数**:3=普通锁存(行不变) / 4=换行锁存(行+1) / 5=首行锁存 / 11=WR_REG1 / 12=WR_REG2;LE 高但 0 边沿 = Reset (老版本;新版要求 Reset 的 LE 必须含 CLK 上升沿)。真值表确认 LE=H 期间 CLK 边沿数据**照常移位** → LE 可与数据尾部重叠 (与 2049 auto 引擎同套路)。

---

## 1. 双沿数据路径:方案对比与选型

### 1.1 三个候选

**方案 A:纯 fabric 同沿翻 (板2 旧法,icnd2047_panel_seq.v 的做法)**
50MHz 单域,`dclk_out <= ~dclk_out` 每拍翻,SDI 每拍换,同一 posedge 出。
DCLK 边沿与数据翻转在管脚处名义同时 → 对芯片 5ns setup/5ns hold **名义裕量 0ns**,
实际靠 IOB clk-to-out 离散 (±0.5ns) 和芯片内部延迟碰运气。板2 实证后果:需要
`col_shift` 寄存器现场调像素偏移 (PARAM[15:12]),相位是"调出来的"不是"设计出来的"。
9 lane + 74HC245 + 排线离散下不可复制。**否决**。

**方案 B:ODDR 全家桶 + 25MHz 逻辑域**
逻辑降到 25MHz,每拍备双 bit (D1/D2),9×SDI/LE/DCLK 全走 ODDR,DCLK 用 MMCM 90° 相移
时钟转发居中采样。输出时序最干净,但:
- 现设计 AXI/fb/ddr_slice_fetch/angle_tracker 全在 50MHz aclk → 引入 25MHz 域要么整
  IP 搬家 (m_axi 也 CDC,smc 加异步桥),要么 fb 写仲裁/控制/status 全套 CDC,工作量大;
- BD 要加 MMCM 新时钟 → module_ref + 时钟资产双坑;
- **奇数沿 LE 是硬伤**:3/5 沿命令 = 25MHz 域的 2.5 拍,LE 必须在半拍处翻 → LE 也要
  ODDR D1≠D2 逐半拍控制,LE 生成逻辑复杂度翻倍;
- 收益 (更高线速潜力) 当前用不到:25MHz DCLK 已是芯片 fMAX。

**方案 C (选型):50MHz 单域 + DCLK 经 ODDR 半周期延迟转发,数据/LE 走 SDR-ODDR**
- 全部逻辑留在 50MHz aclk:**1 aclk = 1 沿 = 1 bit**,零 CDC,LE 奇数沿原生支持;
- 数据/LE:每拍 posedge 更新,经 ODDR **D1=D2=值** 输出 (等效 SDR,但与 DCLK 共享完全
  相同的 ODDR→pad 路径,通道间/对时钟 skew 最小);
- DCLK:经 ODDR **D1=上一拍相位, D2=当前拍相位** (SAME_EDGE 模式) → DCLK 边沿被推迟
  半个 aclk = **10ns**,落在数据 bit 眼图正中;
- 采样裕量:setup = 10ns − skew,hold = 10ns − skew;同类 ODDR 输出 skew <1ns,屏内
  74HC245 同片同向传播差 ~1ns → **~8ns/8ns,芯片要求 5/5ns,裕量 1.6×**,双沿两个方向
  对称成立。

### 1.2 方案 C 的 ODDR 接线细目 (照此写 RTL)

```
时刻 (ns):   0        10        20        30        40
aclk:        ↑fabric   |         ↑          |         ↑
SDI[i] pad:  bit_k 稳定 ---------- bit_k+1 --------- bit_k+2   (ODDR D1=D2=sdi_r[i])
DCLK  pad:   ----------↑边沿------ ----------↓边沿----          (ODDR D1=dclk_d, D2=dclk_r)
LE    pad:   同 SDI 相位,posedge 更新
采样点:                ↑ 距数据翻转 ±10ns
```

```verilog
// 每 lane (9 个) + LE,共 10 个:
ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC"))
  u_oddr_sdi (.C(aclk), .CE(1'b1), .D1(sdi_r[i]), .D2(sdi_r[i]),
              .R(!aresetn), .S(1'b0), .Q(sdi_pad[i]));
// DCLK,1 个:dclk_r 在 SHIFT 期间每拍翻转 (=25MHz 方波),idle 保持 0
reg dclk_r, dclk_d;  always @(posedge aclk) dclk_d <= dclk_r;
ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), ...)
  u_oddr_dclk (.C(aclk), .D1(dclk_d), .D2(dclk_r), .Q(dclk_pad));
// → dclk_r 翻转的那拍,pad 上前半拍还是旧值、后半拍变新值 → 边沿恰在 +10ns
```

- OE 非 bit 级时序信号 (窗口 ±1 拍无所谓),普通 IOB FF 即可;为管脚延迟一致性也可走
  D1=D2 ODDR,任选,建议统一走 ODDR 省心。
- XDC:全部输出 `set_property IOB TRUE`(ODDR 天然在 IOB);无需 output delay 约束闭环
  (板级异步接口),但建议写 `set_output_delay -max/-min` 报告项监视 skew。
- 占空比:dclk_r 严格每拍翻 → pad 上 20ns/20ns,满足 twCLK≥20ns。
- **降速逃生门** (`ddr_slow`,见 §4):dclk_r 每 2 拍翻 (DCLK 12.5MHz),数据每 2 拍换
  → 25Mbps,等效现役速率,SI 出问题时现场降级不重编译。

### 1.3 结论

**选方案 C**。理由压缩:零 CDC、LE 奇数沿原生、10ns 对称眼图 (1.6× 裕量)、-2 片子
50MHz fabric 时序毫无压力、ODDR 资源 11 个 (9 SDI + LE + DCLK[+OE])。方案 B 保留为
"未来 DCLK 超 25MHz 芯片换代"时的升级路径,方案 A 禁止再用。

---

## 2. 移位 FSM 逐状态设计

### 2.1 计数单位定义 (全文档统一)

- **1 aclk (20ns) = 1 半 DCLK 周期 = 1 沿 = 1 bit**。以下 "沿" 与 "拍" 同义。
- 行数据 = 192 bit = 192 沿 = 96 DCLK 周期 = 3.84µs。
- 主计数器 `sh_cnt[7:0]`:0→191,SHIFT 态内每拍 +1。
- 派生索引 (纯组合,保持与 v5 fb 布局逐 bit 兼容):
  - `word_idx = sh_cnt[7:4]` (0..11,当前 16-bit 词)
  - `pair = word_idx[3:1]` (0..5,fb 地址低 3 位)
  - `half = word_idx[0]` (0=低半 [15:0] 先发,1=高半 [31:16])
  - `bit_sel = 4'd15 - sh_cnt[3:0]` (词内 MSB first)
  - `sdi_r[i] = half ? pair_reg[i][16+bit_sel] : pair_reg[i][bit_sel]`
  - 上线序 = word0[15..0], word1[15..0], … word11[15..0] — **与 v5 auto/fb 完全一致,
    pack_obs / gen_chess_obs / DDR slice 格式零改动**。

### 2.2 fb 流水预取 (消灭 v5 的逐词注入气泡)

v5 auto 引擎每词走 AU_FBRD→AU_FILL2 注入,每词 ≥1 拍气泡 (12 词 ≈ +12~24 拍)。新引擎
改连续流:
- `pair_reg[0..8][31:0]`:9 lane 并行持有当前 pair;
- `sh_cnt[4:0]==29` 时发 `fb_raddr = {shift_row[5:0], pair+1}` (BRAM 同步读 1 拍延迟);
- `sh_cnt[4:0]==31` 时 `pair_reg[i] <= fb_dout[i]` 装载下一 pair;
- 行首 pair0 由 EG_FETCH/EG_LOAD 两拍序幕装载 (每行固定 2 拍开销,不做跨行预取,
  换取实现简单;192+2 → 帧率损失 1%)。

### 2.3 主引擎状态表 (EG_*,替换 v5 的 AU_*)

计满 54 行为一屏;`shift_row` = 正在移位的行,`disp_row` = 隐含为 shift_row−1 (2047
latch1/reg2 双缓存持有,RTL 不需存)。

| 状态 | 停留 (拍) | DCLK | SDI | LE | OE | 动作 / 转移条件 |
|---|---|---|---|---|---|---|
| EG_IDLE | — | 0 | 0 | 0 | 1 | auto_en & !busy(手动) & row_drv 就绪 → EG_FETCH;`shift_row<=0`,`primed<=0` |
| EG_FETCH | 1 | 0 | 0 | 0 | 保持 | 发 `fb_raddr={shift_row,3'd0}` |
| EG_LOAD | 1 | 0 | 0 | 0 | 保持 | `pair_reg[i]<=fb_dout[i]`;`sh_cnt<=0` → EG_SHIFT |
| EG_SHIFT | 192 | dclk_r 每拍翻 | 每拍出 1 bit (§2.1) | 尾部 le_len 拍 =1 (§2.4) | 保持 (窗口进程管) | 29/31 预取重装 (§2.2);`sh_cnt==191` → EG_LWAIT。**本态结束时 latch1 = shift_row 数据** |
| EG_LWAIT | 0..n | 0 | 0 | 0 | 保持 | 等 `oe_done && adv_fired && !row_busy` (上一显示窗收完 + 行选已推进到 shift_row 并稳定) → EG_DISP。稳态 0 拍 (条件早已满足) |
| EG_DISP | 1 | 0 | 0 | 0 | **1→0** | OE↓:reg2←latch1,开始显示 shift_row;`oe_cnt<=oe_window`,`oe_done<=0`,`adv_fired<=0`;`shift_row<=(shift_row==row_max)?0:+1`;`primed<=1` → EG_FETCH (立刻移下一行) |

停 auto (`auto_en=0`):任意态收尾当前行后回 EG_IDLE,OE 拉 1,dclk_r 清 0。

**两个独立并行进程** (与状态机解耦,照抄 v4.1 结构):

P1 — OE 窗口计数 (单位:拍=沿):
```
if (!oe_done) begin
  if (oe_cnt==0) begin oe_out<=1; oe_done<=1; end
  else oe_cnt <= oe_cnt-1;
end
```
oe_window 上限箝位 **187** (硬编码,原因见 §2.5),下限箝位 2 (twOE≥40ns)。

P2 — 行推进藏尾 (v4.1 的 adv_fired 逻辑移植):
```
if (auto_en && !adv_fired && oe_done && !row_busy && 引擎在 EG_FETCH/LOAD/SHIFT) begin
  row_first <= (shift_row == 0);   // shift_row 已 +1,==0 表示本次推进回首行
  row_go    <= 1;                  // 1 拍脉冲给 row_drv
  adv_fired <= 1;
end
```
即:OE 回高后立即在移位窗剩余时间里并行推进行选。行选推进期间 OE 恒为 1 (消隐) —
由 `oe_done` 前置条件保证。

### 2.4 LE 沿数生成 (3/4/5 在双沿域)

- LE 与数据尾部**重叠** (datasheet 真值表:LE=H 时数据照常移位;2049 同套路已验证):
  `le_len = (shift_row==0) ? 5 : 4;  le_r <= (sh_cnt >= 192 - le_len);`
  即 LE 高覆盖 EG_SHIFT 最后 le_len 拍 = 恰好 le_len 个 DCLK 沿。
- 首行 5 / 换行 4;**3 (普通锁存) 本引擎 1-bit 单锁存/行用不到**,保留给未来 BCM 多
  plane (同行多次锁存:首次 4/5,后续 3) 和手动路径。
- 沿相位核对:第 k 个沿 (k=1..192) 奇数为上升。le_len=4 覆盖沿 189..192 (R,F,R,F 中的
  R 起) → 含 2 个上升沿;le_len=5 覆盖 188..192 → 含 ≥2 上升沿;满足"新版 Reset 要求
  LE 含上升沿"从而永不落入 0 沿 Reset 歧义。
- LE-CLK setup/hold:LE 在 posedge 翻,最近 DCLK 沿在 +10ns → tSETUP2/tHOLD2 = 10ns ≥ 5ns ✓。
- twLE:le_len≥3 → LE 高 ≥60ns ≥ 20ns ✓。
- **RTL 级断言规则:le_r=1 仅允许出现在 EG_SHIFT 内** (即 LE 高期间必有 DCLK 边沿),
  杜绝 0 沿 LE 触发老版本芯片 Reset。手动 marker_LE 路径同样强制带沿 (§3)。

### 2.5 overlap 时序全景 + oe_window≤187 的由来

```
拍:      0    1    2         2+oe_window            194   195(=下一行的0)
         |FETCH|LOAD|SHIFT row N+1 (192拍) ......LE▄▄|LWAIT|DISP↓|FETCH...
OE:      ↓(显示 row N)————————————↑(oe_done)              ↓(显示 row N+1)
行选:                  ……row N …… |←推进 row N+1 (row_busy)→| 稳定
latch1:  row N 数据 ——————————————————————→ (LE 尾) row N+1 数据
reg2:    row N 数据 (OE↓ 时转移) ——————————————————————→ row N+1
```

- **oe_window ≤ 187 硬箝位**:LE 从 sh_cnt=187 (le=5 时) 开始。若 OE 在 LE 期间仍为低,
  且 2047 的 latch1→reg2 转移语义是 "LE↓ 即转" 而非 "OE↓ 才转" (datasheet 刷新率原理
  一节示意为 OE↓ 转移,与 2049 实测一致,但未拿硬件验证 2047),显示中的 row N 会被
  row N+1 提前顶替 → 撕裂。箝位 187 后 LE 必然落在 OE 回高之后,**两种转移语义等价**,
  风险直接消除 (见 §6-R2)。
- **行推进藏尾条件**:推进耗时 T_adv ≤ (192+3) − oe_window。按 3019 同款预算
  T_adv=80 拍 (8+64+8) → oe_window ≤ 115;取默认 **96** (占空比 96/195=49%) 留 19 拍余量。
- 亮度/帧率档位表 (54 扫,T_adv=80 假设):

| oe_window (沿) | 占空比 | LWAIT 拖尾 | 行周期 (拍) | 整屏刷新 |
|---|---|---|---|---|
| 48 | 24.6% | 0 | 195 | 4.75kHz |
| **96 (默认)** | **49.2%** | 0 | 195 | **4.75kHz** |
| 115 | 59% | 0 (临界) | 195 | 4.75kHz |
| 160 | 71% | 45 | 240 | 3.86kHz |
| 187 (上限) | 77% | 72 | 267 | 3.47kHz |

### 2.6 row_drv 行驱抽象子模块 (行芯片未知,接口先钉死)

```verilog
module row_drv (
  input  clk, rst_n,
  input  row_go,        // 1 拍:推进到下一行
  input  row_first,     // 本次推进回到首行 (链头灌 '1' / 复位译码)
  input  [31:0] cfg,    // 0x1C 运行时时序参数 (§4)
  output row_busy,      // 推进中 (含末尾 setup 死区);busy 期间引擎保证 OE=1
  // 物理线:按"行选串行链 + 锁存/消隐"抽象,共阳高侧
  output row_sdi, row_dclk, row_lck,  // 串行链数据/时钟/锁存 (LCK)
  output row_bk                       // 高侧总消隐 (BK),极性 cfg 可翻
);
```
- v0 实现 = ICND3019 时序克隆 (ADV_PRE 8 / ADV_HIGH 64 / ADV_HOLD 8 拍,cfg 可调),
  三线映射 row_sdi/row_dclk/row_lck ↔ 老 icnd_sdi/icnd_dclk/icnd_rclk,row_bk 悬空;
- 新行驱 datasheet 到手后只改 row_drv 内部,**顶层端口/引擎握手不动**;
- 若新屏行选是并行译码 (非串行链):row_drv 内部改成地址计数器 + 译码输出,复用
  row_sdi/row_dclk/row_lck 三线当 ADDR[2:0] 用或 BD 层换 XDC,接口仍不变;
- cfg 时序参数运行时可写 (§4 0x1C),换芯片不重编译。

### 2.7 手动命令路径 (保留,按双沿重写)

xsdb 逐词灌图 / WR_REG1/REG2 配置必需。语义沿用 v5 0x00/0x04,内部改:
- word 模式:16 bit = 16 拍 = 8 DCLK 周期;`le_count` 单位改 **沿**,LE 覆盖最后
  le_count 拍 (le_count≤16;寄存器写用 11/12);
- 级联寄存器配置套路不变:BURST=10 (11 词无 LE) + 末词 le_count=11 或 12,12 颗芯片
  同时锁存;
- marker_LE 模式重定义:LE 高 + **N 个沿** (DCLK 照跑,SDI=0) — 注意会移入 N 个 0,
  仅调试用;禁止无沿 LE (Reset 保护)。

---

## 3. 与 icnd2049_panel_pov.v 的 diff 清单

**顶层改名:`icnd2049_panel_pov` → `icnd2047_panel_pov`,文件 icnd2047_panel_pov.v。**
端口变化 (spin_sync 之外新增/改名任何一根线都算) → 按 BD module_ref 缓存坑规矩
(feedback_vivado_bd_module_ref_update / feedback_vivado_bd_addr_width_cache):BD 里
**删旧 module_ref 单元重加新名**,不做 update_module_reference 原地刷新;xci/wrapper
重生成。端口实际变化:`dclk_out/le_out/sdi_out` 语义同名保留;`icnd_sdi_out/
icnd_dclk_out/icnd_rclk_out` → 改名 `row_sdi/row_dclk/row_lck` + 新增 `row_bk`
(共 +1 根输出) — 必须改模块名的直接原因。

| 模块/块 | 处置 | 说明 |
|---|---|---|
| angle_tracker | **原样保留** (含两 bug 修复版) | 一字不改 |
| ddr_slice_fetch | **原样保留** | fb 布局 9×512×32b `{row[5:0],pair[2:0]}` 逐 bit 不变;DDR slice 格式、pack_obs/gen_chess_obs 打包器、slice_base/n_slices 语义全部不变 |
| fb BRAM ×9 + 写仲裁 (pov_en 选 df/AXI) | 原样保留 | 读口从 auto 引擎移交给 EG 引擎,读地址生成改 §2.2 |
| AXI-Lite 写/读 FSM 骨架 | 保留,解码扩展 | awaddr[5:2] 4bit 解码已够到 0x3C;新增 0x1C/0x20 (§4) |
| 0x00 CMD / 0x04 BURST 手动 sequencer | **重写** | 双沿:16 bit=16 拍;le_count 单位改沿;marker_LE 重定义 (§2.7);门控 divider (div_count/HALF) 整块删除 |
| dclk_fast / HALF 分频 | **删除** | 双沿固定 1 拍 1 沿;0x0C bit29 改义为 ddr_slow (§4,老脚本注意) |
| AU_* auto 引擎 (AU_IDLE..AU_FBRD) | **重写为 EG_*** | 流式 192 拍 + pair 预取,逐词注入路径删除 (§2.2/2.3) |
| oe_cnt/oe_done/adv_fired 并行进程 | 结构保留,单位改 | oe_cnt 单位 DCLK→沿 (拍),装载不再 ×2/×4;oe_window 箝位 [2,187] |
| ICND3019 FSM | **搬入 row_drv 子模块** | v0 内部时序照抄,外部改 row_go/row_first/row_busy 握手 (原 au_icnd_go/au_icnd_sdi/icnd_busy 改名);手动 0x08 直控路径保留接到 row_drv |
| 输出级 | **新增 ODDR 层** | 9×SDI/LE (D1=D2) + DCLK (D1=dclk_d,D2=dclk_r) + OE;dclk_will_fall 机制删除 |
| locked_ever/slice_max 诊断锁存 | 原样保留 | |
| POV 取帧触发 (df_go/df_last_slice) | 原样保留 | |

工程接线注意:X_INTERFACE_PARAMETER 的 ASSOCIATED_BUSIF 注释照搬;新增 row_bk 走 XDC
分配 (管脚待新屏接口板确认);spin_sync 仍走 panel_spi_miso workaround 口
(feedback_sensor_const0)。

---

## 4. 寄存器映射增量 (基址 0x40010000,在 0x00~0x18 之上)

原样兼容:0x00 CMD (le_count 单位变沿,见下)、0x04 BURST、0x08 行驱手动、0x10/0x14/0x18
POV 三件套、fb 窗 awaddr[15]=1、R 0x00/0x10/0x14/0x18。

| 地址 | R/W | 定义 | 变化 |
|---|---|---|---|
| 0x00 | W | [15:0] data, [22:16] le_count (**单位:沿**,寄存器写填 11/12), [25:24] mode | 语义微变:le_count 按沿计 |
| 0x0C sub10 | W | [0]=oe_val, [24:16]=扫描行数 (新屏默认 54), [27]=cfg_we, [28]=overlap_en, **[29]=ddr_slow** (0=25M DCLK 50Mbps / 1=12.5M DCLK 25Mbps 降级), [15:8]=oe_window (**单位:沿**,箝位 [2,187],默认 96) | bit29 改义 (原 dclk_fast);oe_window 单位改沿 |
| **0x1C** | W | ROW_DRV_CFG:[7:0]=adv_high 拍数 (默认 64), [15:8]=pre/hold 拍数 (默认 8), [16]=row_bk 极性, [17]=row_dclk 极性, [18]=row_lck 极性, [31]=cfg_we | 新增;行驱换芯片不重编译 |
| **0x1C** | R | {eg_state[2:0], 2'b0, shift_row[8:0], frame_count[15:0]} | 新增;引擎在线诊断 |
| **0x20** | R | frame_period:最近一整屏 (54 行) 的 aclk 计数 → 刷新率 = 50e6/该值,期望 ~10530 (4.75kHz) | 新增;免示波器测刷新率 |
| R 0x00 | R | 原 status + [11]=oe_done [12]=adv_fired [13]=row_busy | 位扩展,原有位不动 |

脚本兼容警示:旧 0x0C 写脚本若带 bit29=1 (原 dclk_fast=25M) 现在会进 ddr_slow 半速 —
迁移时全量检查 xsdb 脚本。

---

## 5. 仿真计划 (tb_icnd2047_panel_pov.v)

芯片行为模型 `icnd2047_bfm.v` (12 级联 ×9 lane 例化 1 lane 精查 + 8 lane 抽查):
双沿采样 SR (192b) + LE 沿计数器 + 命令译码 + latch1/reg2 + OE↓ 转移 + 内部行计数;
带 5ns setup/hold 违例检查 (对 pad 波形,含 ODDR unisim 仿真模型,glbl 初始化)。

| # | 验项 | 通过判据 (具体数字) |
|---|---|---|
| 1 | 双沿采样对齐 | 棋盘 slice (gen_chess_obs 同源金样) 灌 fb → BFM reg2 内容与金样 192b/lane 逐 bit 相等;BFM 报 0 次 setup/hold (<5ns) 违例;实测 pad 上数据-DCLK 沿距 = 10ns±仿真δ |
| 2 | LE 沿计数 | 每行 LE 覆盖沿数:row0=5,row1..53=4;BFM 译码序列 = 5,4,4,…,4 循环;LE 高期间 DCLK 沿数≥1 断言 (0 沿 Reset 保护);LE 重叠尾 bit 仍正确移入 (金样含尾 5 bit 特征图案);WR_REG1/2 手动路径:11/12 沿译码正确、12 颗同时锁存 |
| 3 | overlap | OE 低宽 = oe_window 拍整 (96→1920ns);OE 低期间 reg2=row N 同时 SR 正在收 row N+1 (BFM 双缓存内容断言);oe_window=187 上箝位、=1 写入被箝到 2;行周期 =195 拍、帧周期 =10530±5 拍 → tb 打印 kHz 并断言 ≥4.7kHz;oe_window=160 时 LWAIT=45、行周期 240 |
| 4 | 行切换消隐 | 断言:row_busy=1 ⇒ oe_out=1 (推进永远在消隐下);row_first 仅在 shift_row 回 0 那次推进为 1,且同行 le_len=5;T_adv=80 拍时零 LWAIT (藏尾成功),人为把 0x1C adv_high 调到 200 → LWAIT 出现且无功能错 |
| 5 | POV 路径回归 | 复用 v5 tb 的 AXI slave DDR 模型 + fake_en:slice 步进 → df 取帧 → 屏上数据切 slice;fb 写仲裁 pov_en 下 AXI 写被屏蔽 |
| 6 | 手动/auto 互斥与恢复 | auto 中发手动 CMD 不穿插 (busy 保护);auto_en 0→1→0→1 重入;aresetn 中途拉低恢复,OE 回 1、DCLK 无毛刺沿 |
| 7 | idle 静默 | 非 SHIFT 态 pad DCLK 零边沿断言 (2049 教训:空闲沿会移坏数据,2047 同门控策略);ddr_slow=1 全量重跑 #1~#4 |

跑法:xsim batch (reference_vivado_batch_tcl 流程),金样由 python 参考模型
(pack_obs 同源) 生成 memh。

---

## 6. 风险清单

| # | 风险 | 概率/影响 | 缓解 |
|---|---|---|---|
| R1 | LE 重叠尾部时数据是否照移:真值表说移,若实际芯片 LE=H 停移,尾 4~5 bit 整体偏移 | 低/中 | 上板首测用手动路径灌"末 5 bit 特征词"验证;若停移 → LE 改后置 + 数据预偏 5 bit (RTL 留 le_overlap 参数开关) |
| R2 | latch1→reg2 转移时刻 (OE↓ vs LE↓) 未在 2047 硬件验证 | 中/高(撕裂) | **设计上已消除**:oe_window≤187 硬箝位使 LE 必落在 OE 回高后,两语义等价 (§2.5);上板仍做 A/B 双行单步观察确认 |
| R3 | 行驱芯片未知:协议/极性/速度与 3019 克隆不符;T_adv>115 拍则藏不住,帧率掉 (200 拍→~4.0kHz) | 高/中 | row_drv 抽象 + 0x1C 运行时参数;datasheet 到手前不焊死任何时序;共阳高侧驱动能力(上升慢)预留 pre/hold 可调 |
| R4 | 50Mbps SI:每 20ns 一 bit 过 74HC245 + 排线,振铃/通道 skew 吃掉 10ns 裕量;CLK 边沿要求 tr/tf<500ns 易满足但数据眼图可能闭 | 中/高 | ddr_slow 一键降 25Mbps (等现役速率,2.4kHz 保底);示波器看 DCLK-数据相位;必要时 XDC 调 SLEW/DRIVE |
| R5 | 0 沿 LE = Reset (老版芯片):任何脚本/状态机漏洞发无沿 LE → 芯片寄存器复位,现象为亮度/消影配置丢 | 中/高 | RTL 断言 + marker_LE 强制带沿;手动脚本 code review;现象排查表加一条"疑似被 Reset → 重写 REG1/2" |
| R6 | 首行偏暗/低灰:默认 REG1/REG2 不适配,需要写 11/12 沿配置且 12 级联广播时序对 | 中/低 | 手动路径已支持 (§2.7);沿用 2049 级联配置套路 (BURST 10 + 末词 LE) |
| R7 | 共阳翻案:数据极性反 (共阳低有效?) — 2047 仍是灌电流 (低侧 sink,OUT 接 LED 负极),屏改共阳指行侧;但若厂家把数据也反相 | 低/低 | 1-bit 图案一眼可辨;fb 出口留 data_inv 调试位可加 (未进寄存器表,编译期参数) |
| R8 | BD module_ref 缓存坑:改名后 xci/wrapper 残留旧端口 | 高(踩过)/低 | 按既有 SOP:删单元重加 + 删 impl 缓存;refresh_bit.sh 确认读新 xsa (feedback_refresh_bit_stale_xsa) |
| R9 | 老脚本 bit29 语义反转 (dclk_fast→ddr_slow) 误降速 | 中/低 | 迁移 checklist;R 0x20 frame_period 一读便知实际刷新率 |
| R10 | ODDR 复位毛刺:SRTYPE=SYNC 下 aresetn 释放瞬间 D1/D2 不一致可能出窄沿 | 低/中 | dclk_r/dclk_d 复位同值 0;释放后先经 EG_IDLE (DCLK 恒 0) 再启动;tb #7 覆盖 |

---

## 附:上板 bring-up 顺序建议 (对齐仿真验项)

1. 手动路径:单词灌 + LE=4 锁存 + OE 手动开 → 单行点亮 (验 R1/R4/R7);
2. R2 验证:A/B 两行交替单步,看切换时刻;
3. auto 1 行循环 (row_max=0) → 54 行棋盘 (pack_obs 老图直接用);
4. 读 0x20 确认 ~10530 → 4.75kHz;示波器复核 OE 占空比;
5. POV 三件套照 v5 恢复序列回归。
