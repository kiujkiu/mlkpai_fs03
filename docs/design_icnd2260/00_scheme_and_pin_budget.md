# ICND2260 方案定案与 IO 预算 (2026-08-17)

上游: `zynq_pov/docs/claude_memory/reference_icnd2260_spec.md` (芯片规格 + 08-17 级联更正)
本文范围: 屏侧驱动方案选型、FPGA 引脚预算、四道验证闸门、SSN 实算任务书。

## 0. 口径 (2026-08-04 定案, 未变)

| 项 | 值 |
|---|---|
| 面板 | 192×216 px = 41,472 px = 124,416 LED / 面 |
| 双面 | 偏心背靠背, 两面数据量相同 (`fold-a` 不省面板带宽) |
| 转速 | 900 RPM = 15 rps (用户 08-05 确认, 再高振动过大) |
| 片/圈/面 | 603 (= 2π×96px, 由最外圈像素间距定) |
| 片率 | **9,045 片/s/面 ⇒ 110.6 µs/片** |
| 体帧率 | 30 Hz (= 2×转速) |

## 1. 芯片账

单颗 ICND2260 = 120 恒流通道 (40 px) + 48 内置行管, BGA225。

- 版图: 192 方向 48 扫 → 4 组; 216 方向 40 px 槽 → 6 颗 (用 36/40)
- **24 颗/面, 双面 48 颗**; 行驱 **0 颗** (48 NMOS 内置, ICND3019/1028 整条链消失)
- 像素槽利用率 90% (216/40 = 5.4 ⇒ 6 颗). ⚠ 待核: 换 44 扫 × 5 颗排法可到 94%,
  单颗数据量 92,160 → 84,480 bit, 有可能把「2 颗/链」的上限从 315 顶到 360 片/圈。

## 2. 链路账 (级联是主设计旋钮)

2260 自带 mLVDS 输出 (L_D*OP/ON + L_CKOP/ON), `VHEAD[19:16]` 级联最多 16 颗,
链内共享同一条链路带宽 ⇒ **FPGA 只驱动链头**。

```
链路能力 = 3 对 × 333 Mbps = 1,000 Mbps   (166.7MHz 双沿, 按 tLVCP 保守取)
单颗每片 = 40px × 3 × 16bit × 48扫 = 92,160 bit   ← pixel#0..#39 固定, 不可截断
单颗需求 = 1.3824 × N(片/圈) Mbps  @900RPM
```

| 颗/链 | 最大 片/圈 (留 15% 余量) | 链数 (双面48颗) | FPGA 差分对 | 总 PL IO |
|---|---|---|---|---|
| **1** | **629** (603 口径 ✅ 余量 20%) | 48 | 192 | 439 ❌ |
| 2 | 315 | 24 | 96 / 时钟外扇出 73 | 223 ❌ / 177 ✅ |
| 3 | 210 | 16 | 64 | 151 ✅ |
| 4 | 157 | 12 | 48 | 115 ✅ |

⚠ 360 片/圈挂 2 颗 = 995 Mbps, **余量 0.5%**, 与当年 0.5° 口径同一个坑, 不算数。

7020 CLG484 / DR1V90 都是 **96 对 + 8 单端 = 200 PL IO** ⇒ 603 双面直出差分不成立。
换 FPGA 也救不了: 7z045 FFG900 只有 **181 对 < 192**, 要 K7-325T FFG900 (250 对) 那一档。

## 3. ✅ 定案: FPGA 出单端 → 外置 LVDS 驱动器 → 差分进 2260

保「603 片/圈 + 双面全速」的唯一路。

| 项 | 根数 | 说明 |
|---|---|---|
| 数据单端 | **144** | 48 颗 × 3 通道, 每根 333 Mbps = 166.7 MHz DDR |
| 时钟单端 | **8** | 每 6 颗驱动器一根, 板上再短距扇出 (不要一根扇 48 个负载) |
| 控制 | 7 | I_SYNC / RSTN / SCL / SDA / 光电 / UART×2 |
| ACK 回传 | 6 | 48 路经 8:1 mux; 首期不做开短路回读则为 0 |
| **合计** | **165 / 200** | ✅ 余 35 |

**时钟取 8 根的理由 (2026-08-17 定)**: 8 根整除得干净, 四个 bank 完全对称 ——
```
48 颗驱动器 ÷ 4 bank = 每 bank 12 颗
每 bank 2 个时钟组, 每组 6 颗驱动器 = 18 数据 + 1 时钟 = 19 脚
每 bank = 2 组 = 38 脚          ⇒ 38/38/38/38
```
(原方案 6 根时, 6×8颗 与「每 bank 12 颗」不整除, 时钟组会跨 bank, 白白恶化关 3 的 skew 预算。)
⇒ 定案引脚表 `ssn_2260/xdc/RECOMMENDED_pinout_152.xdc`, **按驱动器分组编排**:
同一颗驱动器的 3 数据 + 其组时钟排在一起, 画板直接照着连。

- 驱动器: **48 颗四通道**, 一颗管一个 2260 的「3 数据 + 1 时钟」。
  🔴 **别把时钟单独放一颗** —— 跨器件 skew 会吃掉 1.5 ns 的 tSTU/tHLD 预算;
  同片内通道间 skew 通常 <200 ps。
- 附带好处: FPGA bank 保持 LVCMOS, 不用为 LVDS_25 改 2.5V。
  ~~甚至降 1.8V 减 SSO~~ 🔴 **2026-08-17 实测推翻**: LVCMOS18 的 SSN 余量**比 LVCMOS33 差**
  (同 DRIVE 4: 46.5% vs 66.4%) —— 噪声预算按 VCCO 绝对值缩放。降压只在省功耗上有意义。
- ✅ **定案电气配置: LVCMOS33 / DRIVE 4 / SLEW FAST** (见 §4 关 1)。
- FPGA 侧速率不是问题: DS187 Table 48 OSERDES DDR 950 Mb/s(-1) ⇒ 333 Mbps 余量 2.8×;
  负载只有驱动器输入 ~5 pF + 板上 2-3 cm 短线。

## 4. 四道闸门 (按此顺序过)

### 关 1 ✅ SSN —— **2026-08-17 已过, 不是瓶颈**

150 根单端摊到 4 个 HR bank (38/38/37/37), `report_ssn` **6 组配置全部
`Full Analysis;Passed`, `Ports Exceeding SNN Margin: 0/150`**:

| IOSTANDARD | DRIVE | SLEW | 最差余量 |
|---|---|---|---|
| LVCMOS33 | 4 | SLOW | +66.4% |
| **LVCMOS33** | **4** | **FAST** | **+66.4%** ← 定案 (FAST 是白拿的边沿) |
| LVCMOS33 | 8 | FAST | +30.9% ← 安全退路 |
| LVCMOS18 | 4 | SLOW/FAST | +46.5% |
| LVCMOS18 | 8 | FAST | +37.7% |

天花板扫描: **每 bank 塞满 49 根 (共 196 根) 在 DRIVE 4 下仍 Passed (+56.6%)**
⇒ DRIVE ≤8 时 SSN 从不构成限制, 真限制只是 CLG484 物理上 200 根 PL IO。
⇒ **163/200 预算站得住, 不用退到「2 颗/链 + 315 片/圈」。**
工业级温度补跑过, 余量与商业级完全相同。

🔴 **但这个"过"要正确解读 —— 7 系列 report_ssn 与数据率无关。**
把 MMCM 输出从 166.667 MHz 改成 25 MHz 重跑, 两份报告**逐字节相同**(已复核);
`-phase` 也报 `Average noise reduction due to phase distribution: 0%`。
它给的是 UG471 的**静态 SSO 加权预算**(标准×驱动×摆率×终端×引脚位置), 不是 di/dt 的频率函数。
⇒ 应读作「引脚数×驱动强度落在 Xilinx 的 SSO 预算内」, **不等于「333 Mbps 板上一定能跑」**,
后者仍需板级 SI 仿真/眼图。好处是: SSN 这关无论如何不会由这个工具判死。

🔴 **看报告必须连 `Off-Chip Termination` 列一起看**: DRIVE ≥12 时 Vivado 自动把负载模型
换成 `FP_VTT_50`(远端 50Ω 终端), DRIVE 12 默认判 106/150 超标 −141%; 强制
`set_property OFFCHIP_TERM NONE` 后只剩 3/150 超标 −4.1%。本项目真实负载(5 pF + 2-3 cm 无终端)
对应 `NONE`, 而定案的 DRIVE 4/8 恰好就在 `NONE` 下评估。

⚠ `help report_ssn` 的器件列表**没列 Zynq**(只有 V7/K7/A7 + UltraScale), 但
`get_property SSN_REPORT [get_parts xc7z020clg484-1]` = 1, 实跑输出 `SSN Data Version: Production`。
**别被帮助文档吓退。** 另: `report_ssn` 需 placed(不需 route), `-format` 默认 CSV。

**证据与复现**: `pov3d/ssn_2260/` (CONCLUSION.md / 21 组报告 / placed dcp / tcl / xdc)。
负对照跑过: DRIVE 16 @49根/bank = `Partial Analysis;Failed` 182/196, −390% ⇒ 工具确实会判死。

### ✅ 定案引脚表 (152 脚 / 8 时钟 / 按驱动器分组) —— 已实跑确认

`ssn_2260/xdc/RECOMMENDED_pinout_152.xdc` (153 个 PACKAGE_PIN 零重复, 每 bank 恰 38 根, 已复核)

| 配置 | Status | Exceeding | 最差余量 | 四 bank |
|---|---|---|---|---|
| **LVCMOS33 / DRIVE 4 / FAST** (定案) | `Full Analysis;Passed` | **0/152** | **+66.4%** | 66.4 全对称 |
| LVCMOS33 / DRIVE 8 / FAST (退路) | `Full Analysis;Passed` | **0/152** | **+30.9%** | 30.9 全对称 |

152 行 `Off-Chip Termination` **全部 `NONE`** (已逐行核, 无 `FP_VTT_50` 污染)。
余量与 150 脚版完全相同 —— 多的 2 根时钟只是把 bank 34/35 从 37 补到 38。

**分组硬校验**: `ADJACENCY_CHECK: 0 driver(s) with non-contiguous data lanes` ⇒ 48/48 颗
驱动器的 3 根数据脚全部相邻。

**7 系列 HR bank 的结构恰好合适**: 每 bank = 4 个 byte group × 12 根 + 2 根单端,
而 **12 根 = 恰好 4 颗驱动器的数据通道**:
```
时钟组 = [byte group A 全 12 根: 驱动器 0-3] + [B 前 6 根: 驱动器 4-5] + [B 第 7 根: 本组时钟] = 19 根
bank 内: 组0 用 (T3,T2), 组1 用 (T1,T0); 余 T2尾5 + T0尾5 + 2 单端 = 12 根
```

🔴 **唯一妥协: 一个时钟组 19 根 > 一个 byte group 12 根 ⇒ 必然跨 2 个 byte group。**
同 byte group 的驱动器 4/5 距时钟 1-7 个 IOB ✅; 相邻 byte group 的驱动器 0-3 **最远 18 个 IOB** ⚠。
(这只是**封装内物理距离**, 不等于 skew —— 144 路 ODDR 同一 BUFG, 片内 skew 由时钟树定;
它影响的是**板上等长走线的难度**。)
⇒ 若关 3 的 1.5 ns 最后很紧, 备选 **16 时钟**: 3 颗驱动器 (9 数据 + 1 时钟 = 10 脚) 完整装进
一个 byte group, 最远 9 IOB, 总脚数 160 仍放得下。**⚠ 此备选未实跑, 要用先跑一遍, 别外推。**

### 画板编号规则
```
驱动器 #d = bank序号×12 + 组号×6 + 组内序号      (bank序号 0/1/2/3 = bank 13/33/34/35)
数据      = dat[3d .. 3d+2]
时钟 clkout[c],  c = bank序号×2 + 组号,  服务 #(6c) ~ #(6c+5)
```
xdc 每行带 `;# bank 13 grp 0 drv #0 lane 0` 注释, 可直接照连。

### 🔴 两个 Vivado 坑 (排引脚时会撞)
1. **MMCM 输入必须 LOC 到 P 侧 CCIO**。挑了 N 侧会 `ERROR: [DRC PLIO-9] ... LOCed to a N-Type CCIO`
   直接 place 失败。**`IS_CLK_CAPABLE==1` 不足以判定**, 还要看 `PIN_FUNC` 是 `L<n>P_` 还是 `L<n>N_`。
   本表 clk50 = **D20** (`IO_L14P_T2_AD4P_SRCC_35`)。
2. **`PKGPIN_BYTEGROUP_INDEX` 在 7 系列恒为 0** (UltraScale 专用属性)。要从 `PIN_FUNC` 正则抠
   `_T([0-3])_`; `IO_0` / `IO_25` 两根没有 `_Tn_`, 归余量池。

### 关 2 🔴 驱动器 Voc 选型
2260 的 VIC = **1.1~1.3 V**, 输入高阻 (IIN 10 µA, 无内部终端)。
通用四通道 LVDS 驱动器多为 **Voc 1.125~1.375 V**, 纸面上端超窗 75 mV。
- 必须按 datasheet 的 **Voc max ≤ 1.3 V** 筛。
- ⚠ **别用 AC 耦合绕**: mLVDS 数据不是 DC 平衡码, 长连 0/1 会基线漂移。
- ⚠ **别用中心抽头拉 1.2V 掰共模**: LVDS 是电流型输出带共模反馈环, 会跟偏置网络打架。
- ✅ 最省事: **向映己鸿鹄要 2260 的 mLVDS 参考设计 / 推荐驱动器型号** (需用户去问, agent 做不了)。

### 关 3 时钟-数据 skew
tSTU/tHLD = 1/4 tLVCP = **1.5 ns @166.7 MHz**。3 数据 + 时钟必须同封装;
时钟多经一级板上扇出要等长补偿, 或用芯片采样相位调整 (1/8, 1/4, 3/8 tLVCP, ±0.7 ns)。
FPGA 侧 144 路 OSERDES 用同一 BUFIO/BUFR, 输出 skew 可压到 100 ps 内。

### 关 4 线束 (贵但不判死)
192 对 = **384 根导线** + **192 个精密终端电阻** (数据链 200Ω / 模块链 100Ω) 全部上转子。
现在整屏只有一根 50pin 线 ⇒ 连接器 / 线束 / 接口板 / 屏侧转接板全部重做, 且在旋转体上。

## 5. 关 1 任务书: SSN 实算

**器件** `xc7z020clg484-1` (与现役 FS03 同型, 新板暂按同型评估)
**工具** Vivado 2024.2, `report_ssn` 已确认存在 (`help -syntax report_ssn` 探过)
**调用** WSL 里 `cmd.exe /c "cd /d D:\... && call C:\Xilinx\Vivado\2024.2\settings64.bat && vivado -mode batch -source x.tcl"`

**待答问题**
1. 150 根 LVCMOS 输出 @166.7 MHz DDR, 每 bank ~38 根, SSN 余量是否 >0?
2. 哪组 (IOSTANDARD, DRIVE, SLEW) 能过? LVCMOS33 vs LVCMOS18 差多少?
3. 如果过不了, 每 bank 最多能摆几根? ⇒ 反推「必须降到几颗/链」。

**产出** `pov3d/ssn_2260/` 下的 tcl + 报告 + 一份结论 md (含 report_ssn 原文摘录)。
