# 02 正反双屏架构设计 (dual panel, 背靠背 180°)

更新: 2026-07-13 | 基线 RTL: `vivado/hdl/icnd2049_panel_pov.v` (v5) + `vivado/hdl/ddr_slice_fetch.v`
范围: 双屏取帧/相位/寄存器/供电/验证路线。列驱引擎 (ICND2047 双沿, 另文档) 本文当黑盒。

## 0. 目标与总账

- 电机 13 rps, 两屏背靠背相位差 180°。每屏每转刷全部 360 片 → 同一体素在每转被扫过 2 次 → **体积帧率 26 fps** (半圈一个完整体积)。
- 每屏切片率 = 13 × 360 = **4680 片/s**, **每片 213.7 µs** (76.92 ms/圈 ÷ 360)。
  ⚠ 任务书里 "21.4µs/片" 是 10× 笔误, 全文按 213.7 µs 设计; 即便未来提速到 26 rps 也还有 106.8 µs/片。
- 两屏显示**同一份** 360 片帧数据, 屏 B 的 slice_idx = (屏 A idx + 180) mod 360 → **DDR 数据零翻倍**。
- 两屏 slice 边界在时间上**同拍跳变** (同一 angle_tracker, 固定 idx 偏移), 所以每 213.7 µs 出现一次 "双取帧" 突发, 这是带宽设计点。

## 0.5 ⚠ 几何更新 (2026-07-27): 两屏有间距, 不再过转轴

实机改成**正反背靠背 + 13.8mm 间距**, 屏面不穿过旋转圆心 (对称安装, 各偏 6.9mm)。

**对本文档的影响: 无。** 两屏关于轴对称 ⇒ 屏B@θ ≡ 屏A@(θ+180) 严格成立 ⇒
`PHASE_B=180`、单份 360 片 DDR 数据、213.7 µs/片 带宽预算**全部照旧**, RTL 一行不用改。

**唯一变化在 host 切片生成** (`tools/gen_anime_slices.py --gap-mm 13.8`):
切片从「过轴的子午面」变成「与轴平行、垂距 6.9mm 的偏移平面」, 采样式改为
`wx = u·cosθ − off·sinθ; wz = u·sinθ + off·cosθ` (off = 7.36 px)。

两个语义变化要记住:
1. **360 片不再是 2× 冗余** — 穿心时 slice i 与 i+180 互为镜像 (信息量只有 180 片),
   有间距后 360 片各不相同。存储量没变, 但余量没了。
2. **中心盲区** 半径 6.9mm 永远扫不到 (占直径 9.2%, 但仅占截面积 0.85%, 轴心是模型内部, 基本无损)。

风险: 同一体素半圈内被 u>0/u<0 两次命中, 机械角度差近 180° →
**视角相关着色 (depth-fade/z-buffer) 会重影**。详见记忆 project_pov3d_offset_axis_geometry。

## 1. fetch 架构选型: 方案 A (单 fetch 串行喂两屏) ✅

### 1.1 带宽账

| 项 | 数值 |
|---|---|
| 双屏取帧率 | 2 × 4680 = 9360 次/s |
| 每次取帧 | 11664 B (9 lane × 54 row × 6 word × 4B) |
| 持续带宽需求 | **109.2 MB/s** |
| HP0 32-bit @ FCLK0 50 MHz 理论 | 200 MB/s |
| 现 RTL 16-beat burst 有效带宽 | ~16/(16+~20 AR 往返) ≈ 44% → **~89 MB/s ✗ 不够** |
| 升 64-beat | ~76% → 152 MB/s ✓ |
| 升 256-beat (1KB, v6 同款) | ~92% → **~185 MB/s ✓ 余量 1.7×** |

逐片时间预算 (50 MHz, 256-beat): 一片 = 2916 beat ≈ 12 个 1KB burst, 2916 + 12×~25 拍开销 ≈ 3216 拍 ≈ **64.3 µs**。串行 A→B 两片 ≈ **129 µs < 213.7 µs** ✓ (余量 1.65×)。zynq_pov v6 实测背书: 1KB×64 burst 取 64KB, 8KB 紧凑帧 26 µs, 4.4× 余量 — 同套 FSM 移植可信。

**结论: 保持 50 MHz 不动 (引擎所有时序常数按 50M 标定), 只把 `ddr_slice_fetch` 的 burst 上限 16→256 beat** (4KB 边界动态截断逻辑已有, 保留即可; slice 内 2916 word 线性递增, 不跨 slice 无对齐问题)。

### 1.2 方案 B (双 fetch 双 AXI) 为什么不选

- **瓶颈在 HP0 口不在 fetch 引擎**: 双引擎并发最终仍串行化在同一 DDR 口, 吞吐不增, 只是把排队从 RTL 层挪到 SmartConnect 仲裁层。
- **历史教训 (zynq_pov, feedback_pov_4x_ip_breaks_hdmi)**: 4× pov IP 经同一 smc 并发突发直接把 HDMI 打成噪点, solo-fire 全正常 — 多 master 并发经 smc 的仲裁/交织行为有未量化的失效模式, 排查成本极高。本设计虽是纯读 (风险低于当年读写混合), 但没有收益就不要引入这个变量。
- 双 fetch 还要多一套 AR FSM + smc SI 口 + BD 连线, v6 经验单 master 单 outstanding 就是最好调的形态。
- 若未来真需要 (如 26 rps + 更大 slice), 升级路径是 fetch 双 outstanding / HP0+HP2 分口, 不是现在做。

### 1.3 方案 A 数据通路

```
angle_tracker ──slice_idx──┐
                           ▼
                 dual 触发器 (idx 变化 → 锁存 idx + slice_base)
                           │
              ┌── fetch_go(A, idx) ──► ddr_slice_fetch ──► fb_A (9×512×32 BRAM)
              └── A done 后 fetch_go(B, (idx+phase)%360) ─► 同一 fetch, 写口带 target 标志 ──► fb_B
                           │
                   engine_A 读 fb_A → P1 引脚组
                   engine_B 读 fb_B → P3 引脚组 (_2, XDC 已备)
```

- fetch 增加 1-bit `target` 输入, `fb_we` 输出带 target, 顶层按 target 路由到 fb_A/fb_B 写口。
- **屏 B 数据晚到 ~64 µs ≈ 0.3 片 (0.3°)**: 与 v5 已接受的 "帧内换片撕裂" 同量级, 转动下不可见, 不补偿。如强迫症可在 phase 微调里吃掉。
- fb BRAM ×2: 每屏 9 lane × 512 × 32bit = 18 KB, 双屏 36 KB, 7020 (140 个 BRAM36) 无压力。
- 引擎复制: 建议把 v5 的 auto FSM + ICND3019 FSM + fb + 输出寄存器抽成 `panel_engine` 子模块实例化 ×2, 顶层只留 AXI / angle_tracker / fetch / 触发器。⚠ BD module_ref 加端口有已知缓存坑 (feedback_vivado_bd_module_ref_update / addr_width_cache), 改完按老配方删 cache 重建。

## 2. angle_tracker 双屏相位

### 2.1 +180° 加在哪一级: **idx 域** ✅

- `idx_B = at_slice_idx + phase_b; if (idx_B >= n_slices) idx_B -= n_slices;` — 一个加法器 + 一个比较器, 组合逻辑完事。
- 不碰角度域 (acc/slice_period): 角度域偏移要么复制一套累加器, 要么在除法/插值里加偏置, 都引入第二套误差源, 且 v5 的 acc 硬回零语义会被搅乱。
- 粒度: 1 idx = 1° (n_slices=360), 机械安装误差 ±几度 → ±几个 idx, 粒度够。若将来要亚度, 正路是 n_slices 升 720 (数据侧同步加密), 不是在角度域打补丁。
- angle_tracker 本体**零改动**: 仍单实例、单 slice_idx 输出, 两 bug 修复版语义原样保留。

### 2.2 翻页原子性 (两屏必须同帧)

v5 现状: fetch 在 `fetch_go` 拍锁存 slice_idx, 但 `slice_base_r` 是**每次取帧各自采样**。双屏下若 pov_rxd 恰在 A、B 两次取帧之间写 0x18 翻 bank, 会出现 A=旧帧 / B=新帧的半转分裂。

**修法: pair 级锁存。** dual 触发器在 idx 变化那一拍把 `slice_base_r` 快照进 `base_lat`, A、B 两次 fetch 都用 `base_lat`:

```verilog
if (pov_en && !pair_busy && at_slice_idx != df_last_slice) begin
    df_last_slice <= at_slice_idx;
    base_lat      <= slice_base_r;   // 翻页原子点: 每 pair 只采样一次
    pair_state    <= FETCH_A;        // → done → FETCH_B → done → idle
end
```

- pair 期间 (`pair_busy`) 新 idx 到来整对丢弃 (丢帧跳最新, 与 v5 同语义), **绝不允许只丢 B 不丢 A** — 保证任意时刻两屏 fb 来自同一 bank 同一帧。
- 残余不原子窗口: 翻页发生在两个 pair 之间时, 相邻两片 (1°) 分属新旧动画帧 — 与 v5 单屏行为完全一致, 26 fps 相邻帧差异, 不可见, 接受。
- pov_rxd 写 0x18 的时机**完全不用管** ✓。

## 3. 寄存器映射 (awaddr[5:2] 解码, 0x00-0x18 全部原样兼容)

| 地址 | 方向 | 定义 |
|---|---|---|
| 0x10 POV_CTRL | W | [0]=pov_en [1]=fake_en [31:16]=n_slices (原样); **新 [2]=dual_en** (0=纯单屏, B 引擎静默消隐, 完全向后兼容); **[3]=fb_sel_b** (AXI fb 窗 awaddr[15]=1 的写落到 fb_B, 调试直灌用, pov_en=0 时有效) |
| 0x14 / 0x18 | W | fake_period / slice_base (原样) |
| **0x1C PHASE_B** | W | [8:0] 屏 B slice 偏移, 复位默认 **180**。软校准: 机械装配差 ±k° 直接写 180±k, 0..359 全范围有效 |
| **0x1C** | R | {locked, 6'b0, idx_B[8:0], pair_miss[15:0]} — idx_B 回读核相位; pair_miss = 因 pair_busy 丢弃的取帧对饱和计数 (带宽/转速余量哨兵) |
| **0x20 BRIGHT_B** | W | [7:0] oe_window_B (0 = 跟随屏 A 的 oe_window); 两屏 LED bin/驱动板差异的亮度匹配旋钮。屏 A 亮度沿用 0x0C subcmd10 的 oe_window (不动老脚本) |
| 0x10 | R | 原 status 追加 [11]=dual_en [12]=engine_B_busy |

- 每屏独立亮度只做 oe_window 级 (整屏), 不做逐色 — 色平衡属于数据侧预补偿 (R×0.xx 老套路)。
- 相位标定流程: 灌 "0° 径向亮线" 标定帧 → 低速转 → 目视两屏亮线是否重合 → 写 0x1C 微调 → 重合后记录进板端启动脚本。

## 4. DDR 布局与 pov_rxd 兼容性: **零改动** ✅

- 帧格式不变: 360 片 × 0x3000 stride × 11664B 有效 = 每 bank 1.08 MB 有效 (占位 1.35 MB)。
- 双 bank 0x1000_0000 / 0x1050_0000 翻页机制**不动**: 两屏读同一 bank 同一帧, 屏 B 只是 idx 加偏移, DDR 侧没有 "屏 B 的数据" 这个概念。
- 板端 pov_rxd **零改动**: 仍是 "写满 inactive bank → 写 0x18 slice_base 翻页"。原子性由 §2.2 的 pair 级锁存在 RTL 侧兜底。
- 唯一板端可选动作: 启动脚本加两笔写 (0x10 置 dual_en / 0x1C 写标定相位), 属配置不属协议。
- 带宽复核: pov_rxd WiFi 写 DDR (~3.5 MB/s) + 双屏读 109 MB/s, 对 DDR3 (~1 GB/s 级) 合计 <12%, PS DDR 控制器无压力。

## 5. 光电传感器: 一个 spin_sync 够 ✅

- 屏 B 与屏 A 是刚体固连, 相对相位是常数 → 第二个传感器提供不了任何新信息, 反而多一路去抖/同步/标定。结论: **单传感器 + idx 偏移**, 硬件不加。
- 插值精度账 (13 rps, 50 MHz):
  - rev_period ≈ 3,846,154 拍; slice_period = rev_period/360 ≈ 10,684 拍, 截断误差 <1 拍/片 → 每转累计漂移 <360 拍 = 7.2 µs = **0.034 片 (0.034°)**, 屏 B 在半圈处承受其一半 ≈ 0.017° — 可忽略, 串行除法器精度不用升。
  - 真正的新敏感项是**圈内转速波动**: 屏 B 的绝对角度正确性依赖 "半圈内匀速" 假设。圈内速度波动 ε 引起屏 B 中位角误差 ≈ ε×90°; 要求接缝 <1° → 圈内波动 <1.1%。13 rps 无刷电机 + 转盘惯量通常 <0.5%, 但**验证阶段要实测** (连续读 0x14 rev_period 抖动 + 目视 180° 处接缝)。
  - 脉冲硬回零的接缝: 屏 A 的修正缝出现在传感器角 (0°), 屏 B 的同拍跳变缝出现在物理 180° — 体积里两条半亮度接缝线, 与单屏一条同性质, 锁速稳定后幅度 <0.1°。
- locked 判据 (相邻圈差 <12.5%) 只做指示不做门控 (bug2 修复语义), 双屏不需要收紧; 递增转速阶段用 0x18 slice_max / 0x1C pair_miss 做量化哨兵。

## 6. 供电账: TPS54560 5A 临界, 要实测 + 软限流

| 负载 | 电流 (5V 轨折算) |
|---|---|
| FS03 本体 (经 50pin pin1) | ~1.0–1.5 A |
| 屏 ×1 全白最坏 (overlap 1/4 亮度, 实测 1.9A@3.8V ≈ 7.2W, 转接板 buck ~90% 效率, 含 2.8V R 轨小头) | ~1.7 A |
| 屏 ×2 | ~3.4 A |
| **合计最坏** | **~4.4–4.9 A ≈ 5A 上限, 无余量** |

- 转接板侧没问题: 每屏自带 panel_0.93cob_trans (TPS563201 3A), 1.9A@3.8V 占 63%, 双屏各自独立 ✓。
- 瓶颈只在接口板 TPS54560 5V 总口。对策 (按顺序):
  1. **POV 实景内容占空比低** (典型 <20% 像素点亮), 全白是不会出现的合成最坏; 但不能拿这个当设计依据, 只当一级余量。
  2. **软限流**: oe_window_A + oe_window_B 之和封顶 (板端配置约定, 如各 ≤48/192 = 1/4), 双屏同时全白也压在 ~4.4A 内; 需要更亮时先实测再放。
  3. 验证路线每档**实测 5V 轨电压跌落 + TPS54560 温度** (§7), 24V 输入侧电流表常备。
  4. 若确认不够: 接口板 P4 24V 现成, 加第二片 buck 单独喂屏 2 的 30pin 5V (P3 供电脚割线飞), 信号不动 — 留作 B 计划, 不进首版。
- 顺带: 双屏共 24 根信号线经 50pin 排线, 同时翻转的 SDI ×18 + DCLK ×2 — 排线地回流密度翻倍, 已有 33Ω+10Ω 串阻, 首测时示波器看屏 2 远端波形即可, 预期无事 (双屏共享信号干扰旧案是共享 SI 线, 本设计两屏信号完全独立)。

## 7. 分阶段验证路线

| 阶段 | 内容 | 通过判据 |
|---|---|---|
| **0. 单屏 2047 回归** | 2047 双沿引擎 (黑盒) 替换进 v5 单屏路径, P1 口, fake + sensor 全跑 | 与 v5 收官行为等价 (棋盘/anime/光电) |
| **1. 双屏静态点亮** | dual_en=0, fb_sel_b=1 经 AXI fb 窗直灌 fb_B, engine_B auto 扫描; 屏 2 单色 R/G/B 逐色 | P3 全 24 线映射/色序确认 (新屏必做单色, 老规矩); 双屏同时点亮测 5V 轨电流第一笔 |
| **2. fetch 扩 burst 单屏回归** | ddr_slice_fetch 16→256 beat, 先单屏 pov_en 跑满 fake 4680 片/s | 画面无损, pair_miss=0, ILA 抽查 burst 不跨 4KB |
| **3. 双屏 fake POV** | dual_en=1, fake_period 从慢 (~500 片/s) 递增到 4680 片/s; 棋盘 + anime | 两屏内容正确且 idx 差恒 180 (读 0x10/0x1C 对账); 4680 片/s 下 pair_miss 恒 0 = 带宽账落实 |
| **4. 双屏 sensor 低速** | 光电接入, 手拨/电机最低速 (~3 rps) | locked=1, 两屏图像各自稳定无滚动 |
| **5. 电机递增** | 3 → 6 → 10 → 13 rps 逐档, 每档 ≥2 min | 每档记录: locked / slice_max=359 / pair_miss / rev_period 抖动 / 5V 轨电压+电流 / TPS54560 温度 |
| **6. 相位与亮度标定** | 径向亮线标定帧, 目视 180° 重合度 → 0x1C 微调; 全白限亮帧 → 0x20 匹配两屏亮度 | 接缝 <1 片宽; 两屏目视等亮; 标定值固化进板端启动脚本 |

回滚保障: dual_en=0 时 RTL 行为与 v5 逐位等价 (B 引擎复位消隐、fetch 只发 A), 任何阶段可退单屏。

## 8. RTL 改动清单 (汇总)

1. `ddr_slice_fetch.v`: burst 上限 16→256 beat (改 `lim_words` 常数); 加 1-bit `target` 透传到 fb 写口。
2. 顶层: auto FSM + 3019 FSM + fb + 输出寄存器抽 `panel_engine` ×2 实例; dual 触发器 (pair FSM + base_lat + idx_B 加偏移); 新寄存器 0x1C/0x20 + 0x10 扩位; 端口加 `panel_*_2` 一组 (XDC 已备, 直接解注释)。
3. `angle_tracker`: 零改动。
4. BD: module_ref 加端口按老配方处理缓存坑; smc/HP0 拓扑不变 (仍单 m_axi)。
