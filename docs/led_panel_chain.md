# FS03 → LED 屏 (P0.9375 COB / ICND2049) 全链路 pin 映射与调试须知

更新: 2026-07-06 21:00 CST (原理图坐标级抽取 + 用户实测线序确认)

## 拓扑

```
FS03 J12 (50pin, Bank13, 全等长, 每线 33Ω)
  ↕ 50pin 线
米联派接口板 mlp_panel_v1.0 (P2=50pin; 24V→TPS54560→5V; HT7533→3.3V; 每线 10Ω)
  ├─ P5 (3pin): 光电传感器 (3.3V / CEP_11P / GND)
  ├─ P1 (30pin) → 屏1     ├─ P3 (30pin) → 屏2 (_2 信号组)
  ↕ 30pin 线 (信号 + 5V)
屏侧转接板 panel_0.93cob_trans (P1=30pin → J1=40pin; TPS563201×2: 5V→3.8V + 3.8V→2.8V)
  ↕ BTB 2×20 (转接板 J1 行序编号 vs 屏 J1 奇偶编号不同, 已逐 pin 核对 100% 对位, 用户实测 OK)
屏 (108× ICND2049 列驱 + 24× ICND3019 行驱 + SPI Flash, VCC=3.8V / VCC_R=2.8V)
```

原理图文件:
- 屏: `mlkpai_fs03/C2-P0.9375-ICND2065-RT5960-IC-SCH.pdf` (位号 ICN2065GP→实贴 ICND2049, RT5960→实贴 ICND3019)
- 接口板: `D:\工程项目\硬件\pov\zynq-mlp_gpio_pV1.0\SCH\mlp_panel_v1.0.pdf`
- 转接板: `D:\工程项目\硬件\pov\zynq-mlp_pV1.1\SCH\panel_0.93cob_trans.pdf`
- 手册: ICND2049 (微信文件), ICND3019 = `zynq_pov/docs/ICND3019_datasheet_CN_V2.0_20220106.pdf`

## 🔴 上电前必须处理

1. **P2.1 (接口板 5V) ↔ J12.1 (VCC_CEP2 = FS03 的 VCC_3V3 经磁珠 L18/3A)**
   接口板把自产 5V 灌进 FS03 的 3.3V 轨 → **对灌, 会损坏 FS03**。
   处理: 50pin 线缆的 **pin1 必须断开** (拔针/剪线), 或接口板 P2.1 脱焊。
   (J12.2=GND ↔ P2.2=GND 没问题; J12.43-50 = CEP2_21~24 对 P2 侧 NC, 安全。)
2. **FS03 跳帽 J1 (2.54mm×3, ADJ_BANK13) 必须跳 1-2 (VCC_3V3)**。
   Bank13 VCCO 可选 3.3/1.8V; 屏侧 245 buffer VCC=3.8V → VIH=0.7×3.8=2.66V,
   必须 LVCMOS33 才驱得动, 1.8V 档全链路不工作。(J2=ADJ_BANK35 与本链无关。)
3. 24V 电源接接口板 P4 (pin3=24V+, pin4=GND; pin1/2 悬空勿接)。屏侧 3.8/2.8V 由链路自动产生。

## 端到端 pin 表 (屏1, 经 J12 → CEP2_x → 接口板 P1 → 转接板 → 屏 J1)

| 信号 | 用途 | FPGA ball | J12 pin | CEP2_ | 接口板 P1 | 屏 J1 pin |
|---|---|---|---|---|---|---|
| DCLK | 2049 CLK | **AB4** | 22 | 10N | 8 | 10 (DCLKIN) |
| LAT | 2049 LE | **AB5** | 21 | 10P | 9 | 12 (LATIN) |
| GCLK | **= 2049 OE** (旧 PWM 芯片遗留名) | **Y5** | 20 | 9N | 10 | 14 (GCLKIN→OE) |
| R1 | 数据 区1 R | AB12 | 6 | 2N | 24 | 24 (R1IN) |
| G1 | 数据 区1 G | AB11 | 10 | 4N | 20 | 25 (G1IN) |
| B1 | 数据 区1 B | Y9 | 7 | 3P | 22 | 22 (B1IN) |
| R2 | 数据 区2 R | Y11 | 11 | 5P | 19 | 23 (R2IN) |
| G2 | 数据 区2 G | Y8 | 8 | 3N | 21 | 20 (G2IN) |
| B2 | 数据 区2 B | Y10 | 12 | 5N | 18 | 21 (B2IN) |
| R3 | 数据 区3 R | AA6 | 16 | 8N | 12 | 18 (R3IN) |
| G3 | 数据 区3 G | AB10 | 13 | 6P | 17 | 19 (G3IN) |
| B3 | 数据 区3 B | Y6 | 19 | 9P | 11 | 16 (B3IN) |
| A | 行驱链 (→屏 AIN) | W12 | 4 | 1N | 26 | 28 (AIN) |
| B | 行驱链 (→屏 BIN) | AA11 | 9 | 4P | 21* | 27 (BIN) |
| C | 行驱链 (→屏 CIN) | AA12 | 5 | 2P | 25 | 26 (CIN) |
| SPI_CS | 屏上 Flash | AB9 | 14 | 6N | 16 | 15 |
| SPI_MOSI | 屏上 Flash | AA9 | 15 | 7P | 15 | 13 |
| SPI_CLK | 屏上 Flash | AA8 | 16* | 7N | 14 | 11 |
| SPI_MISO | 屏上 Flash (输入) | AA7 | 17 | 8P | 13 | 9 |
| SPIN_SYNC | 光电 (输入, P5.1 经 R5 10Ω) | **W6** | 23 | 11P | — | — |

(*) P1 pin 号以接口板原理图为准; J12 pin = 2k+1/2k+2 对 CEP2_kP/N。
**A/B/C = ICND3019 行驱链** (老 160×180 屏 2026-05-27 已实测确认, 同族连接器约定):
**A(AIN) = 3019 DCLK, B(BIN) = 3019 RCLK, C(CIN) = 3019 SDI/DIN** (见 zynq_pov led_panel_seq.v 注释)。新屏首点亮时用单行扫验证一次。
屏内行 LCK/BK 复用列 LAT/OE 线组合产生 (屏原理图 U4 段)。

### 2026-07-07 上电改动 (用户已做)
- FS03: L16/L17 贴上 (VIN_5V0→VCC_CEP1/2), L18/L19 移除 (3.3V 断开) → J12.1 = VIN_5V0。
- **已核安全**: FS03 输入级是二极管 OR (VBUS_+5V→D15, VIN_5V0→D16, 汇入 V_5V→AO3401→VCC_5V0),
  接口板 5V 从 pin1 灌入 = 合法第三路电源, 反灌到不了 USB; 单 24V 可带全系统 (FS03 经 pin1 供电)。
  注意 TPS54560 总预算 5A (FS03 ~1-1.5A + 双屏), 全白高亮时留意。
- FS03 跳帽 J1 已设 3.3V (ADJ_BANK13)。

## 屏2 (P3, _2 信号组)

| 信号 | ball | | 信号 | ball |
|---|---|---|---|---|
| DCLK_2 | W8 | | R2_2 | AB7 |
| LAT_2 | V8 | | G2_2 | V4 |
| GCLK_2(OE) | W7 | | B2_2 | AB6 |
| R1_2 | AA4 | | R3_2 | T6 |
| G1_2 | U4 | | G3_2 | AB2 |
| B1_2 | V5 | | B3_2 | V7 |
| A_2 | W5 | | SPI_CS_2 | AB1 |
| B_2 | T4 | | SPI_MOSI_2 | U6 |
| C_2 | Y4 | | SPI_CLK_2 | U5 |
| | | | SPI_MISO_2 | R6 |

## 芯片要点 (ICND2049, 手册 V2.0)

- 协议**兼容 ICND2038S**: 移位 (CLK 上升沿) + LE 锁存 (LE 长度编码指令, 开路检测用不同 LE 长度) + OE 低有效。与 ICND2047 经验同族。
- **内置双缓存**: OE=0 显示期间可继续移入下一个 16bit → 官方称刷新率 +50%。对 BCM 小 plane 遮蔽问题是利好, 待实测。
- FCLK max **25MHz**; twCLK/twLE/twOE min 20ns; VIH=0.7×VDD, VIL=0.3×VDD。
- 共阴架构: 列驱恒流**源** (输出灌向 LED 阳极), VDD 分轨: R 芯片 2.8V (0.5~15mA), G/B 芯片 3.8V (0.5~20mA); 行驱 ICND3019 NMOS sink — 极性配套, 无 MBI 式对撞。
- 每色每区独立 CLK/LAT/OE ×9 组在屏内由 245 扇出, 连接器侧只有一组 DCLK/LAT/GCLK(OE) — 所有 108 颗列驱同拍。
- 数据组织: 9 根数据线 × 12 颗级联 × 16ch = 1728 列通道; 行 24× 3019。像素几何靠 calib sweep 反推 (工具在 zynq_pov/tools)。

## 验证顺序建议

1. 断 50pin pin1 → 查 FS03 J1 跳帽 3.3V → 只插 FS03 + 接口板, 测 J12.1 电压 (应 3.3V, 无对灌)。
2. 接 24V, 空载测接口板 5V / 转接板 3.8V/2.8V。
3. 全链接屏, PL 先出静态测试图形 (单色 R/G/B 验证九线映射与色序 — 新屏必做单色验证)。
4. 光电 W6 输入通路 (示波器/ILA 看脉冲)。

## 进度快照 2026-07-07 22:40
- ✅ 首点亮 → v2 auto 自主扫描 → **v3 framebuffer IP (icnd2049_panel_fb) 棋盘格上屏**。
- 屏几何: 扫描 160 = 3 区 54+53+53 并行 (rows=54); 列 180 = 12 芯片 × 15 通道; 屏竖放; 色序丝印可信。
- 待办: 取向 (逆时针90°+镜像, orient_f 探针在做) / lane 排列核对 / 手焊 2049 虚焊地图 (顶带右段缺失=右区G链末芯片可疑) / DCLK 25M 运行时切换。
- 工具速查: _panel_auto.tcl (烧+auto) / _panel_chess.tcl (灌棋盘) / _panel_color / _panel_rows / rebuild_panel.tcl (5min 增量)。
