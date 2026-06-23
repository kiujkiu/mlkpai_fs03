# MLKPAI-FS03-ZYNQ 基础硬件配置 (建 Vivado PS7 工程依据)

来源: 米联客硬件手册 `米联客-MLKPAI_FS03_ZYNQ开发平台硬件使用手册.pdf` + 出厂 `devicetree.dtb` 反编译 (`factory.dts`)。
日期: 2026-06-23。

## 芯片 / 速度档
- **XC7Z020-1CLG484I** — 速度档 **-1**, 工业级(I)
- Vivado part: `xc7z020clg484-1` (建工程时在 part 列表确认精确串)

## DDR (PS BANK502/503)
- DDR3**L**, 单片 **512MB** (256M×16bit), 16-bit 总线, **1066 Mbps** (1.35V)
- 实物芯片: 国产 GD **GDP2BFLM-CB**, **兼容镁光 MT41K256M16TW-107** → Vivado DDR 选 `MT41K256M16 ...` (1066 速度档, 需在 DDR part 下拉确认精确型号)
- DT 确认: `memory@0 reg = <0x0 0x20000000>` = 512MB; ddrc `xlnx,zynq-ddrc-a05`
- ⚠ **DDR 时序精确值最好取米联客参考 Vivado 工程的 ps7 preset** (本下载包没有); 暂按 MT41K256M16/1066 配, 上板前/后用 memtest 验

## 系统时钟
- **PS_CLK = 33.333 MHz** 单端 (PCW_CRYSTAL_PERIPHERAL_FREQMHZ)
- PL 时钟 = 25 MHz 单端 (板载, 给 PL 用)

## PS 外设 (大多用 Zynq 标准默认 MIO)
| 外设 | 配置 | MIO | DT 节点 |
|---|---|---|---|
| **UART1** (console) | 115200 8N1 | **48/49** | serial@e0001000 (serial0/stdout) |
| UART0 | 第二路 | — | serial@e0000000 |
| **GEM0** 千兆以太网 | RGMII (`rgmii-id`), YT8531DH PHY @ MDIO **addr 4**, 25M 晶振 | **16-27** + MDIO 52/53 | gem@e000b000 (okay); gem@e000c000 disabled |
| **USB0** | **Host 模式** (`dr_mode=host`), USB3320 ULPI 1.8V | **28-39** | usb@e0002000 |
| **SD0** | TF 卡 (TXS02612 1.8↔3.3V) | **40-47** | mmc@e0100000 (mmc0) |
| **SD1** | 第二 TF | **9-15** | mmc@e0101000 (mmc1) |
| **QSPI** | W25Q128 16MB 1.8V x1/2/4 | **1-6** | qspi (lqspi) |
| CAN0/1, GPIO, XADC, 2×SPI, I2C | 板上有, 按需 | — | dts 有节点 |

## 板名 / DT 标识
- `compatible = "xlnx,zynq-MZ7X", "xlnx,zynq-7000"` (米联客内部板名 MZ7X)

## 电源序列 (参考)
VBUS_5V → 0.95V VCCINT → 1.8V VCCAUX → 3.3V/1.8V/**1.35V VCC_DDR(DDR3L)** → DDR_VTT

## 待办 / 风险
- 🔴 **DDR 精确时序**: 没米联客 ps7 preset, 用 MT41K256M16/1066 通用配, 风险点。建议设法拿米联客参考 Vivado 工程对齐 DDR。
- MIO 大多标准默认即对; SD1 (MIO9-15) 和 UART 通道要确认不冲突。
- 这块板 Linux 出厂系统已用此配置跑通 (Phase 0 已验), 即硬件配置是 work 的, 我们是在 Vivado 里复刻它。
