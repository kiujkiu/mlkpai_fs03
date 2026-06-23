# MLKPAI-FS03-ZYNQ 工程 (米联客 XC7Z020-1)

为 USB-WiFi + Linux + (后续 POV PL) 换板到米联客 MLKPAI-FS03 后的 Vivado/PS 基础工程。
板子身份/选型/USB host/dongle 见 memory `project_mlkpai_fs03_usb_wifi_board`。

## 现状 (2026-06-23)
- **Linux 侧已通**: 出厂 Debian Buster SD 启动 + USB host 确认; 自编 **6.6 内核(带 mt7921u 无线栈)** 已 kernel-swap 装板验证通过 (工件在 WSL `~/mlkpai-kernel/`).
- **本工程 = PS7 基础硬件配置**: 复刻出厂板子的 Zynq PS 配置 (DDR/MIO/时钟), 出 XSA, 给 PetaLinux/内核 DT 和后续 POV PL 用.

## 结构
- `docs/board_config.md` — 基础配置参数 (芯片-1档/DDR/MIO/时钟, 米联客手册+出厂dtb抠的)
- `docs/factory.dts` — 出厂 devicetree.dtb 反编译 (PS 外设/MIO 真值参考)
- `vivado/create_zynq_ps.tcl` — 建 Vivado 2024.2 PS7 工程脚本

## 关键参数速查
- part `xc7z020clg484-1` (速度 -1, 工业)
- DDR3L 512MB / 16bit / 1066 / MT41K256M16 兼容
- PS_CLK 33.333M; UART1=console(MIO48/49); GEM0 RGMII(MIO16-27); USB0 host(MIO28-39); SD0(40-47)/SD1(9-15); QSPI(1-6)

## 待办
- [ ] 跑 `create_zynq_ps.tcl` 建工程 + validate
- [ ] 确认 DDR 时序 (理想拿米联客参考工程对齐; 暂用 MT41K256M16/1066 通用配)
- [ ] 出 XSA
- [ ] (后续) PetaLinux/内核 DT 对齐 + POV PL 移植
