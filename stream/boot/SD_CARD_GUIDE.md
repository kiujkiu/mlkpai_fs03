# FS03 SD 启动集成指南 — 自制 BOOT.BIN (v5 bit + AFI FSBL) + mem=256M

日期: 2026-07-08。目标: SD 启动 Linux 时 **FSBL 直接加载我们的 v5 PL bitstream**，
FSBL 的 ps7_init 带 HP0 32-bit AFI 写 (0xF8008000/0xF8008014)，
且内核 cmdline 加 `mem=256M`，把 0x10000000~0x1FFFFFFF (高 256MB DDR) 留给 POV 帧数据。

## 0. 这套东西是怎么工作的 (背景)

- **BOOT.BIN** (本目录, 4.7MB) = 3 个分区:
  1. `fsbl.elf` — 从 v5 `mlkpai_panel.xsa` (2026-07-08 19:32) 生成，ps7_init 含 AFI 写 (已验证 ELF 内 3 处 `0xF8008000` mask-write，对应 3 个硅版本 init 变体)
  2. `system_wrapper.bit` — v5 PL 工程 (`build_panel/mlkpai_panel.runs/impl_1/`)，FSBL 在 u-boot 之前烧进 PL
  3. `u-boot.bin` — **出厂 u-boot 原封不动** (U-Boot 2019.01, 2024-11-08 米联客编译)，从出厂恢复镜像 BOOT.bin 里按 Zynq boot 格式抠出的原始分区 (load/startup=0x4000000)，字节级与出厂一致
- **uEnv.txt** — 出厂 u-boot 的启动脚本会自动加载 FAT 分区上的 `uEnv.txt` 并 `env import`。
  出厂 bootargs 模板是 `${console} root=${root} rw rootfstype=ext4 rootwait ${optargs}`，
  所以我们只需 `optargs=mem=256M` 一行，最小侵入，不改 DTB 不改 u-boot。
- **出厂 DTB 里 bootargs 是空的** (`bootargs=[00]`)，cmdline 完全由 u-boot 生成 — uEnv.txt 生效。
- 6.6 内核 `CONFIG_CMDLINE=""` (用 bootloader cmdline)，`mem=` 是 ARM 标准参数，直接生效。
- ⚠ 出厂 u-boot 启动流程里有 `fpga_loadbit`: 如果 FAT 分区存在 `system.bit.bin`，
  u-boot 会**用它覆盖 FSBL 已加载的 PL**！所以第 2 步必须把它改名拿掉 (文件不存在时 u-boot 静默跳过，不影响启动)。

## 1. 备份 (SD 卡 FAT boot 分区, 插读卡器或板上 Linux 里 /boot 都行)

```
BOOT.BIN        → BOOT.BIN.factory.bak      (出厂 FSBL+u-boot)
uEnv.txt        → uEnv.txt.bak              (如果原来就有的话)
system.bit.bin  → system.bit.bin.factory.bak (如果存在 — 必须改名, 见上)
```
uImage / devicetree.dtb **不动** (6.6 uImage 之前已换好, 出厂 DTB 继续复用)。

## 2. 拷入新文件 (都在 `D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot\`)

| 源文件 | → SD FAT 分区 |
|---|---|
| `BOOT.BIN` | `BOOT.BIN` (覆盖) |
| `uEnv.txt` | `uEnv.txt` (新增/覆盖) |

拷完安全弹出 (Windows) 或 `sync` (Linux)。

## 3. 拨码开关 = SD 启动

模式开关 **PIN1 PIN2 = OFF-OFF** (即 "11" = SD 启动)。
(米联客文档 `02-1_Linux出厂系统测试.pdf`: ON-ON=JTAG(00), ON-OFF=QSPI(01), **OFF-OFF=SD(11)**。)

## 4. 上电验证 (串口 COM13, 115200 8N1; 登录 uisrc / 密码 root)

u-boot 阶段串口应看到:
```
U-Boot 2019.01 (Nov 08 2024 ...)          ← 还是出厂 u-boot
Loaded environment from uEnv.txt           ← uEnv.txt 被吃进去了
console=ttyPS0,115200 earlyprintk root=/dev/mmcblk0p2 rw rootfstype=ext4 rootwait mem=256M
                                           ← bootargs 回显带 mem=256M
```
(不应再出现加载 system.bit.bin 的字样。)

进 Linux 后:
```bash
uname -r                    # 6.6.0-xilinx-g343f487d6341 (确认还是 6.6)
cat /proc/cmdline           # 应含 mem=256M
free -m                     # total ~230 (256M 减内核保留), 不再是 ~460
sudo devmem 0x40010000      # 期望 0x00000008 (STATUS: oe_reg 复位=1, 其他位 0)
                            #   读到值 = PL bit 已由 FSBL 加载, AXI 通;
                            #   若总线挂死/全 F = bit 没加载
sudo devmem 0xF8008000      # 期望 bit0=1 (AFI HP0 读通道 32-bit, FSBL 已写)
sudo devmem 0xF8008014      # 期望 bit0=1 (AFI HP0 写通道 32-bit)
```

## 5. 回滚

任何异常: FAT 分区上 `BOOT.BIN.factory.bak` 改回 `BOOT.BIN`、
`system.bit.bin.factory.bak` 改回 `system.bit.bin`、删掉 `uEnv.txt` 即回出厂启动路径。

## 6. 重新构建 (可复现)

```
# WSL: 从出厂恢复镜像抠 u-boot (只需一次; factory_BOOT.bin 已存本目录)
python3 extract_uboot.py factory_BOOT.bin u-boot.bin

# Windows: FSBL(xsct, 从 mlkpai_panel.xsa) + bootgen
cmd /c D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot\build_boot.bat
```
前提: 仓库根 `mlkpai_panel.xsa` 和 `build_panel\...\impl_1\system_wrapper.bit` 是同一次 v5 build 的产物。
bit 或 XSA 更新后重跑 build_boot.bat 即可 (fsbl_ws/ 会整个重建)。

## 附: 备选路径 (本次没用上, 记录备查)

- **Plan B** (出厂 u-boot 抠不出来时): 用 `~/mlkpai-kernel` 工具链源码编 u-boot-xlnx 2024.x
  `xilinx_zynq_virt_defconfig` — 未走, 因为 Plan A 成功且 0 风险 (env/DDR 参数与出厂完全一致)。
- **Plan C** (完全不动 BOOT.BIN): 出厂 BOOT.BIN + Linux 里 fpga_manager/xdevcfg 加载 bit,
  但 AFI 写必须在加载后手工补:
  `devmem 0xF8008000 32 0x1; devmem 0xF8008014 32 0x1`
  (ps7_init.tcl 原文: `mask_write 0XF8008000 0x00000001 0x00000001`, 0xF8008014 同)。
  缺点: 每次开机要跑脚本, 且 u-boot 的 fpga_loadbit 时序上还会抢 — 不如 Plan A 干净。
