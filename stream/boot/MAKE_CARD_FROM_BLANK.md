# 从**一张空卡**做出能跑的 FS03-Zynq 系统卡

2026-08-26 建立。适用: MLKPAI-FS03 (Zynq7020) 整机装机 / 换卡 / 卡损坏重做。

⚠ 与 `SD_CARD_GUIDE.md` 的区别: 那篇假设**卡上已经有能启动的系统**, 只讲怎么换
`BOOT.BIN`。本篇从**空白卡**开始, 不依赖任何现存的卡, 也不依赖那台已装好的机器。

---

## 0. 需要的四样东西 (全部已在本仓库, 不用再求任何一台板子)

| 文件 | 位置 | md5 | 说明 |
|---|---|---|---|
| `BOOT.BIN` | `stream/boot/BOOT.BIN` | `98faaaf9…` | FSBL(含 HP0 32-bit AFI 写) + v5 bitstream + 出厂 u-boot |
| `uImage` | `stream/boot/board_backup/uImage.6.6.0-xilinx` | `fcab2f1b…` | **6.6.0-xilinx-g343f487d6341**, 现役内核 |
| `devicetree.dtb` | `stream/boot/board_backup/devicetree.dtb` | `e945217d…` | 🔴 **就是出厂那份**, 6.6 内核直接复用, 没改过 |
| `uEnv.txt` | `stream/boot/uEnv.txt` | — | 内容一行: `optargs=mem=256M` |
| `rootfs.tar.gz` | `stream/boot/board_backup/rootfs.tar.gz` | 见同目录 `.md5` | 完整 Debian + POV 软件 + systemd 服务 |

**备用/回退件** (同目录, 出问题时用):
- `BOOT.bin.card_factory.bak` — 原厂 BOOT.bin (1,776,640 B), 想回出厂状态用
- `uImage.card_factory419` — 出厂 4.19 内核。⚠ **它跑不了 POV 软件** (`povmem.ko`
  是给 6.6 编的), 只用来验证"卡和板子本身是好的"
- `BOOT.bin.lz4x3` — 带 3 个 PL lz4 解码引擎的版本, 见 §6

---

## 1. 分区 (Linux / WSL 下操作; ⚠ 确认设备名, 写错盘会毁硬盘)

```bash
lsblk                      # 认准容量, 比如 29.2G 的那个 = /dev/sdX
sudo umount /dev/sdX*      # 确保没挂载

sudo parted -s /dev/sdX mklabel msdos
sudo parted -s /dev/sdX mkpart primary fat32 1MiB 101MiB
sudo parted -s /dev/sdX set 1 boot on
sudo parted -s /dev/sdX mkpart primary ext4 101MiB 100%

sudo mkfs.vfat -F 32 -n BOOT /dev/sdX1
sudo mkfs.ext4  -L rootfs      /dev/sdX2
```

🔴 **分区 1 必须置 boot 标志**, 且必须是 FAT32 —— Zynq 的 BootROM 只认这个。
🔴 **分区顺序不能反**: u-boot 的 bootargs 写死了 `root=/dev/mmcblk0p2`。

## 2. 灌 boot 分区

```bash
cd <repo>/mlkpai_fs03/stream/boot
sudo mount /dev/sdX1 /mnt/card

sudo cp BOOT.BIN                          /mnt/card/BOOT.BIN
sudo cp board_backup/uImage.6.6.0-xilinx  /mnt/card/uImage
sudo cp board_backup/devicetree.dtb       /mnt/card/devicetree.dtb
sudo cp uEnv.txt                          /mnt/card/uEnv.txt
sudo cp BOOT.bin.card_factory.bak         /mnt/card/BOOT.BIN.factory.bak   # 退路, 可选

sync; sudo umount /mnt/card
```

🔴 **卡上绝不能有 `system.bit.bin`**。出厂 u-boot 的启动流程里有 `fpga_loadbit`:
一旦发现这个文件, 它会**覆盖 FSBL 已经烧好的 PL**, 于是你的 bitstream 白装了
(文件不存在时 u-boot 静默跳过, 不影响启动)。若从原厂卡改造而来, 把它改名成
`system.bit.bin.factory.bak`。

## 3. 灌 rootfs

```bash
sudo mount /dev/sdX2 /mnt/card
sudo tar xzf board_backup/rootfs.tar.gz -C /mnt/card --numeric-owner
sync; sudo umount /mnt/card
```

⚠ `--numeric-owner` **不能省** —— 否则属主会按**本机**的 /etc/passwd 重映射,
板上的 `uisrc` (uid 1000) 会变成别人, systemd 服务和家目录权限全乱。

打包时已排除: `/tmp/*`、`/var/log/*`、`/home/uisrc/*.log`、`~/.cache`。
所以解包后 `/var/log` 是空的, 正常。

## 4. 上电

拨码开关拨到 **SD 启动**。串口 COM13 / 115200 8N1, 登录 `uisrc` / 密码 `root`。

**逐条判据** (全过才算成功):

| # | 检查 | 期望 | 不过说明 |
|---|---|---|---|
| 1 | u-boot 串口 | `U-Boot 2019.01 (Nov 08 2024)` + `Loaded environment from uEnv.txt` | BOOT.BIN 没写对 / 分区 1 没 boot 标志 |
| 2 | 有没有加载 system.bit.bin | **不该出现**这一行 | 见 §2 的 🔴 |
| 3 | `uname -r` | `6.6.0-xilinx-g343f487d6341` | uImage 拷错了(可能拷成 4.19 那个) |
| 4 | `cat /proc/cmdline` | 含 `mem=256M` | uEnv.txt 没生效 |
| 5 | `free -m` | total ≈ 230 | 同上 |
| 6 | `sudo devmem 0x40010000`(或 /dev/mem 读) | `0x00000008` | PL 没加载 —— FSBL 没烧成 |
| 7 | `sudo devmem 0xF8008000` / `0xF8008014` | 两者 bit0 = 1 | **HP0 的 32-bit AFI 没配**, 面板取数会坏。FSBL 里那 3 处 mask-write 丢了 |
| 8 | `systemctl is-active povrxd` | `active` | rootfs 解包不全 / 服务没使能 |

⚠ 判据 6/7 是**硬判据**: 它们证明"FSBL 真的跑了并且配对了", 光看 Linux 起来了不够。

## 5. 装完之后

新机器的显示内容、转速、槽数配置都在 rootfs 里带过来了 (`/home/uisrc/pov_boot.sh`、
systemd drop-in、`helix3b.pvs` 开机动画)。**如果新机器的机械参数不同**
(转速/屏偏心距/槽数), 要改 `pov_boot.sh` 里的 `N_SLICES`、`PHASE_B` 等。

屏幕自检: 用 `tools/gen_wedge.py --pattern colors8` 生成 8 色循环图, 推流验收 ——
判据见该工具的输出说明 (串色/坏行/坏芯片/plane0 是否到位)。

## 6. 要不要用带 PL lz4 解码器的 BOOT.BIN

`BOOT.bin.lz4x3` 把解码从 CPU 搬到 PL, 帧率 5.5 → 10-12 fps。但:
- 它需要板端 `pov_rxd` 支持 `--pl-lz4` (rootfs 里带的这版已支持)
- 截至 2026-08-26, `fold_a` 的画面**还没经过人眼验收**
- **给别人用的机器建议先用 `BOOT.BIN` (pre-lz4)**, 那是长期验证过的

想换的话**不用重做卡**: 2026-08-25 验证过运行时换 bit 的路子 ——
FPGA Manager + configfs overlay, 板子转着不用断电。配方见
`stream/boot/plbin/` 和记忆 `feedback_dr1_load_bit_without_jtag`。
之所以成立: 新旧 FSBL **字节完全相同** (HP1/2/3 是 64-bit 复位默认值, 启用它们
不产生任何 ps7_init 写), 而 AFI 的 `0xF8008000` 是 PS 侧寄存器, 不被 PL 重配置清掉。

## 附: 这份材料是怎么来的 (2026-08-26)

- `uImage` / `devicetree.dtb` / `rootfs.tar.gz` 从**现役那台**取的 (板子当时已装好并在跑)
- 🔴 教训: 在此之前, 这三样**只存在于那一块板子上**, 本地零副本。
  文档里提到的 `~/mlkpai-kernel` 内核源码在换开发机时丢了, 官方站上
  FS03 只有安路版 (DR1V90G/DR1M90G) 没有 Zynq 版。
  **一块板子挂掉就等于整套系统失传** —— 所以有了这份备份和这篇文档。
- ⚠ 另一个教训: 用 `ls` 看 WSL 挂载的 SD 卡**会撒谎** (显示空目录, 实际有文件)。
  判断卡上有什么要**直接读原始扇区**或用 Windows 侧确认。

## 附二: 取 rootfs 时踩的坑 (2026-08-26)

🔴 **别走 WiFi 传 rootfs**。板子跟着屏在转, 天线环境本来就差,
587 MB 传了三次都在中途 `Software caused connection abort` 断掉
(40% / 12 块 / 13 块各断一次), 前后折腾了一个多小时。

**下次直接拔卡插读卡器**: USB 2.0 约 25 MB/s, 1.7 GB 原始数据 70 秒读完,
而且压缩交给 PC 做 (Zynq 的 A9 跑 gzip 只有 0.78 MB/s, 打包 1.7GB 要 15 分钟)。
唯一门槛是 WSL 挂 ext4 要管理员权限, 在**管理员 PowerShell** 里:
```powershell
wsl --mount \\.\PHYSICALDRIVE<N> --partition 2
```

⚠ 如果非要走网络, 这几条是实测有效的:
1. **边打包边分块** (`tar | gzip -1 | split -b 40M`), 不要等整包做完再传
2. **打包落点别放 `/tmp`** —— 这台机器的 `/tmp` 虽然在磁盘上(不是 tmpfs),
   但**开机会被清空**, 断电重来一次就白打包了。放 `/home/uisrc/` 下
3. `gzip -1` 比默认档快 45% (0.78 vs 0.54 MB/s), 压缩率损失可以接受
4. 逐块传 + 失败只重传那一块, 最后 `cat p_a? > x.tar.gz` 合并, 用 md5 对整包校验

## 附三: 同一块 FS03 的不同屏 (ICND2049 vs ICND2047)

`docs/led_panel_chain.md` 证实: 2049 屏和 2047 屏用的是**同一块 MLKPAI-FS03**,
连引脚都一样 (`DCLK | 2049/2047 CLK | Y10`)。所以两台机器的卡**大部分通用**:

| | 通用? |
|---|---|
| `uImage` | ✅ 实测两张卡逐字节相同 (`fcab2f1b`) |
| rootfs | ✅ 同一块板同一个内核 |
| `BOOT.BIN` | ❌ 里面的 bitstream 是按屏编的 |
| `devicetree.dtb` | ❌ 实测不同 (2049 是 `665800cd`, 2047 是 `e945217d`) |

⇒ 从一台的卡改造成另一台, **只要换 `BOOT.BIN` + `devicetree.dtb` 两个文件**,
而这两个都在 FAT 分区 —— **Windows 直接读写, 不用碰 ext4, 不用管理员权限**。

⚠ 但别拿正在服役的机器的卡去改 —— 恢复难度大。要么用空卡从零做(见正文),
要么先把那张卡完整镜像下来。
