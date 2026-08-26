#!/bin/bash
# make_card_image.sh — 做一张 FS03-Zynq 系统卡的完整镜像 (WSL 内跑, 普通权限即可)
#
#   bash stream/boot/make_card_image.sh [输出路径] [主机名]
#   默认: /mnt/d/claude_workspace/pov3d/fs03_card.img.gz   主机名 pov2
#
# 产出的 .img.gz 直接喂给 balenaEtcher 烧卡 (它认 gz, 不用解压)。
#
# 🔴 为什么绕这一圈而不直接写卡: WSL2 访问 Windows 盘走 9p, 写可移动介质会被拒
#    (sudo 也没用 —— Linux 的 root 管不到 Windows 文件系统驱动), 而且 ext4 需要
#    块设备级访问, 9p 不提供。全程在 loop 设备上做就没这些问题。详见
#    MAKE_CARD_FROM_BLANK.md 附四。
set -e

OUT="${1:-/mnt/d/claude_workspace/pov3d/fs03_card.img.gz}"
NEWHOST="${2:-pov2}"
B="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMG="$HOME/_fs03_card_build.img"
SIZE_MB=2600            # rootfs 解出来 1.3GB, 留够余量; 卡上多余空间开机后 resize2fs 可扩

need() { command -v "$1" >/dev/null || { echo "缺 $1 —— sudo apt-get install -y parted dosfstools"; exit 1; }; }
for t in parted mkfs.vfat mkfs.ext4 losetup; do need $t; done

for f in "$B/BOOT.BIN" "$B/board_backup/uImage.6.6.0-xilinx" \
         "$B/board_backup/devicetree.dtb" "$B/uEnv.txt" "$B/board_backup/rootfs.tar.gz"; do
    [ -f "$f" ] || { echo "缺源文件: $f"; exit 1; }
done

cleanup() {
    sudo umount /mnt/_img_boot /mnt/_img_root 2>/dev/null || true
    [ -n "$LOOP" ] && sudo losetup -d "$LOOP" 2>/dev/null || true
    rm -f "$IMG"
}
trap cleanup EXIT

echo "=== 1/6 建镜像 ${SIZE_MB}MB + 分区 ==="
rm -f "$IMG"; truncate -s ${SIZE_MB}M "$IMG"
sudo parted -s "$IMG" mklabel msdos
sudo parted -s "$IMG" mkpart primary fat32 1MiB 101MiB
sudo parted -s "$IMG" set 1 boot on          # 🔴 Zynq BootROM 只认带 boot 标志的 FAT32
sudo parted -s "$IMG" mkpart primary ext4 101MiB 100%
LOOP=$(sudo losetup -fP --show "$IMG")

echo "=== 2/6 格式化 ==="
sudo mkfs.vfat -F 32 -n BOOT "${LOOP}p1" >/dev/null
sudo mkfs.ext4 -q -L rootfs -F "${LOOP}p2"
sudo mkdir -p /mnt/_img_boot /mnt/_img_root
sudo mount "${LOOP}p1" /mnt/_img_boot
sudo mount "${LOOP}p2" /mnt/_img_root

echo "=== 3/6 灌 boot 分区 ==="
sudo cp "$B/BOOT.BIN"                        /mnt/_img_boot/BOOT.BIN
sudo cp "$B/board_backup/uImage.6.6.0-xilinx" /mnt/_img_boot/uImage
sudo cp "$B/board_backup/devicetree.dtb"     /mnt/_img_boot/devicetree.dtb
sudo cp "$B/uEnv.txt"                        /mnt/_img_boot/uEnv.txt
# 🔴 卡上绝不能有 system.bit.bin: u-boot 的 fpga_loadbit 会覆盖 FSBL 烧好的 PL

echo "=== 4/6 解 rootfs (几分钟) ==="
sudo tar xzf "$B/board_backup/rootfs.tar.gz" -C /mnt/_img_root --numeric-owner

echo "=== 5/6 重置机器标识 (否则两台机器会抢同一个 IP) ==="
R=/mnt/_img_root
echo "$NEWHOST" | sudo tee $R/etc/hostname >/dev/null
sudo sed -i "s/\bpov\b/$NEWHOST/g" $R/etc/hosts
sudo truncate -s 0 $R/etc/machine-id                    # systemd 首启重新生成
sudo rm -f $R/var/lib/dbus/machine-id
sudo ln -sf /etc/machine-id $R/var/lib/dbus/machine-id
sudo rm -f $R/etc/ssh/ssh_host_*
# 🔴 删 host key 还不够: sshd 首启**不一定**自动生成 (2026-08-26 pov2 就没生成,
#    SSH active 但一握手就 abort, 只能靠串口救)。装一个 oneshot 服务强制生成。
sudo tee $R/etc/systemd/system/regen-sshkeys.service >/dev/null <<'UNIT'
[Unit]
Description=Regenerate SSH host keys on first boot
Before=ssh.service
ConditionPathExists=!/etc/ssh/ssh_host_ed25519_key
[Service]
Type=oneshot
ExecStart=/usr/bin/ssh-keygen -A
ExecStartPost=/bin/systemctl try-restart ssh.service
[Install]
WantedBy=multi-user.target
UNIT
sudo ln -sf /etc/systemd/system/regen-sshkeys.service \
    $R/etc/systemd/system/multi-user.target.wants/regen-sshkeys.service
sudo rm -f $R/home/uisrc/*.log $R/home/uisrc/.bash_history $R/root/.bash_history
sudo rm -rf $R/home/uisrc/rfsbak $R/var/lib/dhcp/*
sudo rm -f $R/etc/udev/rules.d/70-persistent-net.rules  # 否则网卡名钉在旧机器 MAC 上

echo "=== 6/6 校验 + 压缩 ==="
sudo md5sum /mnt/_img_boot/BOOT.BIN /mnt/_img_boot/uImage /mnt/_img_boot/devicetree.dtb
sudo sync
sudo umount /mnt/_img_boot /mnt/_img_root
sudo e2fsck -fp "${LOOP}p2" || true
sudo losetup -d "$LOOP"; LOOP=""
gzip -1 -c "$IMG" > "$OUT"
echo
echo "完成: $OUT  ($(du -h "$OUT" | cut -f1))"
echo "md5:  $(md5sum "$OUT" | cut -d' ' -f1)"
echo
echo "烧录: balenaEtcher → Flash from file 选它 → 选卡 → Flash"
echo "      (Etcher 直接认 .gz; 烧完 Windows 弹\"需要格式化\"点取消, 那是 ext4 分区)"
