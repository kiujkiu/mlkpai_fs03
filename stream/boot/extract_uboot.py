#!/usr/bin/env python3
"""Extract the u-boot partition (raw binary) from the factory Zynq BOOT.bin.

The factory MLKPAI-FS03 BOOT.bin (from the vendor restore image
rst_to_factory_img.tar.gz, sdcard_image/boot/BOOT.bin, U-Boot 2019.01
built 2024-11-08) contains 3 partitions:
    part0: zynq_fsbl.elf   (attr 0x10, PS exec)
    part1: system.bit      (attr 0x20, PL bitstream)
    part2: u-boot.elf      (attr 0x10, load=0x4000000 exec=0x4000000)

bootgen cannot unpack a BOOT.BIN, but the Zynq-7000 boot image format
(UG585 ch.6 / UG821) is simple: the words at 0x98/0x9C point to the
image header table / partition header table; each partition header is
16 little-endian words:
    [0] encrypted data word length   [1] unencrypted data word length
    [2] total partition word length  [3] load address
    [4] execution address            [5] partition data word offset
    [6] attributes                   [7] section count ...

The u-boot partition is stored as the raw loadable image (bootgen already
flattened the ELF), so we re-inject it into our new BOOT.BIN as a .bin
partition with [load = 0x4000000, startup = 0x4000000].

Usage: python3 extract_uboot.py <factory_BOOT.bin> <out_u-boot.bin>
"""
import struct
import sys


def parse_partitions(data: bytes):
    pht_off = struct.unpack("<I", data[0x9C:0xA0])[0]
    parts = []
    off = pht_off
    while off + 64 <= len(data):
        w = struct.unpack("<16I", data[off:off + 64])
        if w[0] == 0 or all(x == 0xFFFFFFFF for x in w[:4]):
            break
        parts.append({
            "len": w[1] * 4,          # unencrypted data length (bytes)
            "load": w[3],
            "exec": w[4],
            "data_off": w[5] * 4,
            "attr": w[6],
        })
        off += 64
        if len(parts) > 16:
            break
    return parts


def main():
    src, dst = sys.argv[1], sys.argv[2]
    data = open(src, "rb").read()
    parts = parse_partitions(data)
    for i, p in enumerate(parts):
        print(f"part{i}: data@{p['data_off']:#x} len={p['len']:#x} "
              f"load={p['load']:#x} exec={p['exec']:#x} attr={p['attr']:#x}")
    # u-boot = last PS partition, load/exec at 0x4000000
    uboot = [p for p in parts if p["load"] == 0x4000000 and p["attr"] & 0x10]
    assert len(uboot) == 1, f"expected exactly 1 u-boot partition, got {uboot}"
    p = uboot[0]
    blob = data[p["data_off"]:p["data_off"] + p["len"]]
    open(dst, "wb").write(blob)
    print(f"wrote {dst}: {len(blob)} bytes (load/startup = {p['load']:#x})")


if __name__ == "__main__":
    main()
