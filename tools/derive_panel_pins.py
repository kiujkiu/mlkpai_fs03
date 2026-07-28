#!/usr/bin/env python3
"""由三块板原理图网表推导 FPGA ball <-> 屏信号, 并与 XDC 对账。

链路:  FPGA ball --FS03 J12--> 接口板 P2 --10R--> P1/P3 --转接板--> 屏 J1

三段映射的来源与可信度:
  1. ball <-> J12 pin        : docs/led_panel_chain.md (CEP2_kP/N = J12 2k+1/2k+2)
  2. J12 pin <-> P1/P3 pin   : mlp_panel_v1.0.pdf 网表, 规律 P1=30-J12 / P3=50-J12
  3. P1 pin <-> 屏 J1 pin    : 转接板网表 (v1.1 = panel_0.93cob_trans.pdf,
                               v1.2 = LKS_FOC.pdf) —— **本次唯一变更的一段**

自检: 用 v1.1 网表推导必须逐 pin 复现 panel_pins_trans_v11.xdc。复现得上,
      才说明方法可信, v1.2 的推导结果才能拿去烧板。

跑法:  python3 tools/derive_panel_pins.py
"""

# ---- 1. ball <-> J12 pin (FS03 侧, 未变) ----
J12_BALL = {
    4: "W12", 5: "AA12", 6: "AB12", 7: "Y9", 8: "Y8", 9: "AA11", 10: "AB11",
    11: "Y11", 12: "Y10", 13: "AB10", 14: "AB9", 15: "AA9", 16: "AA8",
    17: "AA7", 18: "AA6", 19: "Y6", 20: "Y5", 21: "AB5", 22: "AB4",
    # 屏2 (P3 组)
    24: "W5", 25: "Y4", 26: "AA4", 27: "V5", 28: "V4", 29: "T4", 30: "U4",
    31: "AB7", 32: "AB6", 33: "AB2", 34: "AB1", 35: "U6", 36: "U5", 37: "R6",
    38: "T6", 39: "V7", 40: "W7", 41: "V8", 42: "W8",
}

# ---- 3. 转接板 P1 pin -> 屏 J1 pin (tools/sch_netlist.py 抽的) ----
TRANS = {
    # net:        (v1.1 P1, v1.2 P1, 屏 J1)   —— J1 两版完全一致
    "DCLK":      (8,  18, 25),
    "LAT":       (9,  15, 26),
    "GCLK":      (10, 16, 27),   # 线名 GCLK, 实为 2049/2047 OE
    "R1":        (24,  9, 32),
    "G1":        (20, 26,  8),
    "B1":        (23, 12, 31),
    "R2":        (19, 23,  9),
    "G2":        (22, 11, 30),
    "B2":        (18, 24, 10),
    "R3":        (12, 14, 29),
    "G3":        (17, 21, 11),
    "B3":        (11, 13, 28),
    "A":         (26,  8, 34),
    "B":         (21, 25,  7),
    "C":         (25, 10, 33),
    "SPI_CS":    (16, 22, 13),
    "SPI_MOSI":  (15, 19, 14),
    "SPI_CLK":   (14, 20, 15),
    "SPI_MISO":  (13, 17, 16),
}

# 屏信号 -> XDC 端口名 (SPI 四根未接, 无端口)
PORT = {
    "DCLK": "panel_dclk", "LAT": "panel_lat", "GCLK": "panel_oe",
    "R1": "panel_sdi[0]", "G1": "panel_sdi[1]", "B1": "panel_sdi[2]",
    "R2": "panel_sdi[3]", "G2": "panel_sdi[4]", "B2": "panel_sdi[5]",
    "R3": "panel_sdi[6]", "G3": "panel_sdi[7]", "B3": "panel_sdi[8]",
    "A": "panel_row_dclk", "B": "panel_row_rclk", "C": "panel_row_sdi",
}


def ball(p_pin, panel):
    """P1/P3 pin -> FPGA ball。接口板规律: P1 = 30 - J12, P3 = 50 - J12。"""
    j12 = (30 if panel == 1 else 50) - p_pin
    return J12_BALL[j12]


def derive(rev, panel):
    idx = 0 if rev == "v1.1" else 1
    return {net: ball(pins[idx], panel) for net, pins in TRANS.items()}


def read_xdc(path):
    import re
    out = {}
    for line in open(path, encoding="utf-8"):
        if not line.strip().startswith("set_property -dict"):
            continue
        m = re.search(r"PACKAGE_PIN\s+(\S+).*get_ports\s+\{?([\w\[\]]+)\}?\]", line)
        if m:
            out[m.group(2)] = m.group(1)
    return out


def main():
    ok = True
    for rev, xdc in (("v1.1", "vivado/panel_pins_trans_v11.xdc"),
                     ("v1.2", "vivado/panel_pins.xdc")):
        got = read_xdc(xdc)
        print("=== %s  vs  %s ===" % (rev, xdc))
        for panel, sfx in ((1, ""), (2, "_2")):
            want = derive(rev, panel)
            for net, port in PORT.items():
                p = port.replace("panel_", "panel_").replace("[", sfx + "[") if sfx and "[" in port else port + sfx
                exp, act = want[net], got.get(p)
                if exp != act:
                    ok = False
                    print("  ✗ 屏%d %-9s %-16s 推导=%-5s XDC=%s" % (panel, net, p, exp, act))
        print("  (无 ✗ 即逐 pin 吻合)")
    print("\n自检结果:", "PASS — 推导方法可复现旧 XDC, v1.2 结果可信" if ok else "FAIL")

    print("\n=== v1.2 相对 v1.1 的 ball 置换 (屏1) ===")
    o, n = derive("v1.1", 1), derive("v1.2", 1)
    for net in TRANS:
        used = "" if net in PORT else "   (未接)"
        print("  %-9s %-5s -> %-5s   P1 %2d -> %2d, 屏J1.%d%s"
              % (net, o[net], n[net], TRANS[net][0], TRANS[net][1], TRANS[net][2], used))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
