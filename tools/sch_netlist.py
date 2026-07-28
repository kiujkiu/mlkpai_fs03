#!/usr/bin/env python3
"""从 Altium 导出的 PDF 原理图里抽网表 (PI<designator><pin> / NL<netname> 图层)。

Altium 打印 PDF 时会把每个引脚的连接点写成不可见文本 `PI<位号><pin>`，
紧跟其后的 `NL<网名>` 是这一簇引脚所属的网。据此可无损还原 pin↔net。

用 Windows Python (装了 PyMuPDF) 跑:
  C:\\Users\\<u>\\AppData\\Local\\Programs\\Python\\Python312\\python.exe \\
      tools/sch_netlist.py "D:\\path\\to\\sheet.pdf" [--only J1,P1]

--only 只打某几个位号的 pin (常用: 只看两个连接器就得到端到端映射表)。
"""
import re
import sys
import io

import fitz  # PyMuPDF

PIN_RE = re.compile(r"^PI([A-Z]+\d+)0*?(\d+)$")


def split_pin(tok, designators=None):
    """PIJ1034 -> ('J1', 34)。位号与 pin 号之间无分隔符，靠已知位号集合消歧。"""
    body = tok[2:]
    if designators:
        for d in sorted(designators, key=len, reverse=True):
            if body.startswith(d) and body[len(d):].isdigit():
                return d, int(body[len(d):])
        return None
    m = re.match(r"^([A-Z]+)(\d+)$", body)
    if not m:
        return None
    letters, digits = m.groups()
    # 无位号表时的兜底: 假定位号数字只有 1 位 (J1/P1/U2...)
    return letters + digits[0], int(digits[1:] or 0)


def parse(path):
    doc = fitz.open(path)
    designators, nets = set(), []
    for page in doc:
        toks = [t.strip() for t in page.get_text().split("\n") if t.strip()]
        for t in toks:
            for w in t.split():
                if w.startswith("CO"):
                    designators.add(w[2:])
        cur = []
        for t in toks:
            for w in t.split():
                if w.startswith("PI"):
                    cur.append(w)
                elif w.startswith("NL"):
                    if cur:
                        nets.append((w[2:], cur))
                    cur = []
                elif w.startswith("CO"):
                    cur = []
    return designators, nets


def main():
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    path = sys.argv[1]
    only = None
    if "--only" in sys.argv:
        only = set(sys.argv[sys.argv.index("--only") + 1].split(","))
    tail = None
    if "--tail" in sys.argv:
        tail = int(sys.argv[sys.argv.index("--tail") + 1])
    designators, nets = parse(path)
    for net, pins in nets:
        # ⚠ 电源网 (GND/VCC) 走 power port, PDF 里没有 NL 标号, 其引脚簇会被并到
        # 下一条网前面。原始顺序下本网自己的引脚永远在末尾 → --tail N 截尾即可。
        if tail:
            pins = pins[-tail:]
        entries, seen = [], set()
        for p in pins:
            sp = split_pin(p, designators)
            if sp and sp not in seen and (only is None or sp[0] in only):
                seen.add(sp)
                entries.append("%s.%d" % sp)
        if entries:
            print("%-12s %s" % (net, "  ".join(entries)))


if __name__ == "__main__":
    main()
