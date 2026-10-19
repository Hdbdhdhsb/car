"""从 STC32G144K246 官方手册抽出「端口 -> 全部复用功能」表。

手册管脚图那一段是这样一个文本块（每个引脚一行，顺序与封装位号未必一致，
所以按端口名建索引，不依赖行序）：
    TXD4_2/CMP3+_2/LIN_TX_2/CAN2_TX_2/P5.3
    OPA1+_2/OPA1-_1/PWMA_E2_2/SCLK2_2/T3CLKO/ADCB5/AD5/P0.5
    P9.7/
    UCap       (P1.2/ADC2/ADC_ETR/ADCB_ETR/T2/SS/PWMA2P/QSPI1_NCS)
"""
import re, sys, json, collections

SRC = sys.argv[1] if len(sys.argv) > 1 else "manual_raw.txt"
PORT = re.compile(r"^(P[0-9A-B]\.\d)$")

funcs = collections.defaultdict(set)     # port -> {功能}
raw_line = {}

with open(SRC, encoding="utf-8") as f:
    for line in f:
        txt = line.rstrip("\n").split("\t")[-1].strip()
        if "/" not in txt and "(" not in txt:
            continue
        # 括号里可能还藏着一个端口（如 UCap 行）
        for blob in re.findall(r"\(([^)]*)\)", txt) or [""]:
            pass
        cand = txt
        toks = []
        for part in re.split(r"[/()\s]+", cand):
            part = part.strip()
            if not part:
                continue
            m = PORT.match(part)
            if m:
                funcs[m.group(1)].add("(自身)")
                raw_line.setdefault(m.group(1), txt)
            elif re.fullmatch(r"[A-Za-z][\w+\-_.]{0,24}", part):
                toks.append(part)
        # 该行涉及的端口：第一个命中的
        hit = [p for p in re.split(r"[/()\s]+", cand) if PORT.match(p or "")]
        for h in hit:
            for t in toks:
                funcs[h].add(t)
            raw_line.setdefault(h, txt)

for p in sorted(funcs):
    funcs[p].discard("(自身)")

print(f"覆盖端口 {len(funcs)} 个\n")
out = []
for p in sorted(funcs, key=lambda s: (s[0], s[1], int(s.split('.')[1]))):
    fs = sorted(funcs[p])
    line = f"{p:<7} {' '.join(fs)}"
    print(line)
    out.append(line)

with open("pin_functions.txt", "w", encoding="utf-8") as f:
    f.write("\n".join(out) + "\n")
print("\nwrote pin_functions.txt")
