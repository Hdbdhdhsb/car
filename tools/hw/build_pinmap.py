"""把逐飞核心板原理图 PDF 的文字层还原成 LQFP100 完整引脚表。

原理图 MCU 符号排版（坐标由 pinout_raw.txt 实测）：
  左列:  脚号 x≈108  <-- 端口名 x=130.4
  右列:                     端口名 x≈198  -->  脚号 x=227.5  -->  短标签 x=239
脚号相对端口名的 y 偏移恒为 -4.8pt，据此精确配对（不靠"最近邻"猜）。
"""
import re, sys, collections

SRC = sys.argv[1] if len(sys.argv) > 1 else "pinout_raw.txt"

rows = []
with open(SRC, encoding="utf-8") as f:
    for line in f:
        p = line.rstrip("\n").split("\t")
        if len(p) == 4:
            rows.append((int(p[0]), float(p[1]), float(p[2]), p[3]))
rows = [r for r in rows if r[0] == 1]                 # 原理图页

DY = -4.8                                             # 脚号 y 偏移
name_l = [r for r in rows if 125 < r[1] < 136]        # 左列端口名
name_r = [r for r in rows if 195 < r[1] < 205]        # 右列端口名
nums = [r for r in rows if re.fullmatch(r"\d{1,3}", r[3])]

print(f"左列名 {len(name_l)} / 右列名 {len(name_r)} / 候选脚号 {len(nums)}")

table = {}
unmatched = []
for _, nx, ny, ntext in name_l + name_r:
    cands = [n for n in nums if abs(n[2] - (ny + DY)) < 2.0 and abs(n[1] - nx) < 40]
    if len(cands) == 1:
        table[int(cands[0][3])] = ntext
    else:
        unmatched.append((ntext, ny, len(cands)))

print(f"配对 {len(table)} 个，未配对 {len(unmatched)}")
for u in unmatched:
    print("   未配对:", u)

print(f"\n{'脚':>4}  {'端口':<16}")
print("-" * 26)
for pin in range(1, 101):
    print(f"{pin:>4}  {table.get(pin, '<未标注>'):<16}")

missing = [p for p in range(1, 101) if p not in table]
print(f"\n缺失脚 ({len(missing)}): {missing}")

with open("pinmap_lqfp100.txt", "w", encoding="utf-8") as f:
    for pin in sorted(table):
        f.write(f"{pin}\t{table[pin]}\n")
print("wrote pinmap_lqfp100.txt")
