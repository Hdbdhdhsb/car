"""硬件拓扑还原：把 .epro2 网表 + LQFP100 引脚表合成「端口 -> 网络」映射。

输入
  pinmap_lqfp100.txt   LQFP100 物理脚 -> 端口名（由核心板原理图 PDF 抽出）
  ProPrj_*.epro2       嘉立创工程（ZIP + 行分隔 JSON 文档流）

输出
  topology_u16.txt     U16(STC32G144K246) 逐脚: 物理脚 / 端口 / 网络 / 对端器件
"""
import json, os, re, sys, zipfile, collections

EPRO2 = r"C:\Users\16275\Desktop\ProPrj_智能车最终版_2026-09-21.epro2"
HERE = os.path.dirname(os.path.abspath(__file__))
PINMAP = os.path.join(HERE, "pinmap_lqfp100.txt")


# ---------- .epru 解析（格式见知识库代码片段） ----------
def load_records(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        raw = f.read()
    recs, fail = [], 0
    for i, line in enumerate(raw.split("\n")):
        s = line.strip()
        if not s or not s.endswith("|"):
            if s:
                fail += 1
            continue
        core = s[:-1]
        p = core.find("||")
        if p < 0:
            fail += 1
            continue
        try:
            head = json.loads(core[:p])
        except Exception:
            fail += 1
            continue
        body_s = core[p + 2:]
        if not body_s:
            body = None
        else:
            try:
                body = json.loads(body_s)
            except Exception:
                fail += 1
                continue
        recs.append((i, head, body))
    return recs, fail


def extract_epro2():
    """解出内层 .epru 到临时目录，返回路径。"""
    import tempfile
    out = os.path.join(tempfile.gettempdir(), "geesetrace_hw")
    os.makedirs(out, exist_ok=True)
    with zipfile.ZipFile(EPRO2) as z:
        epru = [n for n in z.namelist() if n.endswith(".epru")][0]
        dst = os.path.join(out, os.path.basename(epru))
        if not os.path.exists(dst) or os.path.getsize(dst) != z.getinfo(epru).file_size:
            z.extract(epru, out)
            dst = os.path.join(out, epru)
    return dst


def main():
    pin2port = {}
    with open(PINMAP, encoding="utf-8") as f:
        for line in f:
            a, b = line.rstrip("\n").split("\t")
            pin2port[int(a)] = b

    recs, _ = load_records(extract_epro2())
    starts = [i for i, (_, _, b) in enumerate(recs)
              if isinstance(b, dict) and "docType" in b] + [len(recs)]
    docs = [{"head": recs[starts[k]][2], "block": recs[starts[k]:starts[k + 1]]}
            for k in range(len(starts) - 1)]
    pcb = next(d for d in docs if d["head"].get("docType") == "PCB")["block"]

    attrs = collections.defaultdict(dict)
    for _, h, b in pcb:
        if h.get("type") == "ATTR" and b and b.get("parentId"):
            attrs[b["parentId"]][b.get("key")] = b.get("value")
    desig = {cid: a.get("Designator") for cid, a in attrs.items()}

    pinmap = collections.defaultdict(dict)
    for _, h, b in pcb:
        if h.get("type") == "PAD_NET" and b:
            arr = json.loads(h["id"])
            pinmap[arr[1]][arr[2]] = b.get("padNet")

    net2pin = collections.defaultdict(list)
    for cid, pins in pinmap.items():
        for p, net in pins.items():
            if net:                                   # 空串 = 未分配网络
                net2pin[net].append(f"{desig.get(cid, '?')}.{p}")

    by_des = {d: cid for cid, d in desig.items() if d}
    u16 = by_des["U16"]
    pins = pinmap[u16]

    def pk(x):
        return (0, int(x)) if x.isdigit() else (1, x)

    out = []
    out.append(f"{'脚':>4}  {'端口':<14} {'网络':<20} 对端")
    out.append("-" * 100)
    for p in sorted(pins, key=pk):
        try:
            pn = int(p)
        except ValueError:
            pn = None
        port = pin2port.get(pn, "") if pn else ""
        net = pins[p] or "(未分配)"
        peers = [x for x in net2pin.get(net, []) if not x.startswith("U16.")]
        out.append(f"{p:>4}  {port:<14} {net:<20} {', '.join(sorted(peers)[:5])}")

    txt = "\n".join(out)
    print(txt)
    with open(os.path.join(HERE, "topology_u16.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")

    # ---------- 自动校验：网络名若形如 P<x><y>，应与引脚表一致 ----------
    print("\n=== 校验：网络名 vs 引脚表端口 ===")
    ok = bad = 0
    for p in sorted(pins, key=pk):
        net = pins[p] or ""
        m = re.fullmatch(r"P(\d)([0-7])", net)
        if not m or not p.isdigit():
            continue
        expect = pin2port.get(int(p), "")
        want = f"P{m.group(1)}.{m.group(2)}"
        if expect.startswith(want):
            ok += 1
        else:
            bad += 1
            print(f"  ✗ U16.{p} 网络名 {net} 但引脚表说是 {expect}")
    print(f"  一致 {ok} 条, 不一致 {bad} 条")


if __name__ == "__main__":
    main()
