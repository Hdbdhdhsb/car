"""通用 PDF 文字层抽取（带坐标），用于芯片手册/原理图。

用法: python extract_pdf.py <pdf> <out.txt>
"""
import sys
from pypdf import PdfReader

PDF = sys.argv[1]
DST = sys.argv[2] if len(sys.argv) > 2 else "pdf_text.txt"

reader = PdfReader(PDF)
print(f"{PDF}\npages: {len(reader.pages)}")

out = []
for pi, page in enumerate(reader.pages):
    parts = []

    def visit(text, cm, tm, font_dict, font_size, _p=parts):
        t = text.strip()
        if t:
            _p.append((round(tm[4], 1), round(tm[5], 1), t))

    try:
        page.extract_text(visitor_text=visit)
    except Exception as e:
        print(f"  page {pi+1} extract failed: {e}")
        continue
    for x, y, t in parts:
        out.append(f"{pi}\t{x}\t{y}\t{t}")

with open(DST, "w", encoding="utf-8") as f:
    f.write("\n".join(out))
print(f"wrote {DST}: {len(out)} rows")
