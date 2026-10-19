# tools/hw — 硬件拓扑还原工具

不装嘉立创EDA专业版，直接从工程文件 + 官方 PDF 还原板子的引脚与网络。

产物见 [`../../docs/硬件拓扑.md`](../../docs/硬件拓扑.md)。

## 脚本

| 脚本 | 作用 |
|---|---|
| `extract_pdf.py` | 通用 PDF 文字层抽取（带坐标），用于原理图/芯片手册 |
| `build_pinmap.py` | 逐飞核心板原理图 PDF → **LQFP100 物理脚 → 端口名**（`pinmap_lqfp100.txt`） |
| `build_pin_functions.py` | STC 芯片手册 → **端口 → 全部复用功能**（`pin_functions.txt`） |
| `build_topology.py` | `.epro2` 网表 × 引脚表 → **U16 逐脚 端口/网络/对端**（`topology_u16.txt`），并自动校验网络名与引脚表是否一致 |

## 用法

```bash
pip install pypdf            # 唯一外部依赖

python extract_pdf.py "<核心板原理图>.pdf" pinout_raw.txt
python build_pinmap.py                                   # 需要 pinout_raw.txt

python extract_pdf.py "<芯片手册>.pdf" manual_raw.txt
python build_pin_functions.py                            # 需要 manual_raw.txt

python build_topology.py                                 # 需要 pinmap_lqfp100.txt + .epro2
```

`build_topology.py` 里的 `.epro2` 路径写死为桌面那份工程文件，
换工程时改 `EPRO2` 常量即可。

## 三个数据源，互为校验

1. **逐飞核心板原理图 PDF** → LQFP100 引脚-端口对照（图纸上每个引脚都标了端口名+脚号）
2. **STC 官方芯片手册 PDF** → 复用功能表（按端口名索引，不依赖手册排版顺序）
3. **`.epro2` 工程网表** → BOM、网表、板框、实际连线

`build_topology.py` 会自动拿 3 的「网络名」去校验 1 的「引脚表」——
网络名形如 `P40`/`P65` 的必须与引脚表一致，当前 **12/12 通过**。

## .epro2 / .epru 格式要点

```
.epro2 = ZIP
  ├── project2.json          元数据
  ├── <工程名>.epru          行分隔 JSON 文档流： <head>||<body>|\n
  └── IMAGE/<uuid>.webp      各页预览图（文件名 = 文档 uuid）
```

- 行尾单个 `|` 是**记录终止符**；`<head>|||`（空 body）是墓碑/删除标记。
- 文档以带 `docType` 的 `DOCHEAD` 开头，紧跟一条 `META`（含页名）。
- **坐标单位是 mil**（×0.0254=mm），用已知封装反推验证。
- 器件旋转字段是 `angle` 不是 `rotation`。
- 同一 `id` 的多条记录是**增量补丁**，按「首次非 null 优先」合并。
- `padNet == ""` 是**未分配网络**，做网表分组时必须剔除。
- **网表**：`PAD_NET` 的 `head.id = ["PAD_NET", 器件实例id, 焊盘号, 焊盘元素id]`，
  `body.padNet` 即网络名；器件位号查 `ATTR` 的 `Designator`（`parentId` 关联）。
- **板框**：`layerId == OUTLINE` 上的 `POLY`，`polyType == "BOARD_OUTLINE"`，
  `path = ["R", x, y, w, h, angle, r]`，圆角矩形，单位 mil。

完整说明另见个人知识库 `03-Resources/代码片段/嘉立创EDA专业版工程文件解析.md`。

## 已知限制

- 只读解析，不回写 `.epru`（回写要维护 `ticket`/`firstTicket` 版本链，风险高）。
- `build_pin_functions.py` 的输出有少量噪声（手册里混着「打线短接」的说明文字），
  读表时以 `PWMA*/PWMB*/SPI*/UART*/RxD*` 这类规范标识为准。
- U1（STC32G12K128-QFN48）的端口名尚未还原——资料里没有该封装的引脚表。
