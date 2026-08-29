"""合成赛道帧生成器：白场(255) + 黑色边线(0)，用于无车时走几何流水线。

生成的是「符合片上算法预期」的图：走廊为白，两侧黑边线各占 ~2px。
=直道/左右弯/十字/斑马线/圆环(粗略)。
真实赛道调参建议用 viewer --image 导入实际 MT9V03X 灰度帧。
"""
import math
import numpy as np
from my_vision_config import MT9V03X_W as W, MT9V03X_H as H


def _blank():
    return np.full((H, W), 255, np.uint8)


def _draw_corridor(img, left_edges, right_edges):
    """left_edges/right_edges: 长度 H 的 list，每行给左边线内缘白色起点 / 右边线内缘白色终点。
    在该行画 2px 黑边线：左线占 [left_edges-2, left_edges-1]，右线占 [right_edges+1, right_edges+2]。
    """
    for r in range(H):
        le, re = left_edges[r], right_edges[r]
        le = max(2, min(W - 4, le))
        re = max(2, min(W - 4, re))
        if re - le < 8:          # 保底最小走廊，防止线交叉
            re = le + 8
            if re > W - 3:
                re = W - 3
                le = re - 8
        img[r][le - 2:le] = 0
        img[r][re + 1:re + 3] = 0
    return img


def straight():
    img = _blank()
    le = [46] * H
    re = [74] * H
    return _draw_corridor(img, le, re)


def left_curve():
    img = _blank()
    le, re = [], []
    for r in range(H):
        base = 24.0 * math.sin((H - r) / H * 1.8)   # 靠底越大 -> 向左弯
        le.append(int(46 - base))
        re.append(int(74 - base))
    return _draw_corridor(img, le, re)


def right_curve():
    img = _blank()
    le, re = [], []
    for r in range(H):
        base = 24.0 * math.sin((H - r) / H * 1.8)
        le.append(int(46 + base))
        re.append(int(74 + base))
    return _draw_corridor(img, le, re)


def cross():
    """十字：底部边线向外张开，产生「过宽」路段。"""
    img = _blank()
    le, re = [], []
    for r in range(H):
        # H/2 以下向外张开（line 变宽），以上恢复正常
        if r > H // 2:
            k = (r - H // 2) / (H // 2)
            spread = int(30 * k)
            le.append(46 - spread)
            re.append(74 + spread)
        else:
            le.append(46)
            re.append(74)
    return _draw_corridor(img, le, re)


def zebra():
    """斑马线：直道走廊 + 底部若干行布满黑白交替横条。"""
    img = straight()
    # 在底部附近画黑白相间横条
    for r in range(H - 6, H - 1):
        seg = 4
        for c in range(46, 74, seg * 2):
            img[r][c:c + seg] = 0
    return img


def circle():
    """圆环（粗略）：左边界直、右边界向外弯成大弧，模拟环岛入口一侧张开。"""
    img = _blank()
    le, re = [], []
    cx = 82        # 圆心 x
    cy = 10        # 圆心 y
    r0 = 78        # 大弧半径
    for row in range(H):
        left_col = 46
        # 右边界：离圆心近的行向内收，远则外张（一个环）
        dy = cy - row
        d = math.sqrt(r0 * r0 - dy * dy) if abs(dy) < r0 else r0 * 0.6
        right_col = int(cx - d * 0.0 + d)   # 右线在圆心右侧一圈
        # 简化：右线整体右移模拟入口
        right_col = int(74 + 12 * (1 - abs(row - cy) / max(1, cy + 1)))
        le.append(left_col)
        re.append(max(le[-1] + 8, right_col))
    return _draw_corridor(img, le, re)


GENERATORS = {
    "straight": straight,
    "left_curve": left_curve,
    "right_curve": right_curve,
    "cross": cross,
    "zebra": zebra,
    "circle": circle,
}
