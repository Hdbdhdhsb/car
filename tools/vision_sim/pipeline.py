"""逐行镜像项目/code/Camera/camera.c 的视觉流水线：
   二值化(turn_to_bin) -> 找起点(get_start_point) -> 八邻域提边(search_l_r)
   -> 写边界+插值(get_left/get_right) -> 中线(calc_midline) -> 误差(update_track_error)

纯 Python/numpy 复刻，用于 PC 上批量调 VISION_BIN_THRESHOLD / BORDER_GAP_MAX。
输入灰度帧应为 uint8 numpy 数组 (MT9V03X_H x MT9V03X_W)，像素>threshold 判白。
"""
import numpy as np
from my_vision_config import (
    MT9V03X_W as W, MT9V03X_H as H,
    WHITE_PIXEL as WHITE, BLACK_PIXEL as BLACK,
    BORDER_MAX, BORDER_MIN, USE_NUM,
    VISION_BIN_THRESHOLD, BORDER_GAP_MAX,
    TRACK_SAMPLE_OFFSET_NEAR, TRACK_SAMPLE_OFFSET_FAR,
    TRACK_ERROR_W_NEAR, TRACK_ERROR_W_FAR,
)


def turn_to_bin(gray):
    """void turn_to_bin(void)：像素灰度>阈值 -> 白，否则黑。"""
    return (gray > VISION_BIN_THRESHOLD).astype(np.uint8) * WHITE


def get_start_point(bin_img, start_row):
    """uint8 get_start_point(uint8 start_row) —— 返回 (found, x_l, y_l, x_r, y_r)。"""
    row = bin_img[start_row]
    x_l = y_l = x_r = y_r = 0
    l_found = r_found = 0
    for i in range(W // 2, BORDER_MIN, -1):
        if row[i] == WHITE and row[i - 1] == BLACK and row[i - 2] == BLACK:
            x_l, y_l, l_found = i, start_row, 1
            break
    for i in range(W // 2, BORDER_MAX):
        if row[i] == WHITE and row[i + 1] == BLACK and row[i + 2] == BLACK:
            x_r, y_r, r_found = i, start_row, 1
            break
    return (l_found and r_found), x_l, y_l, x_r, y_r


def _my_abs(v):
    return v if v >= 0 else -v


def search_l_r(image, l_start_x, l_start_y, r_start_x, r_start_y):
    """八邻域提边。返回 (points_l, points_r, dir_l, dir_r, l_ds, r_ds, hightest)。
    数组定长 USE_NUM，与 C 端一致。
    """
    seeds_l = [(0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]
    seeds_r = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]

    points_l = np.zeros((USE_NUM, 2), dtype=np.int32)
    points_r = np.zeros((USE_NUM, 2), dtype=np.int32)
    dir_l = np.zeros(USE_NUM, dtype=np.int32)
    dir_r = np.zeros(USE_NUM, dtype=np.int32)

    center_l = [l_start_x, l_start_y]
    center_r = [r_start_x, r_start_y]
    l_ds = 0
    r_ds = 0
    hightest = 0

    for _ in range(USE_NUM):
        # ---- 左边：先记录当前中心点 ----
        points_l[l_ds] = center_l
        l_ds += 1
        # ---- 右边：先记录当前中心点（未自增，索引用 r_ds） ----
        points_r[r_ds] = center_r

        # 邻居坐标
        field_l = [(center_l[0] + s[0], center_l[1] + s[1]) for s in seeds_l]
        field_r = [(center_r[0] + s[0], center_r[1] + s[1]) for s in seeds_r]

        # ---- 左环检测（黑->白过渡） ----
        index_l = 0
        best_x_l = best_y_l = 0
        for i in range(8):
            ni = (i + 1) & 7
            x_cur, y_cur = field_l[i]
            x_nxt, y_nxt = field_l[ni]
            if not (0 <= x_cur < W and 0 <= y_cur < H and 0 <= x_nxt < W and 0 <= y_nxt < H):
                continue
            if image[y_cur][x_cur] == BLACK and image[y_nxt][x_nxt] == WHITE:
                if index_l == 0 or best_y_l > y_cur:
                    best_x_l, best_y_l = x_cur, y_cur
                index_l += 1
                dir_l[l_ds - 1] = i
        if index_l:
            center_l = [best_x_l, best_y_l]

        # ---- 终止条件 ----
        if (r_ds >= 2 and l_ds >= 3):
            pr = points_r[r_ds]
            pl = points_l[l_ds - 1]
            if ((pr[0] == points_r[r_ds - 1][0] and pr[0] == points_r[r_ds - 2][0]
                 and pr[1] == points_r[r_ds - 1][1] and pr[1] == points_r[r_ds - 2][1])
                    or (pl[0] == points_l[l_ds - 2][0] and pl[0] == points_l[l_ds - 3][0]
                        and pl[1] == points_l[l_ds - 2][1] and pl[1] == points_l[l_ds - 3][1])):
                break
        if (_my_abs(points_r[r_ds][0] - points_l[l_ds - 1][0]) < 2
                and _my_abs(points_r[r_ds][1] - points_l[l_ds - 1][1]) < 2):
            hightest = (points_r[r_ds][1] + points_l[l_ds - 1][1]) >> 1
            break
        if points_r[r_ds][1] < points_l[l_ds - 1][1]:
            continue
        if dir_l[l_ds - 1] == 7 and points_r[r_ds][1] > points_l[l_ds - 1][1]:
            center_l = [points_l[l_ds - 1][0], points_l[l_ds - 1][1]]
            l_ds -= 1

        r_ds += 1

        # ---- 右环检测 ----
        index_r = 0
        best_x_r = best_y_r = 0
        for i in range(8):
            ni = (i + 1) & 7
            x_cur, y_cur = field_r[i]
            x_nxt, y_nxt = field_r[ni]
            if not (0 <= x_cur < W and 0 <= y_cur < H and 0 <= x_nxt < W and 0 <= y_nxt < H):
                continue
            if image[y_cur][x_cur] == BLACK and image[y_nxt][x_nxt] == WHITE:
                if index_r == 0 or best_y_r > y_cur:
                    best_x_r, best_y_r = x_cur, y_cur
                index_r += 1
                dir_r[r_ds - 1] = i
        if index_r:
            center_r = [best_x_r, best_y_r]

    return points_l, points_r, dir_l, dir_r, l_ds, r_ds, hightest


def get_left(points_l, total_L):
    """void get_left(uint16 total_L)。返回 (l_border, l_index, l_border_valid)。"""
    l_border = np.full(H, BORDER_MIN, dtype=np.int32)
    l_index = np.zeros(H, dtype=np.int32)
    l_valid = np.zeros(H, dtype=np.uint8)
    for j in range(total_L):
        row = int(points_l[j][1])
        if row >= H:
            continue
        if l_valid[row] == 0:
            l_border[row] = int(points_l[j][0]) + 1
            l_index[row] = j
            l_valid[row] = 1
    _interpolate(l_border, l_valid)
    return l_border, l_index, l_valid


def get_right(points_r, total_R):
    """void get_right(uint16 total_R)。返回 (r_border, r_index, r_border_valid)。"""
    r_border = np.full(H, BORDER_MAX, dtype=np.int32)
    r_index = np.zeros(H, dtype=np.int32)
    r_valid = np.zeros(H, dtype=np.uint8)
    for j in range(total_R):
        row = int(points_r[j][1])
        if row >= H:
            continue
        if r_valid[row] == 0:
            r_border[row] = int(points_r[j][0]) - 1
            r_index[row] = j
            r_valid[row] = 1
    _interpolate(r_border, r_valid)
    return r_border, r_index, r_valid


def _interpolate(border, valid):
    """短缺口线性插值补线（get_left/get_right 共用）。原位修改 border/valid。"""
    last_valid = 0
    for i in range(H - 1, 0, -1):
        if valid[i]:
            last_valid = i
            break
    i = last_valid
    while i > 0:
        if valid[i]:
            last_valid = i
            i -= 1
            continue
        gap_start = i
        next_valid = 0
        found = 0
        for k in range(i, 0, -1):
            if valid[k]:
                next_valid = k
                found = 1
                break
        if not found:
            break
        gap_len = gap_start - next_valid
        if gap_len <= BORDER_GAP_MAX and last_valid > gap_start:
            span = last_valid - next_valid
            delta = int(border[last_valid]) - int(border[next_valid])
            base = int(border[next_valid])
            for k in range(next_valid + 1, last_valid):
                if not valid[k]:
                    step = k - next_valid
                    if delta >= 0:
                        val = base + int((delta * step + (span >> 1)) / span)
                    else:
                        val = base + int((delta * step - (span >> 1)) / span)
                    if val < BORDER_MIN:
                        val = BORDER_MIN
                    if val > BORDER_MAX:
                        val = BORDER_MAX
                    border[k] = val
                    valid[k] = 2
        i = next_valid


def calc_midline(l_border, r_border, l_valid, r_valid):
    """void calc_midline(void)：加权中线 + 三点均值平滑。返回 (midline, leftline, rightline)。"""
    last_good_mid = W // 2
    midline = np.zeros(H, dtype=np.int32)
    leftline = np.zeros(H, dtype=np.int32)
    rightline = np.zeros(H, dtype=np.int32)
    for i in range(H):
        if l_valid[i] and r_valid[i]:
            midline[i] = (l_border[i] + r_border[i]) // 2
            last_good_mid = midline[i]
        else:
            midline[i] = last_good_mid
        leftline[i] = l_border[i]
        rightline[i] = r_border[i]
    for i in range(1, H - 1):
        if midline[i] > 0 and midline[i - 1] > 0 and midline[i + 1] > 0:
            midline[i] = (midline[i - 1] + midline[i] + midline[i + 1]) // 3
    return midline, leftline, rightline


def update_track_error(midline):
    """void update_track_error(void)：加权采样误差。返回 float。"""
    sample_row = (H - TRACK_SAMPLE_OFFSET_NEAR) if H > TRACK_SAMPLE_OFFSET_NEAR else (H - 1)
    sample_row2 = (H - TRACK_SAMPLE_OFFSET_FAR) if H > TRACK_SAMPLE_OFFSET_FAR else (H - 1)
    screen_center = W // 2
    e1 = screen_center - midline[sample_row]
    e2 = screen_center - midline[sample_row2]
    return TRACK_ERROR_W_NEAR * e1 + TRACK_ERROR_W_FAR * e2


def extract_borders(bin_img):
    """uint8 extract_borders(void)：找起点 + 提边 + 写边界。返回 (ok, 各数组)。"""
    hightest = 0
    start_row = H - 2
    min_row = H // 3  # EDGE_MIN_ROW_DIV
    found = 0
    for sr in range(H - 2, min_row - 1, -1):
        ok, x_l, y_l, x_r, y_r = get_start_point(bin_img, sr)
        if ok:
            found = 1
            start_row = sr
            break
        if sr == 0:
            break
    if not found:
        return False, None
    points_l, points_r, _dl, _dr, l_ds, r_ds, hightest = search_l_r(
        bin_img, x_l, y_l, x_r, y_r)
    l_border, l_index, l_valid = get_left(points_l, l_ds)
    r_border, r_index, r_valid = get_right(points_r, r_ds)
    return True, (l_border, l_index, l_valid, r_border, r_index, r_valid)


def run_pipeline(gray):
    """主入口：灰度帧 -> 全部中间量。gray: np.uint8 (H,W)。"""
    bin_img = turn_to_bin(gray)
    ok, borders = extract_borders(bin_img)
    if not ok:
        return {"ok": False, "bin": bin_img, "track_error": 0.0}
    l_border, l_index, l_valid, r_border, r_index, r_valid = borders
    midline, leftline, rightline = calc_midline(l_border, r_border, l_valid, r_valid)
    err = update_track_error(midline)
    return {
        "ok": True, "bin": bin_img,
        "l_border": l_border, "r_border": r_border,
        "l_valid": l_valid, "r_valid": r_valid,
        "midline": midline, "leftline": leftline, "rightline": rightline,
        "track_error": err,
    }
