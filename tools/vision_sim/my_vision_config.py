"""镜像 project/code/Camera/vision_config.h —— 视觉参数（保持与片上一致）。

改这里即可在 PC 上批量调参；调好后把数值抄回 vision_config.h。
注意：只收「需要调」的视觉参数，派生结构常量（BORDER_MAX/USE_NUM/SCAN_TOP 等）与 C 端一样留在各自模块。
"""
import numpy as np

# 图像尺寸（与 zf_device_mt9v03x.h 一致；C 端此处为降分辨率 120x60）
MT9V03X_W = 120
MT9V03X_H = 60

# 派生的结构常量（与 camera.h 保持一致）
WHITE_PIXEL = 255
BLACK_PIXEL = 0
BORDER_MAX = MT9V03X_W - 2
BORDER_MIN = 1
USE_NUM = MT9V03X_H * 3

# -------------------- 二值化 / 灰度 --------------------
VISION_BIN_THRESHOLD = 170  # 二值化阈值：像素灰度>此值判为白

# -------------------- 赛道边线补线 --------------------
BORDER_GAP_MAX = 3  # 短缺口允许跳过的最大行数(<=此值才线性插值补线)

# -------------------- 控制采样行 / 误差加权 --------------------
TRACK_SAMPLE_OFFSET_NEAR = 14  # 主采样行：自屏底向上偏移14行(即 MT9V03X_H-14)
TRACK_SAMPLE_OFFSET_FAR = 8    # 副采样行：自屏底向上偏移8行 (即 MT9V03X_H-8)
TRACK_ERROR_W_NEAR = 0.6       # 主采样行误差权重(error1)
TRACK_ERROR_W_FAR = 0.4        # 副采样行误差权重(error2)

# -------------------- 提边搜索 --------------------
EDGE_START_ROW_OFF = 2         # 提边起始行：自屏底向上偏移2行
EDGE_MIN_ROW_DIV = 3           # 提边最小行：取 MT9V03X_H/3 之上才开始扫描
LOST_TO_SEARCH_THRESH = 5      # 连续丢线>此值 -> 退回 SEARCH_MODE
