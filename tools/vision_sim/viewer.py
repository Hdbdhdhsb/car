"""PC 侧视觉仿真查看器。

用法（在 tools/vision_sim 下，.venv 已建）：
  .venv/Scripts/python viewer.py --synthetic straight
  .venv/Scripts/python viewer.py --synthetic cross --threshold 120 --save out.png
  .venv/Scripts/python viewer.py --image real_frame.jpg        # 真实灰度帧(自动缩放到 120x60)

把片上算法在 PC 上复现，画出 左边界(红)/右边界(蓝)/中线(绿)，并打印 track_error。
调 VISION_BIN_THRESHOLD / BORDER_GAP_MAX 立即可见效果；调好后把数值抄回 vision_config.h。
"""
import argparse
import sys

import cv2
import numpy as np

import my_vision_config as cfg
import pipeline
import synthetic


def _overlay(gray_bgr, res):
    img = gray_bgr.copy()
    if not res["ok"]:
        cv2.putText(img, "NO BORDER", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        return img
    for r in range(cfg.MT9V03X_H):
        l = int(np.clip(res["l_border"][r], 0, cfg.MT9V03X_W - 1))
        rgt = int(np.clip(res["r_border"][r], 0, cfg.MT9V03X_W - 1))
        m = int(np.clip(res["midline"][r], 0, cfg.MT9V03X_W - 1))
        if res["l_valid"][r]:
            img[r][l] = (0, 0, 255)      # 左边界 红
        if res["r_valid"][r]:
            img[r][rgt] = (255, 0, 0)    # 右边界 蓝
        img[r][m] = (0, 255, 0)          # 中线 绿
    return img


def _load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"无法读取图像: {path}")
    if img.shape[0] != cfg.MT9V03X_H or img.shape[1] != cfg.MT9V03X_W:
        img = cv2.resize(img, (cfg.MT9V03X_W, cfg.MT9V03X_H), interpolation=cv2.INTER_AREA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", choices=list(synthetic.GENERATORS.keys()), help="用合成赛道帧")
    ap.add_argument("--image", help="真实灰度帧路径(自动缩放)")
    ap.add_argument("--threshold", type=int, help="临时覆盖 VISION_BIN_THRESHOLD")
    ap.add_argument("--save", help="保存 overlay 到路径")
    args = ap.parse_args()

    if args.threshold is not None:
        cfg.VISION_BIN_THRESHOLD = args.threshold

    if args.image:
        gray = _load_gray(args.image)
    elif args.synthetic:
        gray = synthetic.GENERATORS[args.synthetic]()
    else:
        ap.print_help()
        sys.exit("请指定 --synthetic 或 --image")

    res = pipeline.run_pipeline(gray)
    print(f"frame={args.image or args.synthetic}  threshold={cfg.VISION_BIN_THRESHOLD}")
    print(f"track_error = {res['track_error']:.3f}  (像素, 0=正中, 正=偏右)")
    if res["ok"]:
        print(f"  l_border sample rows: {[int(res['l_border'][r]) for r in (cfg.MT9V03X_H-14, cfg.MT9V03X_H-8)]}")
        print(f"  r_border sample rows: {[int(res['r_border'][r]) for r in (cfg.MT9V03X_H-14, cfg.MT9V03X_H-8)]}")
        print(f"  midline  sample rows: {[int(res['midline'][r]) for r in (cfg.MT9V03X_H-14, cfg.MT9V03X_H-8)]}")

    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    vis = _overlay(gray_bgr, res)
    # 放大 6 倍便于观察
    vis = cv2.resize(vis, (cfg.MT9V03X_W * 6, cfg.MT9V03X_H * 6), interpolation=cv2.INTER_NEAREST)
    if args.save:
        cv2.imwrite(args.save, vis)
        print(f"已保存 {args.save}")
    try:
        cv2.imshow("vision_sim (q 关闭)", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except cv2.error as e:          # 无显示环境：--save 已写出，跳过弹窗
        print(f"[无显示环境，跳过弹窗] {e}")


if __name__ == "__main__":
    main()
