#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成可打印的 ArUco 标定板（默认 DICT_4X4_50 / id 0 / 黑色外框 100 mm）。

用法：
    python3 scripts/make_aruco.py                       # 默认 4x4_50 / id 0 / 100mm / 300dpi
    python3 scripts/make_aruco.py --id 3 --mm 80 --dpi 600
    python3 scripts/make_aruco.py --dict DICT_5X5_50 --id 1 --out ~/aruco.png

打印要求：
1. 用图像查看器/浏览器打开 PNG，选“实际大小 / 100%”（别选“适应页面”）。
2. 打印后拿卡尺量黑色方块的**外边长**，把实测值填给网页：--marker-mm <实测毫米>。
3. 贴到一块硬纸板/亚克力上再放桌面——纸卷边会让 PnP 偏；表面别反光。
4. 白色边框至少留 20 mm，四周不要有其他深色图案。
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def build_marker(dict_name: str, marker_id: int, side_px: int) -> np.ndarray:
    d_id = getattr(cv2.aruco, dict_name, None)
    if d_id is None:
        sys.exit(f"未知字典 {dict_name}，例如 DICT_4X4_50 / DICT_5X5_100 / DICT_6X6_250")
    d = cv2.aruco.getPredefinedDictionary(d_id)
    img = cv2.aruco.generateImageMarker(d, int(marker_id), int(side_px))
    return img


def main():
    ap = argparse.ArgumentParser(description="生成可打印 ArUco 标定板")
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--mm", type=float, default=100.0, help="黑色方块外边长（毫米）")
    ap.add_argument("--margin-mm", type=float, default=25.0, help="四周白边（毫米）")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", default="", help="输出 PNG 路径")
    args = ap.parse_args()

    px_per_mm = args.dpi / 25.4
    side = int(round(args.mm * px_per_mm))
    margin = int(round(args.margin_mm * px_per_mm))
    marker = build_marker(args.dict, args.id, side)

    # 白色纸面 + 标定板
    paper = np.full((side + 2 * margin, side + 2 * margin), 255, np.uint8)
    paper[margin:margin + side, margin:margin + side] = marker

    # 底部加一条说明（英文，避免字体缺失）；不影响黑块测量
    label_h = int(round(12 * px_per_mm))
    canvas = np.full((paper.shape[0] + label_h, paper.shape[1], 3), 255, np.uint8)
    canvas[:paper.shape[0]] = cv2.cvtColor(paper, cv2.COLOR_GRAY2BGR)
    text = f"ArUco {args.dict}  id={args.id}  black square = {args.mm:.1f} mm  print at 100%"
    cv2.putText(canvas, text, (margin, paper.shape[0] + int(label_h * 0.7)),
                cv2.FONT_HERSHEY_SIMPLEX, args.mm / 420.0, (0, 0, 0), max(1, int(round(px_per_mm / 6))),
                cv2.LINE_AA)

    out = Path(args.out) if args.out else (
        Path.home() / ".robonix" / "handeye_board" /
        f"aruco_{args.dict.lower()}_id{args.id}_{args.mm:.0f}mm_{args.dpi}dpi.png")
    out.parent.mkdir(parents=True, exist_ok=True)

    from PIL import Image
    Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)).save(out, dpi=(args.dpi, args.dpi))
    print(f"已生成: {out}")
    print(f"  纸面 {canvas.shape[1] / px_per_mm:.0f} x {canvas.shape[0] / px_per_mm:.0f} mm，"
          f"黑块 {args.mm:.1f} mm，{args.dpi} dpi")
    print("打印后请用卡尺量黑块外边长，把实测值用 --marker-mm 传给网页")


if __name__ == "__main__":
    main()
