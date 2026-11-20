#!/usr/bin/env python3
"""把 ~/grasp_labels 里的手工标注导成 YOLOv8 需要的数据集目录。

输入（scripts/label_tool.py 写的）：
    ~/grasp_labels/images/<name>.jpg
    ~/grasp_labels/labels.json        {"<name>.jpg": [[cx, cy, w, h, cls], ...]}
    ~/grasp_labels/flags.json         {"<name>.jpg": "skip"|"hard"}（可选）

输出（默认 ~/grasp_labels/dataset）：
    images/{train,val}/<name>.jpg
    labels/{train,val}/<name>.txt     YOLO: "<cls> <cx> <cy> <w> <h>"
    data.yaml

规则：
  * 只有"有框"的图进 dataset（X 跳过 / 没画框的不进，当作不属于训练集）。
  * --background 打开后，被标为 skip 且没有框的图会作为负样本进训练集（空 txt）。
  * 类别号与 label_tool.py 的 CLASSES 一致：0 carrot, 1 potato, 2 block, 3 other。

用法：
    python3 scripts/yolo_dataset.py                     # 导出，默认 85/15 划分
    python3 scripts/yolo_dataset.py --background        # 跳过图作负样本
    python3 scripts/yolo_dataset.py --val-ratio 0.2
"""
from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import sys

CLASSES = ["carrot", "potato", "block", "other"]


def load_json(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh) or {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="~/grasp_labels", help="标注根目录")
    ap.add_argument("--out", default="", help="数据集输出目录（默认 <root>/dataset）")
    ap.add_argument("--val-ratio", type=float, default=0.15, help="验证集比例")
    ap.add_argument("--background", action="store_true", help="把无框的 skip 图当负样本放进训练集")
    ap.add_argument("--obb", action="store_true", help="输出 8 点多边形标签（YOLO-OBB 训练用）")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    root = os.path.expanduser(args.root)
    out = os.path.expanduser(args.out) if args.out else os.path.join(root, "dataset")
    images_dir = os.path.join(root, "images")
    labels = load_json(os.path.join(root, "labels.json"))
    flags = load_json(os.path.join(root, "flags.json"))

    if not os.path.isdir(images_dir):
        print(f"没有图片目录：{images_dir}", file=sys.stderr)
        return 2

    positives = [n for n, boxes in labels.items() if boxes and os.path.exists(os.path.join(images_dir, n))]
    negatives: list[str] = []
    if args.background:
        for name in os.listdir(images_dir):
            if name in labels and labels[name]:
                continue
            if flags.get(name) == "skip":
                negatives.append(name)

    if not positives:
        print("没有可用的正样本（labels.json 里没有带框的图）", file=sys.stderr)
        return 2

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    rng = random.Random(args.seed)
    rng.shuffle(positives)
    n_val = max(1, int(round(len(positives) * args.val_ratio))) if len(positives) > 1 else 0
    val = set(positives[:n_val])

    def emit(name: str, split: str, boxes: list) -> None:
        shutil.copy(os.path.join(images_dir, name), os.path.join(out, "images", split, name))
        stem = os.path.splitext(name)[0]
        with open(os.path.join(out, "labels", split, stem + ".txt"), "w", encoding="utf-8") as fh:
            for b in boxes:
                cx, cy, w, h, cls = b
                if args.obb:
                    x1, y1 = cx - w / 2.0, cy - h / 2.0
                    x2, y2 = cx + w / 2.0, cy + h / 2.0
                    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                    fh.write("%d " % int(cls) + " ".join("%.6f %.6f" % p for p in pts) + "\n")
                else:
                    fh.write("%d %.6f %.6f %.6f %.6f\n" % (int(cls), float(cx), float(cy), float(w), float(h)))

    for name in positives:
        emit(name, "val" if name in val else "train", labels[name])
    for name in negatives:
        emit(name, "train", [])

    yaml_path = os.path.join(out, "data_obb.yaml" if args.obb else "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as fh:
        fh.write(f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n")
        for idx, cls_name in enumerate(CLASSES):
            fh.write(f"  {idx}: {cls_name}\n")

    per_class: dict[int, int] = {}
    for name in positives:
        for b in labels[name]:
            per_class[int(b[4])] = per_class.get(int(b[4]), 0) + 1
    print(f"数据集目录：{out}")
    print(f"  训练 {len(positives) - n_val} 张（{sum(len(labels[n]) for n in positives if n not in val)} 框）"
          f" / 验证 {n_val} 张（{sum(len(labels[n]) for n in val)} 框）"
          + (f" / 负样本 {len(negatives)} 张" if negatives else ""))
    print("  类别分布：" + ", ".join(f"{CLASSES[k]}={v}" for k, v in sorted(per_class.items())))
    print(f"  data.yaml：{yaml_path}")
    print("训练：/tmp/yolo_env/bin/python scripts/train_yolo.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
