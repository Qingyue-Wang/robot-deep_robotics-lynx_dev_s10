#!/usr/bin/env python3
"""在 Orin NX 上训练抓取检测器（YOLOv8n，GPU FP16）。

前提：
  * 数据集已导出：python3 scripts/yolo_dataset.py  → ~/grasp_labels/dataset/data.yaml
  * venv：/tmp/yolo_env（numpy 1.26.4 + torch 2.8.0 + ultralytics 8.4.95）
      重建：python3 -m venv --system-site-packages /tmp/yolo_env
            /tmp/yolo_env/bin/pip install "numpy<2" "onnx==1.17.0"
  * 预训练权重：/tmp/yolov8n.pt（离线环境用本地文件，避免每次去 GitHub 下载）

用法：
    /tmp/yolo_env/bin/python scripts/train_yolo.py                 # 默认 100 epochs
    /tmp/yolo_env/bin/python scripts/train_yolo.py --epochs 3 --name smoke   # 冒烟
    /tmp/yolo_env/bin/python scripts/train_yolo.py --export-engine           # 训完导出 TensorRT
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_MODEL = "/tmp/yolov8n.pt"
DEFAULT_DATA = "~/grasp_labels/dataset/data.yaml"
DEFAULT_PROJECT = "~/grasp_labels/runs"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="预训练权重（本地 .pt）")
    ap.add_argument("--data", default=DEFAULT_DATA, help="data.yaml")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--device", default="0", help="0=GPU, cpu=CPU")
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--name", default="grasp")
    ap.add_argument("--patience", type=int, default=30)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--export-engine", action="store_true", help="训练完把 best.pt 导出成 TensorRT engine")
    args = ap.parse_args()

    from ultralytics import YOLO  # 延迟导入，--help 不需要 torch

    data = os.path.expanduser(args.data)
    if not os.path.exists(data):
        print(f"找不到数据集：{data}\n先跑：python3 scripts/yolo_dataset.py", file=sys.stderr)
        return 2
    model_path = os.path.expanduser(args.model)
    if not os.path.exists(model_path):
        print(f"找不到权重：{model_path}（离线环境请先生成/拷贝 yolov8n.pt）", file=sys.stderr)
        return 2

    model = YOLO(model_path)
    results = model.train(
        data=data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        amp=True,
        workers=args.workers,
        patience=args.patience,
        project=os.path.expanduser(args.project),
        name=args.name,
        exist_ok=True,
        resume=args.resume,
        plots=True,
        val=True,
    )

    save_dir = str(getattr(results, "save_dir", os.path.join(os.path.expanduser(args.project), args.name)))
    best = os.path.join(save_dir, "weights", "best.pt")
    print(f"训练完成：{best}")

    if args.export_engine and os.path.exists(best):
        engine = YOLO(best).export(format="engine", half=True, imgsz=args.imgsz, device=args.device, simplify=False)
        print(f"TensorRT engine：{engine}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
