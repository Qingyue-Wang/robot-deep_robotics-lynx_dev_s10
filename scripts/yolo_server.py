#!/usr/bin/env python3
"""YOLO 检测旁路服务（sidecar）。

为什么要单独一个进程：本机系统 python 的 numpy 是 2.2.6，而 torch 2.8.0 是编译在
numpy 1.x 上的，系统环境里 `torch.from_numpy` 直接报 "Numpy is not available"。
所以 YOLO 跑在 `~/yolo_env`（numpy 1.26.4）里，通过本机 HTTP 给 grasp_geometry 提供
检测结果。进程崩了/没起，grasp_geometry 会自动退回 VLM，不会把整栈带走。

接口：
    GET  /health   → {"ok": true, "weights": "...", "names": {...}, "device": "0"}
    POST /detect   {"images": [{"name": "x.jpg", "b64": "<jpeg base64>"}], "conf": 0.35}
                   → {"ok": true, "ms": 12.3,
                      "items": [[{"x1","y1","x2","y2","cx","cy","w","h",
                                  "angle_deg","conf","cls","name"}], ...]}
                   items[i] 对应 images[i]；没检到就是空列表。

启动：
    ~/yolo_env/bin/python scripts/yolo_server.py \
        --weights ~/grasp_labels/runs/finetune/weights/best.pt --port 8899
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LOCK = threading.Lock()
MODEL = None
ARGS = None


def _decode(b64: str):
    import cv2
    import numpy as np
    raw = base64.b64decode(b64.split(",")[-1])
    arr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)  # BGR
    if arr is None:
        raise RuntimeError("这一帧解不成图片")
    return arr


def detect(images, conf: float, trace: bool = False):
    import numpy as np
    out = []
    with LOCK:
        for arr in images:
            _t0 = time.monotonic()
            r = MODEL.predict(arr, imgsz=ARGS.imgsz, conf=conf, device=ARGS.device,
                              verbose=False)[0]
            boxes = []
            obb = getattr(r, "obb", None)
            if obb is not None and len(obb) > 0:
                xyxy = obb.xyxy.cpu().numpy()
                xywhr = obb.xywhr.cpu().numpy()
                confs = obb.conf.cpu().numpy()
                clss = obb.cls.cpu().numpy().astype(int)
                for (x1, y1, x2, y2), (cx, cy, w, h, rad), c, k in zip(xyxy, xywhr, confs, clss):
                    ang = math.degrees(float(rad))
                    while ang > 90.0:
                        ang -= 180.0
                    while ang <= -90.0:
                        ang += 180.0
                    boxes.append({
                        "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
                        "cx": float(cx), "cy": float(cy), "w": float(w), "h": float(h),
                        "angle_deg": round(ang, 2), "conf": round(float(c), 4),
                        "cls": int(k), "name": str(MODEL.names.get(int(k), k)),
                    })
            boxes.sort(key=lambda b: b["conf"], reverse=True)
            if trace:
                print(f"[yolo_server] predict {arr.shape} {(time.monotonic()-_t0)*1000:.0f} ms", flush=True)
            out.append(boxes)
    return out


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # 安静点，错误单独打
        pass

    def _send(self, code: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except BrokenPipeError:
            pass

    def do_GET(self):
        if self.path.startswith("/health"):
            self._send(200, {"ok": True, "weights": ARGS.weights,
                             "names": MODEL.names, "device": ARGS.device})
        else:
            self._send(404, {"ok": False, "message": "unknown path"})

    def do_POST(self):
        if self.path.startswith("/bench"):
            import numpy as np
            arr = np.zeros((720, 1280, 3), dtype=np.uint8)
            times = []
            for _ in range(6):
                t0 = time.monotonic()
                MODEL.predict(arr, imgsz=ARGS.imgsz, conf=0.35, device=ARGS.device, verbose=False)
                times.append(round((time.monotonic() - t0) * 1000, 1))
            self._send(200, {"ok": True, "times": times, "device": ARGS.device,
                             "cuda": str(getattr(MODEL, "device", ""))})
            return
        if not self.path.startswith("/detect"):
            self._send(404, {"ok": False, "message": "unknown path"})
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            req = json.loads(self.rfile.read(n) or b"{}")
            _d0 = time.monotonic()
            images = [_decode(it["b64"]) for it in (req.get("images") or [])]
            _dec = (time.monotonic() - _d0) * 1000
            if not images:
                self._send(400, {"ok": False, "message": "images 为空"})
                return
            conf = float(req.get("conf") or ARGS.conf)
            t0 = time.monotonic()
            items = detect(images, conf, trace=True)
            print(f"[yolo_server] decode {_dec:.0f} ms", flush=True)
            self._send(200, {"ok": True, "ms": round((time.monotonic() - t0) * 1000, 1),
                             "items": items})
        except Exception as error:  # noqa: BLE001 —— 旁路服务别把异常吞掉
            import traceback
            traceback.print_exc()
            self._send(500, {"ok": False, "message": f"{type(error).__name__}: {error}"})


def main() -> int:
    global MODEL, ARGS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8899)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--half", action="store_true", default=True)
    ap.add_argument("--no-half", dest="half", action="store_false")
    ap.add_argument("--torch-threads", type=int, default=2)
    ap.add_argument("--warmup", action="store_true", default=True)
    ap.add_argument("--no-warmup", dest="warmup", action="store_false")
    ARGS = ap.parse_args()

    import torch
    if ARGS.torch_threads > 0:
        torch.set_num_threads(ARGS.torch_threads)   # Jetson 上默认开满 12 线程会和整栈抢核，反而慢
    from ultralytics import YOLO
    weights = os.path.expanduser(ARGS.weights)
    if not os.path.exists(weights):
        print(f"[yolo_server] 找不到权重：{weights}", file=sys.stderr)
        return 2
    MODEL = YOLO(weights)
    if ARGS.half:
        # half 只在加载时切一次；每帧传 half=True 会让 ultralytics 反复转权重（实测 290ms → 40ms）
        try:
            MODEL.model.half()
        except Exception as error:  # noqa: BLE001
            print(f"[yolo_server] half 切换失败（按 FP32 跑）：{error}", flush=True)
    print(f"[yolo_server] 载入 {weights}  task={MODEL.task} names={MODEL.names} half={ARGS.half}", flush=True)
    if ARGS.warmup:
        import numpy as np
        try:
            detect([np.zeros((720, 1280, 3), dtype=np.uint8)], ARGS.conf)
            print("[yolo_server] warmup done", flush=True)
        except Exception as error:  # noqa: BLE001
            print(f"[yolo_server] warmup 失败（忽略）：{error}", flush=True)

    server = ThreadingHTTPServer((ARGS.host, ARGS.port), Handler)
    print(f"[yolo_server] listening http://{ARGS.host}:{ARGS.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
