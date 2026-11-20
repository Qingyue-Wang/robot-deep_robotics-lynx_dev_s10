#!/usr/bin/env python3
"""YOLO 检测网页预览：把腕上相机画面 + YOLO 框实时画出来，用浏览器看准不准。

数据流：
    /camera_435i/color/image_raw (ROS, domain 43)  →  本进程（系统 python，有 rclpy/cv2）
                                                  →  HTTP  http://127.0.0.1:8899/detect （~/yolo_env 旁路）
                                                  →  MJPEG /stream  →  浏览器

用法（在部署根目录）：
    export ROS_DOMAIN_ID=43
    /usr/bin/python3 scripts/yolo_web_view.py --port 8890 --max-fps 4
打开：http://<机器狗IP>:8890/      （或 ssh -L 8890:127.0.0.1:8890 dog@<ip> 后开 http://127.0.0.1:8890/）

页面端点：
    /            HTML（实时画面 + 统计 + conf 滑块 + 暂停）
    /stream      MJPEG 流（带框）
    /snapshot    单张带框 JPEG（存图/排查用）
    /stats       最近一次检测的 JSON（含框、耗时、是否命中）
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
except Exception as error:  # pragma: no cover
    raise SystemExit("需要 rclpy（用 /usr/bin/python3 + source /opt/ros/humble/setup.bash）：%s" % error)


class LatestImage(Node):
    """只保留最新一帧的相机订阅者。"""

    def __init__(self, topic: str, domain: int) -> None:
        super().__init__("yolo_web_view")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info("订阅 %s（domain %d）" % (topic, domain))

    def _on_image(self, msg: Image) -> None:
        try:
            if msg.encoding not in ("rgb8", "bgr8"):
                self.get_logger().warning("相机编码是 %s，只支持 rgb8/bgr8" % msg.encoding,
                                          once=True)
                return
            rows = memoryview(msg.data).cast("B")
            expected = msg.height * msg.step
            if len(rows) < expected:
                return
            frame = np.frombuffer(rows[:expected], dtype=np.uint8).reshape(
                (msg.height, msg.step // 3, 3))[:, : msg.width, :]
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = np.ascontiguousarray(frame)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning("解帧失败：%s: %s" % (type(error).__name__, error),
                                      once=True)
            return
        with self._lock:
            self._frame = frame
            self._stamp = time.monotonic()

    def latest(self):
        with self._lock:
            return None if self._frame is None else (self._frame.copy(), self._stamp)


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.annotated = None          # 带框的 BGR 图
        self.stats = {"ok": False, "why": "还没开始检测"}
        self.paused = False
        self.conf = 0.35
        self.detect_ms = 0.0
        self.frames = 0


def detect(endpoint: str, image, conf: float, timeout_s: float = 5.0) -> dict:
    ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise RuntimeError("JPEG 编码失败")
    import base64
    payload = json.dumps({"images": [{"name": "web", "b64": base64.b64encode(buf).decode()}],
                          "conf": conf}).encode()
    req = urllib.request.Request(endpoint.rstrip("/") + "/detect", data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode())


def draw(image, boxes) -> "cv2.Mat":
    out = image.copy()
    for box in boxes:
        cx, cy = float(box["cx"]), float(box["cy"])
        w, h = float(box["w"]), float(box["h"])
        angle = float(box.get("angle_deg") or 0.0)
        pts = cv2.boxPoints(((cx, cy), (w, h), angle)).astype("int32")
        confirmed = float(box.get("conf") or 0) >= 0.5
        color = (60, 220, 60) if confirmed else (40, 190, 240)
        cv2.polylines(out, [pts], True, color, 2)
        label = "%s %.2f  %.0fdeg" % (box.get("name") or "?", float(box.get("conf") or 0), angle)
        x, y = int(box["x1"]), max(16, int(box["y1"]) - 6)
        cv2.putText(out, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    if not boxes:
        cv2.putText(out, "no detection", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, "no detection", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (80, 80, 255), 2, cv2.LINE_AA)
    return out


def worker(node: LatestImage, state: State, endpoint: str, max_fps: float) -> None:
    period = 1.0 / max(0.5, max_fps)
    while True:
        started = time.monotonic()
        got = node.latest()
        if got is None:
            with state.lock:
                state.stats = {"ok": False, "why": "还没有相机帧（检查 domain 43 / 相机是否在跑）"}
            time.sleep(0.3)
            continue
        frame, stamp = got
        if state.paused:
            time.sleep(0.2)
            continue
        # 帧太旧（相机断了）就别再检测
        if time.monotonic() - stamp > 2.0:
            with state.lock:
                state.stats = {"ok": False, "why": "相机帧已 %.1f s 没更新" % (time.monotonic() - stamp)}
            time.sleep(0.3)
            continue
        t0 = time.monotonic()
        try:
            reply = detect(endpoint, frame, state.conf)
            boxes = [b for item in reply.get("items", []) for b in item]
            stats = {"ok": True, "boxes": boxes, "ms": reply.get("ms"),
                     "conf": state.conf, "t": time.time(),
                     "image_shape": list(frame.shape)}
            annotated = draw(frame, boxes)
        except (urllib.error.URLError, socket.timeout, RuntimeError, json.JSONDecodeError) as error:
            stats = {"ok": False, "why": "检测服务不可用：%s" % error}
            annotated = draw(frame, [])
        with state.lock:
            state.annotated = annotated
            state.stats = stats
            state.detect_ms = (time.monotonic() - t0) * 1000.0
            state.frames += 1
        rest = period - (time.monotonic() - started)
        if rest > 0:
            time.sleep(rest)


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>YOLO 检测预览</title>
<style>
body{background:#111;color:#ddd;font-family:system-ui,-apple-system,"Segoe UI",sans-serif;margin:0;padding:16px}
h2{margin:0 0 12px;font-weight:600;font-size:18px}
img{max-width:100%;border-radius:6px;background:#000;display:block}
.row{display:flex;gap:20px;flex-wrap:wrap;align-items:flex-start}
.card{background:#1b1b1b;border:1px solid #2c2c2c;border-radius:8px;padding:12px 14px;min-width:260px}
kbd{background:#333;border-radius:4px;padding:1px 5px;font-size:12px}
.mono{font-family:ui-monospace,Menlo,monospace;font-size:13px;white-space:pre-wrap}
#hit{font-weight:600}
button{background:#2a2a2a;color:#eee;border:1px solid #3c3c3c;border-radius:6px;padding:6px 10px;cursor:pointer}
button:hover{background:#343434}
input[type=range]{width:160px;vertical-align:middle}
</style></head><body>
<h2>YOLO 检测预览 · <span id="src">--</span></h2>
<div class="row">
  <div style="flex:1 1 720px"><img id="v" src="/stream"></div>
  <div class="card">
    <div class="mono" id="stats">加载中…</div>
    <hr style="border-color:#2c2c2c">
    <div>置信度阈值 <b id="cv">0.35</b><br>
      <input id="conf" type="range" min="0.05" max="0.95" step="0.05" value="0.35">
    </div>
    <p><button id="pause">暂停</button> <button id="snap">存一张</button></p>
    <p class="mono" style="color:#888">绿框 = conf ≥ 0.5，橙框 = 0.35–0.5（建议只信绿框）</p>
  </div>
</div>
<script>
const confEl = document.getElementById('conf');
confEl.oninput = async () => {
  document.getElementById('cv').textContent = Number(confEl.value).toFixed(2);
  await fetch('/set?conf=' + confEl.value);
};
document.getElementById('pause').onclick = async (e) => {
  const r = await fetch('/toggle'); const j = await r.json();
  e.target.textContent = j.paused ? '继续' : '暂停';
};
document.getElementById('snap').onclick = () => { window.open('/snapshot?save=1', '_blank'); };
async function tick() {
  try {
    const s = await (await fetch('/stats')).json();
    document.getElementById('src').textContent = s.ok ? (s.boxes.length + ' 个候选') : '无检测';
    document.getElementById('stats').textContent = JSON.stringify(s, null, 2);
  } catch (e) { document.getElementById('stats').textContent = '取统计失败：' + e; }
  setTimeout(tick, 700);
}
tick();
</script></body></html>"""


def make_handler(node: LatestImage, state: State):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # 安静点
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/stats":
                with state.lock:
                    payload = dict(state.stats)
                    payload["detect_ms"] = round(state.detect_ms, 1)
                    payload["frames"] = state.frames
                    payload["paused"] = state.paused
                    payload["conf"] = state.conf
                body = json.dumps(payload, ensure_ascii=False, indent=2).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/snapshot":
                with state.lock:
                    frame = None if state.annotated is None else state.annotated.copy()
                if frame is None:
                    self.send_error(503, "还没有画面")
                    return
                if "save=1" in self.path:
                    outdir = os.path.expanduser("~/grasp_labels/webcheck")
                    os.makedirs(outdir, exist_ok=True)
                    cv2.imwrite(os.path.join(outdir, time.strftime("check_%Y%m%d-%H%M%S.jpg")), frame)
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                body = buf.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/toggle":
                with state.lock:
                    state.paused = not state.paused
                    paused = state.paused
                body = json.dumps({"paused": paused}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/set":
                query = self.path.split("?", 1)[1] if "?" in self.path else ""
                for part in query.split("&"):
                    if part.startswith("conf="):
                        try:
                            state.conf = max(0.05, min(0.95, float(part[5:])))
                        except ValueError:
                            pass
                body = json.dumps({"conf": state.conf}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with state.lock:
                            frame = None if state.annotated is None else state.annotated
                        if frame is None:
                            time.sleep(0.2)
                            continue
                        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                        if not ok:
                            time.sleep(0.1)
                            continue
                        data = buf.tobytes()
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(data))
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_error(404)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--topic", default="/camera_435i/color/image_raw")
    parser.add_argument("--domain", type=int, default=43)
    parser.add_argument("--endpoint", default="http://127.0.0.1:8899")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--max-fps", type=float, default=4.0, help="检测频率上限（YOLO 一帧约 200-350 ms）")
    args = parser.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", str(args.domain))
    rclpy.init()
    node = LatestImage(args.topic, args.domain)
    state = State()
    state.conf = args.conf

    threading.Thread(target=worker, args=(node, state, args.endpoint, args.max_fps),
                     daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(node, state))
    print("网页：http://0.0.0.0:%d/   （检测端点 %s，conf %.2f，最多 %.1f fps）"
          % (args.port, args.endpoint, args.conf, args.max_fps), flush=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
