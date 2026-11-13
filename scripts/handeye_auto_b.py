#!/usr/bin/env python3
"""自动跑一遍模式 B：换观察位 → 红球检测 → 点击 → 记录，最后保存。"""
import json
import sys
import time
import urllib.request

import cv2
import numpy as np

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8776"
POSES = int(sys.argv[2]) if len(sys.argv) > 2 else 5


def post(path, body=None):
    req = urllib.request.Request(BASE + path, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=90) as r:
        return json.loads(r.read())


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=30) as r:
        return json.loads(r.read())


def grab():
    """从 /stream 取一帧 JPEG。"""
    with urllib.request.urlopen(BASE + "/stream", timeout=15) as r:
        buf = b""
        while True:
            chunk = r.read(8192)
            if not chunk:
                break
            buf += chunk
            i = buf.find(b"\r\n\r\n")
            if i > 0:
                j = buf.find(b"--frame", i)
                if j > 0:
                    return cv2.imdecode(np.frombuffer(buf[i + 4:j - 2], np.uint8), cv2.IMREAD_COLOR)
    return None


def find_red(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 110, 60), (12, 255, 255)) | cv2.inRange(hsv, (168, 110, 60), (180, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None, 0
    k = int(np.argmax(stats[1:, cv2.CC_STAT_AREA])) + 1
    area = int(stats[k, cv2.CC_STAT_AREA])
    if area < 1200:
        return None, area
    return (int(cents[k][0]), int(cents[k][1])), area


print("state:", get("/api/state"))
print(post("/api/mode", {"mode": "B"}))
for k in range(POSES):
    r = post("/api/observe", {"action": "next"})
    print(f"--- 观察位 {k+1}: {r.get('ok')} {r.get('msg')} {r.get('joints_deg')}")
    if not r.get("ok"):
        continue
    time.sleep(1.2)
    img = grab()
    if img is None:
        print("  取不到画面"); continue
    uv, area = find_red(img)
    if uv is None:
        print(f"  没看到红球（area={area}）"); continue
    c = post("/api/click", {"u": uv[0], "v": uv[1], "allow": False})
    print(f"  红球 {uv} area={area} -> {c.get('ok')} z={c.get('P_base', [None,None,None])[2]}")
    rec = post("/api/record")
    print("  record:", rec.get("ok"), rec.get("msg"))
s = get("/api/state")
print("samples:", s.get("samples"))
res = post("/api/save")
print(json.dumps({k: v for k, v in res.items() if k != "report"}, ensure_ascii=False, indent=2))
print(res.get("report", ""))
