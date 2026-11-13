#!/usr/bin/env python3
"""同一帧在不同分辨率下问 VLM 要框：看精度掉多少、延迟省多少。

用法: python3 scripts/vlm_resolution_test.py [图片路径] [重复次数]
"""
import base64
import json
import os
import re
import sys
import time
import urllib.request

import cv2

DEPLOY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_env():
    env = {}
    with open(os.path.join(DEPLOY, ".env"), "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            if line.startswith("export "):
                line = line[len("export "):]
            key, _, value = line.partition("=")
            env[key.strip()] = value.strip().strip('"').strip("'")
    return env


PROMPT = (
    'You are a machine vision expert. Locate the "{name}" in this image.\n'
    'Return PURE JSON, no markdown, with:\n'
    '{{"box_center_x": float, "box_center_y": float, "box_width": float, '
    '"box_height": float, "angle_deg": float, "failed": bool}}\n'
    'All coordinates are NORMALISED to 0..1 of the image width/height.\n'
    'If the object is not visible, set "failed": true.'
)


def ask(base_url, api_key, model, jpeg, name="carrot"):
    body = {
        "model": model, "temperature": 0.0,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": PROMPT.format(name=name)},
            {"type": "image_url", "image_url": {
                "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}},
        ]}],
    }
    request = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(body).encode(),
        headers={"Authorization": "Bearer " + api_key,
                 "Content-Type": "application/json"})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=120) as response:
        content = json.loads(response.read())["choices"][0]["message"]["content"]
    elapsed = time.monotonic() - started
    match = re.search(r"\{.*\}", content, re.S)
    if match is None:
        return {"error": content[:80], "elapsed_s": elapsed}
    answer = json.loads(match.group(0))
    answer["elapsed_s"] = round(elapsed, 2)
    return answer


def main():
    path = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "-" else None
    repeats = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    env = load_env()
    base_url = env["VLM_BASE_URL"]
    api_key = env["VLM_API_KEY"]
    model = env.get("VLM_MODEL", "deepseek-flash")
    print("model=%s base=%s" % (model, base_url))

    if path is None:
        import glob
        candidates = sorted(glob.glob(os.path.expanduser(
            "~/.robonix/grasp_geometry/logs/*/rgb.jpg")))
        path = candidates[-1]
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit("读不到图片: %s" % path)
    height, width = frame.shape[:2]
    print("image=%s %dx%d" % (path, width, height))

    widths = [1280, 960, 768, 640]
    results = {}
    for round_index in range(repeats):
        for target in widths:
            scale = target / float(width)
            resized = frame if abs(scale - 1.0) < 1e-6 else cv2.resize(
                frame, (target, max(1, int(round(height * scale)))),
                interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise SystemExit("编码失败")
            answer = ask(base_url, api_key, model, jpeg.tobytes())
            if answer.get("failed") or "box_center_x" not in answer:
                print("  %4d r%d  failed=%s %s" % (target, round_index,
                                                   answer.get("failed"), answer.get("error", "")))
                continue
            h2, w2 = resized.shape[:2]
            cx = float(answer["box_center_x"]) * w2
            cy = float(answer["box_center_y"]) * h2
            bw = float(answer["box_width"]) * w2
            bh = float(answer["box_height"]) * h2
            print("  %4d r%d  center=(%6.1f,%6.1f) size=(%5.1f,%5.1f) angle=%6.1f "
                  "jpeg=%6.0f KB  %.1f s"
                  % (target, round_index, cx, cy, bw, bh,
                     float(answer.get("angle_deg", 0)), len(jpeg) / 1024.0,
                     answer["elapsed_s"]))
            results.setdefault(target, []).append((cx, cy, bw, bh, answer["elapsed_s"],
                                                   len(jpeg)))
    print("\n== summary (mean of repeats) ==")
    baseline = results.get(1280)
    for target in widths:
        rows = results.get(target)
        if not rows:
            print("  %4d  no answer" % target)
            continue
        cx = sum(r[0] for r in rows) / len(rows)
        cy = sum(r[1] for r in rows) / len(rows)
        bw = sum(r[2] for r in rows) / len(rows)
        bh = sum(r[3] for r in rows) / len(rows)
        sec = sum(r[4] for r in rows) / len(rows)
        size = sum(r[5] for r in rows) / len(rows)
        delta = ""
        if baseline:
            bx = sum(r[0] for r in baseline) / len(baseline)
            by = sum(r[1] for r in baseline) / len(baseline)
            delta = "  shift_vs_1280=%.1f px" % (((cx - bx) ** 2 + (cy - by) ** 2) ** 0.5)
        print("  %4d  center=(%6.1f,%6.1f) size=(%5.1f,%5.1f)  %.1f s  %5.0f KB%s"
              % (target, cx, cy, bw, bh, sec, size, delta))


if __name__ == "__main__":
    main()
