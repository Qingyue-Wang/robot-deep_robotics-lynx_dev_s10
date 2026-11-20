#!/usr/bin/env python3
"""本地画框标注工具（零依赖，python3 标准库即可）。

用法（在机器人上跑，然后用浏览器打开 http://<机器人IP>:8088）：

    python3 scripts/label_tool.py                       # 默认 ~/grasp_labels
    python3 scripts/label_tool.py --images ~/grasp_labels/images --port 8088
    python3 scripts/label_tool.py --collect             # 先把历史日志里的 rgb.jpg 收进 images/

标注结果：
    ~/grasp_labels/labels.json            全部框（归一化 cx,cy,w,h + 类别）
    ~/grasp_labels/labels/<name>.txt      YOLO 格式（每行：cls cx cy w h）
    ~/grasp_labels/flags.json             跳过/难例的标记

快捷键：A/← 上一张，D/→ 下一张，1..5 选类别，Delete 删框，U 撤销，S 保存，空格+拖动 平移，滚轮缩放。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

ROOT = os.path.expanduser("~/grasp_labels")
IMAGES = os.path.join(ROOT, "images")
LABELS = os.path.join(ROOT, "labels")
CLASSES = ["carrot", "potato", "block", "other"]

_lock = threading.Lock()


def _ensure_dirs() -> None:
    os.makedirs(IMAGES, exist_ok=True)
    os.makedirs(LABELS, exist_ok=True)


def _labels_path() -> str:
    return os.path.join(ROOT, "labels.json")


def _flags_path() -> str:
    return os.path.join(ROOT, "flags.json")


def _read_json(path: str, default):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return default


def _write_json_atomic(path: str, payload) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=1)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def collect_from_logs(src_glob_root: str) -> int:
    """把历史 grasp_geometry 日志里的 rgb.jpg 收进 images/（重名跳过）。"""
    import glob

    _ensure_dirs()
    added = 0
    for path in sorted(glob.glob(os.path.join(src_glob_root, "*", "rgb.jpg"))):
        run_id = os.path.basename(os.path.dirname(path))
        dst = os.path.join(IMAGES, f"log_{run_id}.jpg")
        if os.path.exists(dst):
            continue
        try:
            shutil.copy2(path, dst)
            added += 1
        except Exception as exc:  # noqa: BLE001
            print(f"skip {path}: {exc}")
    return added


def list_images() -> list[str]:
    names = []
    for name in sorted(os.listdir(IMAGES)):
        if name.lower().endswith((".jpg", ".jpeg", ".png")):
            names.append(name)
    return names


def save_label(name: str, boxes: list, flag: str) -> None:
    with _lock:
        labels = _read_json(_labels_path(), {})
        flags = _read_json(_flags_path(), {})
        clean = []
        for box in boxes or []:
            try:
                cx, cy, w, h, cls = (float(box[0]), float(box[1]), float(box[2]),
                                     float(box[3]), int(box[4]))
            except Exception:
                continue
            if w <= 0 or h <= 0:
                continue
            clean.append([round(min(max(cx, 0.0), 1.0), 6), round(min(max(cy, 0.0), 1.0), 6),
                          round(min(w, 1.0), 6), round(min(h, 1.0), 6), cls])
        if clean:
            labels[name] = clean
        else:
            labels.pop(name, None)
        if flag:
            flags[name] = flag
        else:
            flags.pop(name, None)
        _write_json_atomic(_labels_path(), labels)
        _write_json_atomic(_flags_path(), flags)

        txt = os.path.join(LABELS, os.path.splitext(name)[0] + ".txt")
        if clean:
            with open(txt, "w", encoding="utf-8") as fh:
                for cx, cy, w, h, cls in clean:
                    fh.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
        elif os.path.exists(txt):
            os.remove(txt)


PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>抓取标注台</title>
<style>
  :root { --bg:#0f1115; --panel:#171a21; --line:#2a2f3a; --fg:#e8ecf1; --dim:#8b94a3; --acc:#4ea1ff; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.5 -apple-system,"Segoe UI",Roboto,"PingFang SC","Microsoft YaHei",sans-serif; }
  header { display:flex; gap:12px; align-items:center; padding:10px 14px; background:var(--panel); border-bottom:1px solid var(--line); position:sticky; top:0; z-index:5; }
  header .title { font-weight:600; }
  header .spacer { flex:1; }
  button, select { background:#202531; color:var(--fg); border:1px solid var(--line); border-radius:6px; padding:6px 10px; cursor:pointer; font-size:13px; }
  button:hover { border-color:var(--acc); }
  button.primary { background:var(--acc); color:#08121f; border-color:var(--acc); font-weight:600; }
  main { display:grid; grid-template-columns: 1fr 300px; gap:12px; padding:12px; }
  #stage { position:relative; background:#000; border:1px solid var(--line); border-radius:8px; overflow:hidden; min-height:420px; display:flex; align-items:center; justify-content:center; }
  #cv { max-width:100%; max-height:calc(100vh - 160px); display:block; }
  aside { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px; }
  .row { display:flex; gap:8px; align-items:center; margin-bottom:8px; }
  .dim { color:var(--dim); }
  .pill { display:inline-block; padding:2px 8px; border-radius:999px; font-size:12px; background:#232936; }
  #list { margin-top:6px; max-height:40vh; overflow:auto; }
  .item { display:flex; justify-content:space-between; align-items:center; padding:6px 8px; border-radius:6px; border:1px solid var(--line); margin-bottom:6px; font-size:13px; }
  .item.sel { border-color:var(--acc); background:#1b2432; }
  .item .x { color:#ff6b6b; cursor:pointer; padding:0 4px; }
  #hint { padding:6px 14px; color:var(--dim); font-size:12px; }
  #toast { position:fixed; right:16px; bottom:16px; background:#1f2735; border:1px solid var(--acc); color:var(--fg); padding:8px 12px; border-radius:8px; opacity:0; transition:opacity .2s; pointer-events:none; }
  label.chk { display:flex; gap:6px; align-items:center; color:var(--dim); font-size:12px; }
</style>
</head>
<body>
<header>
  <span class="title">抓取标注台</span>
  <button id="prev">◀ 上一张 (A)</button>
  <button id="next">下一张 (D) ▶</button>
  <span class="pill" id="pos">-/-</span>
  <span class="pill" id="stat">已标 0</span>
  <span class="spacer"></span>
  <select id="cls"></select>
  <label class="chk"><input type="checkbox" id="showSkip"/> 显示已跳过</label>
  <button id="skip">跳过 (X)</button>
  <button id="hard">难例 (H)</button>
  <button id="save" class="primary">保存 (S)</button>
</header>
<main>
  <div id="stage"><canvas id="cv"></canvas></div>
  <aside>
    <div class="row"><b id="fname" style="word-break:break-all"></b></div>
    <div class="row"><span class="dim">本图框数：</span><b id="cnt">0</b></div>
    <div class="row"><button id="undo">撤销 (U)</button><button id="clear">清空</button></div>
    <div id="list"></div>
    <div class="dim" style="margin-top:10px;font-size:12px">
      拖动鼠标画框；点框选中后按 Delete 删除；滚轮缩放，空格+拖动平移。
    </div>
  </aside>
</main>
<div id="hint">类别：1 胡萝卜 / 2 土豆 / 3 方块 / 4 其它</div>
<div id="toast"></div>
<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
let state = { images: [], classes: [], labels: {}, flags: {} };
let idx = 0, img = null, boxes = [], sel = -1, hist = [];
let zoom = 1, panX = 0, panY = 0, spaceDown = false, panning = false, panStart = null;
let drawing = null, showSkip = false;

const clsSel = document.getElementById('cls');
const toast = (msg) => { const t = document.getElementById('toast'); t.textContent = msg; t.style.opacity = 1;
  clearTimeout(t._h); t._h = setTimeout(() => t.style.opacity = 0, 1600); };

async function load() {
  const r = await fetch('/api/state'); state = await r.json();
  clsSel.innerHTML = '';
  state.classes.forEach((c, i) => { const o = document.createElement('option'); o.value = i; o.textContent = (i+1) + '. ' + c; clsSel.appendChild(o); });
  clsSel.value = localStorage.getItem('cls') || '0';
  clsSel.onchange = () => localStorage.setItem('cls', clsSel.value);
  document.getElementById('showSkip').onchange = (e) => { showSkip = e.target.checked; idx = 0; render(); };
  render();
}

function visibleImages() { return state.images.filter(n => showSkip || !state.flags[n]); }

function currentName() { const v = visibleImages(); return v[idx]; }

function render() {
  const v = visibleImages();
  const name = v[idx];
  document.getElementById('pos').textContent = (idx+1) + '/' + v.length;
  const total = state.images.length, done = state.images.filter(n => (state.labels[n]||[]).length || state.flags[n]).length;
  document.getElementById('stat').textContent = '已标 ' + done + '/' + total;
  if (!name) { ctx.clearRect(0,0,cv.width,cv.height); return; }
  document.getElementById('fname').textContent = name;
  boxes = JSON.parse(JSON.stringify(state.labels[name] || []));
  sel = -1; hist = [];
  img = new Image();
  img.onload = () => { cv.width = img.naturalWidth; cv.height = img.naturalHeight; zoom = 1; panX = panY = 0; draw(); renderList(); };
  img.src = '/img/' + encodeURIComponent(name);
}

function draw() {
  if (!img) return;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.setTransform(zoom, 0, 0, zoom, panX, panY);
  ctx.drawImage(img, 0, 0, cv.width, cv.height);
  boxes.forEach((b, i) => {
    const [cx, cy, w, h, cls] = b;
    const x = (cx - w/2) * cv.width, y = (cy - h/2) * cv.height, ww = w * cv.width, hh = h * cv.height;
    ctx.lineWidth = Math.max(2, cv.width / 500);
    ctx.strokeStyle = i === sel ? '#4ea1ff' : '#ffd166';
    ctx.strokeRect(x, y, ww, hh);
    const label = (state.classes[cls] || ('c' + cls));
    ctx.font = Math.max(14, cv.width / 60) + 'px sans-serif';
    const tw = ctx.measureText(label).width + 10;
    ctx.fillStyle = i === sel ? '#4ea1ff' : '#ffd166';
    ctx.fillRect(x, Math.max(0, y - 22), tw, 22);
    ctx.fillStyle = '#111';
    ctx.fillText(label, x + 5, Math.max(16, y - 6));
  });
  if (drawing) {
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = '#4ea1ff';
    ctx.lineWidth = 2;
    ctx.strokeRect(drawing.x, drawing.y, drawing.w, drawing.h);
    ctx.setLineDash([]);
  }
}

function renderList() {
  const list = document.getElementById('list'); list.innerHTML = '';
  document.getElementById('cnt').textContent = boxes.length;
  boxes.forEach((b, i) => {
    const el = document.createElement('div'); el.className = 'item' + (i === sel ? ' sel' : '');
    el.innerHTML = '<span>' + (i+1) + '. ' + (state.classes[b[4]] || b[4]) +
      ' <span class="dim">' + (b[2]*100).toFixed(0) + '×' + (b[3]*100).toFixed(0) + '%</span></span>' +
      '<span class="x" data-i="' + i + '">✕</span>';
    el.onclick = (e) => {
      if (e.target.dataset.i !== undefined) { removeBox(parseInt(e.target.dataset.i)); }
      else { sel = i; draw(); renderList(); }
    };
    list.appendChild(el);
  });
}

function pushHist() { hist.push(JSON.stringify(boxes)); if (hist.length > 60) hist.shift(); }
function removeBox(i) { pushHist(); boxes.splice(i, 1); sel = -1; draw(); renderList(); }
function undo() { if (!hist.length) return; boxes = JSON.parse(hist.pop()); sel = -1; draw(); renderList(); }

function evPos(ev) {
  const r = cv.getBoundingClientRect();
  const sx = cv.width / r.width, sy = cv.height / r.height;
  return { x: ((ev.clientX - r.left) * sx - panX) / zoom, y: ((ev.clientY - r.top) * sy - panY) / zoom };
}

cv.addEventListener('mousedown', (ev) => {
  if (spaceDown) { panning = true; panStart = ev; return; }
  const p = evPos(ev);
  const hit = boxes.findIndex((b) => {
    const x = (b[0]-b[2]/2)*cv.width, y = (b[1]-b[3]/2)*cv.height;
    return p.x >= x && p.y >= y && p.x <= x + b[2]*cv.width && p.y <= y + b[3]*cv.height;
  });
  if (hit >= 0) { sel = hit; draw(); renderList(); return; }
  drawing = { x: p.x, y: p.y, w: 0, h: 0 };
});
cv.addEventListener('mousemove', (ev) => {
  if (panning && panStart) { panX += ev.clientX - panStart.clientX; panY += ev.clientY - panStart.clientY; panStart = ev; return; }
  if (!drawing) return;
  const p = evPos(ev);
  drawing.w = p.x - drawing.x; drawing.h = p.y - drawing.y;
  if (drawing.w < 0) { drawing.x = p.x; drawing.w = -drawing.w; }
  if (drawing.h < 0) { drawing.y = p.y; drawing.h = -drawing.h; }
  draw();
});
window.addEventListener('mouseup', (ev) => {
  if (panning) { panning = false; panStart = null; return; }
  if (!drawing) return;
  const d = drawing; drawing = null;
  if (d.w < 6 || d.h < 6) { draw(); return; }
  pushHist();
  boxes.push([ (d.x + d.w/2)/cv.width, (d.y + d.h/2)/cv.height, d.w/cv.width, d.h/cv.height, parseInt(clsSel.value) ]);
  sel = boxes.length - 1;
  draw(); renderList(); save(true);
});
cv.addEventListener('wheel', (ev) => { ev.preventDefault(); zoom = Math.min(4, Math.max(1, zoom * (ev.deltaY < 0 ? 1.1 : 0.9))); applyZoom(); }, { passive: false });
function applyZoom() { draw(); }

async function save(auto) {
  const name = currentName(); if (!name) return;
  state.labels[name] = JSON.parse(JSON.stringify(boxes));
  await fetch('/api/save', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ name, boxes, flag: state.flags[name] || '' }) });
  if (auto) toast('已保存'); render();
}
async function setFlag(flag) {
  const name = currentName(); if (!name) return;
  state.flags[name] = flag;
  await fetch('/api/save', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ name, boxes, flag }) });
  toast(flag === 'skip' ? '已跳过' : '已标为难例');
  next();
}
function next() { const v = visibleImages(); if (idx < v.length - 1) { idx++; render(); } }
function prev() { const v = visibleImages(); if (idx > 0) { idx--; render(); } }

document.getElementById('next').onclick = next;
document.getElementById('prev').onclick = prev;
document.getElementById('save').onclick = () => save(false).then(() => toast('已保存'));
document.getElementById('skip').onclick = () => setFlag('skip');
document.getElementById('hard').onclick = () => setFlag('hard');
document.getElementById('undo').onclick = undo;
document.getElementById('clear').onclick = () => { pushHist(); boxes = []; sel = -1; draw(); renderList(); };

document.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT' || ev.target.tagName === 'SELECT') return;
  const k = ev.key;
  if (k === ' ') { spaceDown = true; ev.preventDefault(); return; }
  if (k === 'ArrowRight' || k === 'd' || k === 'D') next();
  else if (k === 'ArrowLeft' || k === 'a' || k === 'A') prev();
  else if (k === 'Delete' || k === 'Backspace') { if (sel >= 0) removeBox(sel); }
  else if (k === 'u' || k === 'U') undo();
  else if (k === 's' || k === 'S') save(true);
  else if (k === 'x' || k === 'X') setFlag('skip');
  else if (k === 'h' || k === 'H') setFlag('hard');
  else if (/^[1-9]$/.test(k)) { const i = parseInt(k) - 1; if (i < state.classes.length) { clsSel.value = String(i); localStorage.setItem('cls', String(i)); } }
});
document.addEventListener('keyup', (ev) => { if (ev.key === ' ') spaceDown = false; });

load();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # noqa: D401
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/api/state":
            labels = _read_json(_labels_path(), {})
            flags = _read_json(_flags_path(), {})
            payload = {"images": list_images(), "classes": CLASSES, "labels": labels, "flags": flags}
            self._send(200, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json")
        elif path.startswith("/img/"):
            name = unquote(path[len("/img/"):])
            full = os.path.join(IMAGES, name)
            if not os.path.isfile(full) or not os.path.abspath(full).startswith(os.path.abspath(IMAGES)):
                self._send(404, b"not found", "text/plain")
                return
            with open(full, "rb") as fh:
                data = fh.read()
            ctype = "image/png" if name.lower().endswith(".png") else "image/jpeg"
            self._send(200, data, ctype)
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):  # noqa: N802
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else b"{}"
        if path == "/api/save":
            try:
                payload = json.loads(body or b"{}")
                name = str(payload.get("name") or "")
                if not name or "/" in name:
                    raise ValueError("bad name")
                save_label(name, payload.get("boxes") or [], str(payload.get("flag") or ""))
                self._send(200, b'{"ok":true}', "application/json")
            except Exception as exc:  # noqa: BLE001
                self._send(400, json.dumps({"ok": False, "error": str(exc)}).encode(), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main() -> int:
    global ROOT, IMAGES, LABELS
    ap = argparse.ArgumentParser(description="抓取标注台")
    ap.add_argument("--root", default=ROOT, help="数据目录（默认 ~/grasp_labels）")
    ap.add_argument("--images", default="", help="图片目录（默认 <root>/images）")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--collect", action="store_true", help="先把历史 grasp_geometry 日志的 rgb.jpg 收进来")
    args = ap.parse_args()

    ROOT = os.path.expanduser(args.root)
    IMAGES = os.path.expanduser(args.images) if args.images else os.path.join(ROOT, "images")
    LABELS = os.path.join(ROOT, "labels")
    _ensure_dirs()

    if args.collect:
        n = collect_from_logs(os.path.expanduser("~/.robonix/grasp_geometry/logs"))
        print(f"[collect] 收进 {n} 张历史帧 → {IMAGES}")

    imgs = list_images()
    labels = _read_json(_labels_path(), {})
    print(f"图片目录：{IMAGES}（{len(imgs)} 张）")
    print(f"已有标注：{len(labels)} 张")
    print(f"打开： http://<机器人IP>:{args.port}/    （本机可试 http://127.0.0.1:{args.port}/）")

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
