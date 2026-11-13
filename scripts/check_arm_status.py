#!/usr/bin/env python3
"""扫本机所有监听端口，找出 piper_ctl 的 MCP 端点（tools 里同时有 home+status），调 status。"""
import json
import re
import subprocess
import urllib.request

def ports():
    out = subprocess.run(["ss", "-ltnp"], capture_output=True, text=True).stdout
    p = set()
    for line in out.splitlines():
        m = re.search(r":(\d+)\s", line)
        if m:
            p.add(int(m.group(1)))
    return sorted(p)

def rpc(port, method, params, sid=None, timeout=8):
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}/mcp", data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("Accept", "application/json, text/event-stream")
    if sid:
        req.add_header("mcp-session-id", sid)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode("utf-8", "replace")
        new_sid = r.headers.get("mcp-session-id")
    for line in raw.splitlines():
        if line.startswith("data: "):
            line = line[6:]
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line), new_sid
            except json.JSONDecodeError:
                continue
    return None, new_sid

for port in ports():
    try:
        r, sid = rpc(port, "initialize", {"protocolVersion": "2024-11-05", "capabilities": {},
                                          "clientInfo": {"name": "probe", "version": "0"}}, timeout=3)
        if not r:
            continue
        rpc(port, "notifications/initialized", {}, sid, timeout=3)
        t, _ = rpc(port, "tools/list", {}, sid, timeout=5)
        names = [x.get("name") for x in (t or {}).get("result", {}).get("tools", [])]
        if "home" in names and "status" in names:
            print(f"port={port} tools={len(names)}")
            res, _ = rpc(port, "tools/call", {"name": "status", "arguments": {}}, sid, timeout=12)
            payload = res["result"]
            for c in payload.get("content", []):
                try:
                    obj = json.loads(c.get("text", ""))
                except json.JSONDecodeError:
                    print(c.get("text"))
                    continue
                print(json.dumps(obj, ensure_ascii=False, indent=2)[:1200])
            sys_exit = 0
            break
    except Exception:
        continue
else:
    print("没找到 piper MCP 端点")
    sys_exit = 1

raise SystemExit(sys_exit)
