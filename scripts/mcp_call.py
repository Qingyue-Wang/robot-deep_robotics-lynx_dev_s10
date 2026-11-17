#!/usr/bin/env python3
"""从任意 MCP 端口调一个工具（默认找含 home+status 的 piper 端口）。

用法：python3 scripts/mcp_call.py <tool> '<json args>' [port]
不带 port 时按 `ss -ltnp` 扫出所有监听端口，逐个 initialize + tools/list，
挑第一个同时含 `home` 和 `status` 的（= piper_ctl 的 MCP 端点）。
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
import urllib.request


def _post(port: int, payload: dict, sid: str | None = None) -> tuple[dict, str | None]:
    headers = {"Content-Type": "application/json",
               "Accept": "application/json, text/event-stream"}
    if sid:
        headers["mcp-session-id"] = sid
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}/mcp",
        data=json.dumps(payload).encode(),
        headers=headers,
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        raw = resp.read().decode()
        sid2 = resp.headers.get("mcp-session-id", sid)
    if "data: " in raw:
        lines = [ln[5:].strip() for ln in raw.splitlines() if ln.startswith("data: ")]
        raw = lines[-1] if lines else "{}"
    try:
        return json.loads(raw), sid2
    except json.JSONDecodeError:
        return {}, sid2


def _handshake(port: int) -> tuple[str | None, list[str]]:
    body, sid = _post(port, {"jsonrpc": "2.0", "id": 1, "method": "initialize",
                             "params": {"protocolVersion": "2024-11-05",
                                        "capabilities": {},
                                        "clientInfo": {"name": "mcp_call", "version": "0"}}})
    if not sid:
        return None, []
    _post(port, {"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    body, _ = _post(port, {"jsonrpc": "2.0", "id": 2, "method": "tools/list"}, sid)
    return sid, [t.get("name", "") for t in body.get("result", {}).get("tools", [])]


def find_port(wanted: tuple[str, ...] = ("home", "status")) -> int | None:
    out = subprocess.run(["ss", "-ltnp"], capture_output=True, text=True).stdout
    ports = sorted({int(m) for m in re.findall(r":(\d{4,5})\b", out)
                    if 40000 <= int(m) <= 65000})
    for port in ports:
        try:
            _sid, names = _handshake(port)
        except Exception:  # noqa: BLE001
            continue
        if all(w in names for w in wanted):
            return port
    return None


def main() -> int:
    tool = sys.argv[1] if len(sys.argv) > 1 else "status"
    args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    port = int(sys.argv[3]) if len(sys.argv) > 3 else find_port()
    if port is None:
        print("找不到 piper_ctl 的 MCP 端口", file=sys.stderr)
        return 2
    sid, names = _handshake(port)
    if tool not in names:
        print(f"端口 {port} 没有工具 {tool}；有：{names}", file=sys.stderr)
        return 2
    body, _ = _post(port, {"jsonrpc": "2.0", "id": 3, "method": "tools/call",
                           "params": {"name": tool, "arguments": args}}, sid)
    result = body.get("result", {})
    for item in result.get("content", []):
        print(item.get("text", ""))
    if result.get("structuredContent"):
        print(json.dumps(result["structuredContent"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
