#!/usr/bin/env python3
"""汇总 cmd_chain 记录：三跳 cmd_vel 的帧数/非零比例，以及「守卫输出为 0 时」控制器给了什么。

用法：
    python3 scripts/cmd_chain_report.py [rbnx-boot/data/cmd_chain_*.jsonl]
不给路径时取最新的那份。
"""
import json
import sys
from pathlib import Path


def load(path: Path):
    rows = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def fmt(v: float) -> str:
    return f"{v:+.3f}"


def main(argv) -> int:
    if len(argv) > 1:
        path = Path(argv[1])
    else:
        root = Path(__file__).resolve().parents[1]
        found = sorted((root / "rbnx-boot" / "data").glob("cmd_chain_*.jsonl"))
        if not found:
            print("没找到 cmd_chain_*.jsonl")
            return 1
        path = found[-1]
    rows = load(path)
    if not rows:
        print(f"{path} 里没有记录（当时可能没有导航在跑）")
        return 0

    span = rows[-1]["t"] - rows[0]["t"]
    print(f"轨迹 : {path.name}")
    print(f"时长 : {span:.1f} s，记录 {len(rows)} 帧")
    print()
    topics = ("/cmd_vel_nav", "/cmd_vel_guard_input", "/cmd_vel")
    by_topic = {name: [r for r in rows if r["topic"] == name] for name in topics}
    for name in topics:
        items = by_topic[name]
        if not items:
            print(f"{name:24s} 0 帧（这个话题没数据）")
            continue
        nz = [r for r in items if abs(r["lv"]) > 1e-3 or abs(r["av"]) > 1e-3]
        hz = len(items) / span if span > 0 else 0.0
        print(f"{name:24s} {len(items):5d} 帧 ({hz:4.1f} Hz)，非零 {len(nz):5d} "
              f"({100.0 * len(nz) / len(items):3.0f}%)")
        if items:
            print(f"{'':24s} 线速度 max {max(abs(r['lv']) for r in items):.3f} m/s，"
                  f"角速度 max {max(abs(r['av']) for r in items):.3f} rad/s")

    # 关键判据：输出(= /cmd_vel) 为 0、且 guard 输入也是 0 时，控制器那一刻给了什么
    out_zero = [r for r in by_topic["/cmd_vel"] if abs(r["lv"]) <= 1e-3 and abs(r["av"]) <= 1e-3]
    if out_zero and by_topic["/cmd_vel_nav"]:
        nav = by_topic["/cmd_vel_nav"]
        paired = []
        for r in out_zero:
            near = min(nav, key=lambda q: abs(q["t"] - r["t"]))
            if abs(near["t"] - r["t"]) <= 0.5:
                paired.append(near)
        if paired:
            nz = [p for p in paired if abs(p["lv"]) > 1e-3 or abs(p["av"]) > 1e-3]
            print()
            print(f"输出为 0 的 {len(out_zero)} 帧里，{len(paired)} 帧能对上控制器输出：")
            print(f"  其中控制器**非零**的有 {len(nz)} 帧 "
                  f"({100.0 * len(nz) / len(paired):.0f}%)")
            if nz:
                sample = nz[0]
                print(f"  例：控制器 {fmt(sample['lv'])} m/s / {fmt(sample['av'])} rad/s "
                      f"→ 输出 0（截断发生在控制器之后：smoother 或 guard）")
            else:
                print("  控制器自己就是 0 → 说明 DWB 认为没有可走的轨迹，"
                      "要查代价地图/局部障碍/足迹碰撞，而不是 smoother")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
