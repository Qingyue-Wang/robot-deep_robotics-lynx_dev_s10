#!/usr/bin/env python3
"""分析最近一次（或指定的）导航轨迹，量化「直线程度」与「抬腿程度」。

指标
  * 转角指令占比：|ω| > 0.01 的指令样本比例（越低 = 越少抬腿）
  * 里程膨胀比  ：odom 路径长度 / 指令速度积分（≈1.0 表示轮子一路滚；
                  明显 >1 说明车体在抖 / 在抬腿，2026-11-16 11:07 那趟 1.26）
  * |ω| 分布：落在哪个响应区间

用法：python3 scripts/analyze_nav_run.py [轨迹文件]
"""
import glob
import json
import math
import os
import sys

TRACE_DIR = ("/home/dog/robot-deep_robotics-lynx_s10/rbnx-boot/cache/"
             "service-navigation-rbnx/rbnx-build/data/trajectories")

f = sys.argv[1] if len(sys.argv) > 1 else sorted(
    (p for p in glob.glob(os.path.join(TRACE_DIR, "*.jsonl")) if "scan-anomalies" not in p),
    key=os.path.getmtime,
)[-1]
rows = [json.loads(line) for line in open(f) if line.strip()]
cmd = [r for r in rows if r.get("event") == "cmd"]
odom = [r for r in rows if r.get("event") == "odom"]
if not cmd or not odom:
    raise SystemExit(f"{os.path.basename(f)}: 缺少 cmd/odom 样本")

# 只统计「真的有速度指令」的窗口，避免把停下后的 odom 漂移算进去。
moving = [c for c in cmd if abs(c["linear"]) > 0.02 or abs(c["angular"]) > 0.01]
seg = moving or cmd
t_lo, t_hi = seg[0]["t_wall"], seg[-1]["t_wall"]
win = [o for o in odom if t_lo <= o["t_wall"] <= t_hi] or odom

path = sum(math.hypot(b["pose"][0] - a["pose"][0], b["pose"][1] - a["pose"][1])
           for a, b in zip(win, win[1:]))
cmd_dist = sum(abs(a["linear"]) * (b["t_wall"] - a["t_wall"]) for a, b in zip(seg, seg[1:]))

w = [abs(c["angular"]) for c in cmd]
nz = sum(1 for x in w if x > 0.01)
buckets = {"0": 0, "0-0.22": 0, "0.22-0.35": 0, "0.35-0.6": 0, ">0.6": 0}
for x in w:
    if x <= 0.01:
        buckets["0"] += 1
    elif x < 0.22:
        buckets["0-0.22"] += 1
    elif x < 0.35:
        buckets["0.22-0.35"] += 1
    elif x <= 0.6:
        buckets["0.35-0.6"] += 1
    else:
        buckets[">0.6"] += 1

print(f"轨迹        : {os.path.basename(f)}")
print(f"运动窗      : {t_hi - t_lo:.1f} s   指令样本 {len(cmd)}")
print(f"转角指令占比: {100.0 * nz / len(cmd):.0f}%   （|ω|>0.01）")
print(f"odom 里程   : {path:.2f} m   指令积分 {cmd_dist:.2f} m   膨胀比 {path / max(cmd_dist, 1e-6):.2f}")
print("|ω| 分布    : " + "  ".join(f"{k}={100.0 * v / len(cmd):.0f}%" for k, v in buckets.items()))
print(f"平均线速度  : {sum(c['linear'] for c in cmd) / len(cmd):.3f} m/s")
