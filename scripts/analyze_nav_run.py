#!/usr/bin/env python3
"""分析最近一次导航（或指定）轨迹，量化「直线程度」。

指标：
  * 转角指令占比：|ω| > 0.01 的指令样本比例（越低越"少抬腿"）
  * 里程膨胀比：odom 走过的路径长度 / 指令速度积分（≈1 说明轮子一路滚；
    明显 >1 说明在抖/抬腿，2026-11-16 11:07 那趟是 1.8）
  * 角度分布：|ω| 的直方桶

用法：python3 scripts/analyze_nav_run.py [轨迹文件名]
"""
import glob
import json
import math
import os
import sys

TRACE_DIR = "/home/dog/robot-deep_robotics-lynx_s10/rbnx-boot/cache/service-navigation-rbnx/rbnx-build/data/trajectories"

f = sys.argv[1] if len(sys.argv) > 1 else sorted(
    (p for p in glob.glob(os.path.join(TRACE_DIR, "*.jsonl")) if "scan-anomalies" not in p),
    key=os.path.getmtime,
)[-1]
rows = [json.loads(l) for l in open(f) if l.strip()]
cmd = [r for r in rows if r.get("event") == "cmd"]
odom = [r for r in rows if r.get("event") == "odom"]
if not cmd or not odom:
    raise SystemExit(f"{os.path.basename(f)}: 缺少 cmd/odom 样本")

path = 0.0
# 只统计「有运动指令」的时间窗，避免把停下后的 odom 漂移算进里程。
t_lo, t_hi = cmd[0]["t_wall"], cmd[-1]["t_wall"]
win = [o for o in odom if t_lo <= o["t_wall"] <= t_hi]
if len(win) < 2:
    win = odom
for a, b in zip(win, win[1:]):
    path += math.hypot(b["pose"][0] - a["pose"][0], b["pose"][1] - a["pose"][1])
cmd_dist = sum(abs(a["linear"]) * (b["t_wall"] - a["t_wall"]) for a, b in zip(cmd, cmd[1:]))

turning = [abs(c["angular"]) for c in cmd]
nz = sum(1 for w in turning if w > 0.01)
buckets = {"0": 0, "0-0.22": 0, "0.22-0.35": 0, "0.35-0.6": 0, ">0.6": 0}
for w in turning:
    if w <= 0.01:
        buckets["0"] += 1
    elif w < 0.22:
        buckets["0-0.22"] += 1
    elif w < 0.35:
        buckets["0.22-0.35"] += 1
    elif w <= 0.6:
        buckets["0.35-0.6"] += 1
    else:
        buckets[">0.6"] += 1

dur = cmd[-1]["t_wall"] - cmd[0]["t_wall"]
print(f"轨迹      : {os.path.basename(f)}")
print(f"时长      : {dur:.1f} s   指令样本 {len(cmd)}   里程样本 {len(odom)}")
print(f"转角指令占比: {100.0*nz/len(cmd):.0f}%   （|ω|>0.01 的样本）")
print(f"odom 里程 : {path:.2f} m   指令积分 {cmd_dist:.2f} m   膨胀比 {path/max(cmd_dist,1e-6):.2f}")
print("|ω| 分布  : " + "  ".join(f"{k}={100.0*v/len(cmd):.0f}%" for k, v in buckets.items()))
print(f"平均线速度: {sum(c['linear'] for c in cmd)/len(cmd):.3f} m/s")
