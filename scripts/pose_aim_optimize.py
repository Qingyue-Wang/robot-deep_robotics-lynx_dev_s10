#!/usr/bin/env python3
"""给观察位找“看正下方典型抓取带”的俯仰修正：围绕原点位搜索 J5/J2 的小偏移，
让 底座前方 r=0.40–0.70 m 的环带尽量落在像面中央 70% 区域里。

用法:
  python3 scripts/pose_aim_optimize.py            # 打印每个观察位的建议
  python3 scripts/pose_aim_optimize.py --json     # 只打印可粘贴的 joints 列表
"""
import json
import os
import sys

import numpy as np
import yaml

DEPLOY = "/home/dog/robot-deep_robotics-lynx_s10"
SKILL = os.path.join(DEPLOY, "rbnx-boot/cache/skill-pick-depth-grasp-rbnx")
sys.path[:0] = [os.path.join(SKILL, "grasp_skill", "calib"), SKILL]

from grasp_skill.calib.calib_io import Calibration  # noqa: E402
from grasp_skill import look_gate  # noqa: E402

BAND_R = (0.40, 0.70)
BAND_YAW = (-45.0, 45.0)
CENTER = 0.70          # 像面中央 70%
LIMITS = [(-150, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120)]

with open(os.path.join(DEPLOY, "robonix_manifest.nx.yaml"), "r", encoding="utf-8") as handle:
    manifest = yaml.safe_load(handle)
skill_cfg = manifest["skill"][0]["config"]
calib = Calibration.from_numbers(skill_cfg["calib"])
calib.base_pose = np.eye(4)
cfg = look_gate.merged_config(skill_cfg.get("look_gate"))
surface_z = float(skill_cfg["table_height_m"])
gate = look_gate.ViewGate(calib, cfg)
K, (width, height) = gate._matrix()

radii = np.arange(BAND_R[0], BAND_R[1] + 1e-9, 0.02)
yaws = np.deg2rad(np.linspace(BAND_YAW[0], BAND_YAW[1], 19))
rr, yy = np.meshgrid(radii, yaws, indexing="ij")
BAND = np.stack([rr * np.cos(yy), rr * np.sin(yy),
                      np.full(rr.shape, surface_z)], axis=-1).reshape(-1, 3)


def evaluate(joints_deg):
    camera = gate._camera_in_base(joints_deg)
    inverse = np.linalg.inv(camera)
    cam = (inverse @ np.column_stack([BAND, np.ones(len(BAND))]).T).T[:, :3]
    front = cam[:, 2] > 0.05
    if np.count_nonzero(front) < 0.25 * len(BAND):
        return -1.0, 0.0, float("nan"), float("nan")
    uv = (K @ cam[front].T).T
    u = uv[:, 0] / uv[:, 2]
    v = uv[:, 1] / uv[:, 2]
    inside = (u >= 0) & (u <= width - 1) & (v >= 0) & (v <= height - 1)
    if not np.any(inside):
        return 0.0, 0.0, float("nan"), float("nan")
    half = CENTER / 2.0
    central = inside & (u / width >= 0.5 - half) & (u / width <= 0.5 + half) \
        & (v / height >= 0.5 - half) & (v / height <= 0.5 + half)
    ratio = float(np.count_nonzero(central)) / float(len(BAND))
    coverage = float(np.count_nonzero(inside)) / float(len(BAND))
    return ratio, coverage, float(np.min(v[inside]) / height), float(np.max(v[inside]) / height)


def optimize(joints):
    best = None
    for delta5 in range(-45, 46, 5):
        for delta2 in range(-25, 26, 5):
            candidate = list(joints)
            candidate[4] += delta5
            candidate[1] += delta2
            if any(not (low <= value <= high) for value, (low, high) in zip(candidate, LIMITS)):
                continue
            ratio, coverage, vmin, vmax = evaluate(candidate)
            if coverage < 0.5:
                continue
            score = ratio - 0.0015 * (abs(delta5) + abs(delta2))
            if best is None or score > best[0]:
                best = (score, candidate, ratio, coverage, vmin, vmax, delta5, delta2)
    return best


poses = [("标称位", list(skill_cfg["calib"]["observe_joints_deg"]))]
for index, joints in enumerate(skill_cfg["observe_scan"].get("anchors_deg") or []):
    poses.append(("锚点%d" % (index + 1), list(joints)))

result = {}
print("目标：r=0.40–0.70 m、|yaw|<=45° 的环带落进像面中央 %.0f%%" % (CENTER * 100))
for name, joints in poses:
    ratio, coverage, vmin, vmax = evaluate(joints)
    print("\n%-6s 原点位 %s" % (name, [round(float(v), 2) for v in joints]))
    print("        现状  中央占比=%.2f 入画=%.2f v范围=[%.2f,%.2f]"
          % (ratio, coverage, vmin, vmax))
    best = optimize(joints)
    if best is None:
        print("        没找到能覆盖 >=50%% 环带的修正")
        continue
    _score, candidate, ratio2, coverage2, vmin2, vmax2, d5, d2 = best
    print("        建议  %s  (ΔJ5=%+d°, ΔJ2=%+d°)"
          % ([round(float(v), 2) for v in candidate], d5, d2))
    print("              中央占比=%.2f 入画=%.2f v范围=[%.2f,%.2f]"
          % (ratio2, coverage2, vmin2, vmax2))
    result[name] = [round(float(v), 2) for v in candidate]

if "--json" in sys.argv:
    print(json.dumps(result, ensure_ascii=False))
