#!/usr/bin/env python3
"""给当前清单里的观察位算一下“工作区投影落在像面中央的比例”，用来定预筛阈值。"""
import math
import os
import sys

import numpy as np
import yaml

DEPLOY = "/home/dog/robot-deep_robotics-lynx_s10"
SKILL = os.path.join(DEPLOY, "rbnx-boot/cache/skill-pick-depth-grasp-rbnx")
sys.path[:0] = [os.path.join(SKILL, "grasp_skill", "calib"), SKILL]

from grasp_skill.calib.calib_io import Calibration  # noqa: E402
from grasp_skill import look_gate  # noqa: E402

with open(os.path.join(DEPLOY, "robonix_manifest.nx.yaml"), "r", encoding="utf-8") as handle:
    manifest = yaml.safe_load(handle)
skill_cfg = manifest["skill"][0]["config"]
calib = Calibration.from_numbers(skill_cfg["calib"])
if not hasattr(calib, "base_pose") or calib.base_pose is None:
    calib.base_pose = np.eye(4)
cfg = look_gate.merged_config(skill_cfg.get("look_gate"))
surface_z = float(skill_cfg["table_height_m"])
gate = look_gate.ViewGate(calib, cfg)
K, (width, height) = gate._matrix()
print("image %dx%d  fx=%.1f fy=%.1f" % (width, height, K[0, 0], K[1, 1]))

poses = [("nominal", skill_cfg["calib"]["observe_joints_deg"])]
for index, joints in enumerate(skill_cfg["observe_scan"].get("anchors_deg") or []):
    poses.append(("anchor%d" % (index + 1), joints))

for band in (0.5, 0.7, 0.8):
    print("\n== central band %.0f%% ==" % (band * 100))

for name, joints in poses:
    camera = gate._camera_in_base(joints)
    low, high = cfg["radius_m"]
    step = cfg["grid_m"]
    radii = np.arange(low, high + 1e-9, step)
    count = max(16, int(2 * math.pi * high / step))
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    rr, tt = np.meshgrid(radii, angles, indexing="ij")
    points = np.column_stack([(rr * np.cos(tt)).ravel(), (rr * np.sin(tt)).ravel(),
                              np.full(rr.size, surface_z)])
    inverse = np.linalg.inv(camera)
    cam = (inverse @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]
    depth = cam[:, 2]
    forward = np.isfinite(depth) & (depth > 0.05) & (depth < 2.0)
    cam = cam[forward]
    uv = (K @ cam.T).T
    u = uv[:, 0] / uv[:, 2]
    v = uv[:, 1] / uv[:, 2]
    margin = float(cfg["margin_px"])
    inside = ((u >= margin) & (u <= width - 1 - margin)
              & (v >= margin) & (v <= height - 1 - margin))
    u, v = u[inside], v[inside]
    if not len(u):
        print("  %-8s 不可见" % name)
        continue
    cu, cv = float(np.mean(u)) / width, float(np.mean(v)) / height
    ratios = {}
    for band in (0.5, 0.7, 0.8):
        half = band / 2.0
        central = ((u / width >= 0.5 - half) & (u / width <= 0.5 + half)
                   & (v / height >= 0.5 - half) & (v / height <= 0.5 + half))
        ratios[band] = float(np.count_nonzero(central)) / float(len(u))
    print("  %-8s pts=%4d  centroid=(%.2f,%.2f)  central50=%.2f central70=%.2f central80=%.2f"
          % (name, len(u), cu, cv, ratios[0.5], ratios[0.7], ratios[0.8]))
