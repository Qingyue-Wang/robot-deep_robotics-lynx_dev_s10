#!/usr/bin/env python3
"""预筛阈值探针 v2：只看“典型抓取带”（底座前方 r=0.40–0.60 m、|yaw|<=40°）
在像面里的位置，判断这个位姿会不会把目标顶到画面边缘。"""
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
calib.base_pose = np.eye(4)
cfg = look_gate.merged_config(skill_cfg.get("look_gate"))
surface_z = float(skill_cfg["table_height_m"])
gate = look_gate.ViewGate(calib, cfg)
K, (width, height) = gate._matrix()

poses = [("nominal", skill_cfg["calib"]["observe_joints_deg"])]
for index, joints in enumerate(skill_cfg["observe_scan"].get("anchors_deg") or []):
    poses.append(("anchor%d" % (index + 1), joints))

for band in (0.6, 0.7, 0.8):
    print("== central band %.0f%% of image ==" % (band * 100))
    for name, joints in poses:
        camera = gate._camera_in_base(joints)
        radii = np.arange(0.40, 0.60001, 0.02)
        yaws = np.linspace(-40.0, 40.0, 17)
        rr, yy = np.meshgrid(radii, np.deg2rad(yaws), indexing="ij")
        points = np.stack([rr * np.cos(yy), rr * np.sin(yy),
                          np.full(rr.shape, surface_z)], axis=-1).reshape(-1, 3)
        inverse = np.linalg.inv(camera)
        cam = (inverse @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]
        if np.any(cam[:, 2] <= 0):
            print("  %-8s 有点在相机后面" % name)
            continue
        uv = (K @ cam.T).T
        u = uv[:, 0] / uv[:, 2]
        v = uv[:, 1] / uv[:, 2]
        inside = ((u >= 0) & (u <= width - 1) & (v >= 0) & (v <= height - 1))
        half = band / 2.0
        central = inside & (u / width >= 0.5 - half) & (u / width <= 0.5 + half) \
            & (v / height >= 0.5 - half) & (v / height <= 0.5 + half)
        print("  %-8s 带内 %3d/%3d 入画 中央占比=%.2f  v范围=[%.2f,%.2f]"
              % (name, int(np.count_nonzero(inside)), len(u),
                 float(np.count_nonzero(central)) / max(1, int(np.count_nonzero(inside))),
                 float(np.min(v[inside]) / height) if np.any(inside) else float("nan"),
                 float(np.max(v[inside]) / height) if np.any(inside) else float("nan")))
    print()
