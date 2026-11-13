#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3D 手眼标定工作台 —— 固定 ChArUco 板 + AX=XB + 渐进多轮 + 内参共标 + 臂几何重拟合 + 新位姿验证

这是 scripts/handeye_web.py 的 3D 版：那条工具（模式 B）让操作员在画面上点同一个物点，用
"未知世界点 + 交替 Kabsch" 解 X，误差来源多、重复性差（本机实测半小时内两次解相差 55 mm）。
本工具改成 .168 那套已经过验证的方法，并补上它没有在本机跑通的环节：

  1. 板子固定不动，相机刚性装在腕上。每个机位：
        T_base_board = T_base_gripper(q_i) · X · T_cam_board,i
     板没动，所以 N 个 T_base_board 必须一样。解 X 就是解 AX = XB（cv2.calibrateHandEye）。
  2. 位姿按三轮渐进扩散采集（绕观察位，逐轮把关节张开）：
        R1 ±[12,10,10,20, 0,25]°   R2 ±[35,30,30,50,20,60]°   R3 ±[55,45,45,70,35,90]°
     每轮 18 个机位，**每轮单独求解、不混合轮次**，这样能看出"位姿张得越开，X 越稳"。
  3. 每张照片都过门限：>=30 个 ChArUco 内角点、在画面内留 5% 边、斜视 <=50°、
     重投影 RMS <=1.5 px、臂真的停稳（关节峰峰值 <=0.05°、角点抖动 <=1.0 px、窗口 1 s）。
     停稳是必需的：抖的时候"臂上报的位姿"和"相机曝光那一瞬"对不上，残差还看不出来。
  4. 内参和手眼用同一批照片共标（charuco.calibrate_intrinsics），标完回到存下的角点重解
     板位姿再解 X —— 这样位姿解算与运行时深度反投影用的是同一份 K/dist。
  5. 解法交叉检查：TSAI/PARK/HORAUD/DANIILIDIS 四种都解，互相对不上就说明数据有问题；
     ANDREFF 不参与（不保旋转正交，干净数据上单独偏 28.5 mm，见 common/handeye3d.py 注释）。
  6. 臂几何重拟合：板位姿的散布里有一部分是"URDF 与实物不一致"造成的。放开 J1–J5 零点与
     J2–J5 位置修正去压散布，必须过三道门（散布确实下降、参数量级合理、隔一取一分半验证
     改善 >=35%）才采用；采用后重解 X。J1 平移与 J6 被 X 吸收，不参与拟合。
  7. 验证用**没参与解算的新位姿**：看同一块板的位姿散布是否还在预算内；点画面上一点，
     用 T_base_flange(q_corrected)·X 换到 base 系，悬停 2 cm 上方看残差；再量一次实物。
  8. 产物是一份报告 + 可直接粘回 robonix_manifest.nx.yaml 的 YAML 片段（gripper_camera、
     correction；内参另给 grasp_geometry 的 camera_matrix/dist_coeffs）。--apply 才会写清单，
     并且先备份、写前重读（本仓库有别的会话在改清单）。

运行（NX 上，ROS 栈要先起来；不要和 8770 的 handeye_web 抢臂）：
    cd ~/robot-deep_robotics-lynx_s10
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=43
    python3 scripts/handeye3d_web.py --port 8771
浏览器打开 http://<NX-IP>:8771/ ，板上标定板，按页面顺序：检测 → 采集三轮 → 解算 →
内参 → 几何 → 验证 → 生成清单。--dry-run 只读不动作；--selftest 用合成数据自检；
--probe 只读实机（不动臂）。

安全：沿用 handeye_web 的工作空间软限位；采集只发关节角（move_joints_deg），不走直线插值；
每个候选位姿都先夹进 URDF 限位（J5 的观察角 69.99° 正好贴着 ±69.9° 限位，见
common/robot/ik.py:clamp_joints_deg 的注释），单步超限直接跳过。
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
GRASP_DIR = ROOT / "rbnx-boot" / "cache" / "skill-pick-depth-grasp-rbnx" / "grasp_skill"
CALIB_DIR = GRASP_DIR / "calib"

# handeye_web 里有现成的 ROS 桥（Bridge）、清单读取与 FK 链；common/* 是 3D 手眼与几何的数学。
# 注意顺序：grasp_skill 也要在 sys.path 里，因为 common/robot/ik.py 会 import lm_ik，
# 而打包后的 lm_ik.py 直接躺在 grasp_skill/ 下（源码树里是 grasp_tilt/lm_ik.py）。
for _p in (str(GRASP_DIR), str(CALIB_DIR), str(SCRIPTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import handeye_web as hw  # noqa: E402  Bridge / load_manifest_cfg / find_urdf / rot_deg
from common import arm_geometry, charuco, handeye3d  # noqa: E402
from common.robot import ik as ik_mod, kinematics  # noqa: E402

# 三轮渐进扩散：绕观察位的关节角幅度（度）。与 .168 tools/calibration/eyeinhand/3d_calibration.md 一致。
ROUNDS = [
    {"tag": "R1", "delta_deg": [12, 10, 10, 20, 0, 25]},
    {"tag": "R2", "delta_deg": [35, 30, 30, 50, 20, 60]},
    {"tag": "R3", "delta_deg": [55, 45, 45, 70, 35, 90]},
]

# 采集门限与验收线。改这里之前先想清楚：数值来自 .168 的实测报告与 calib 包里的注释。
GATES = {
    "min_corners": 30,          # 每张至少 30 个内角点（板共 11x8=88 个）
    "view_max_deg": 50.0,       # 斜视超过 50° 的图不用（.168 规则）
    "view_warn_deg": 15.0,      # 几乎正对时只警告（旋转信息少）
    "rms_max_px": 1.5,          # 单张重投影 RMS
    "border_frac": 0.05,        # 角点必须落在画面内 95% 区域
    "settle_joint_deg": 0.05,   # 与 calib/common/settle.py 相同
    "settle_corner_px": 1.0,
    # 本机实测彩色帧率约 5 fps（GIL + 两个订阅者），窗口取 2 s 才能攒到 >=8 帧
    "settle_window_s": 2.0,
    "settle_min_frames": 8,
    "settle_timeout_s": 20.0,
    # 验收线（板在 base 下的散布就是这套标定的真实误差）
    "accept_pos_rms_mm": 3.0,
    "accept_pos_max_mm": 5.0,
    "accept_angle_rms_deg": 0.5,
    "accept_loo_ratio": 2.0,    # 去一留一的位置变化 <= 2 x RMS
    "accept_method_mm": 1.0,    # 四种解法互差
    "accept_method_deg": 0.1,
    "min_poses": 8,             # 少于 8 个位姿只出报告，不给结论
}


# ------------------------------------------------------------------ 小工具 ----

def as_T(value) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(4, 4)


def mean_transform(transforms: list) -> np.ndarray:
    """一组 4x4 的均值：旋转用 SVD 投回 SO(3)，平移取算术平均。"""
    transforms = [as_T(t) for t in transforms]
    rotation = handeye3d._mean_rotation([t[:3, :3] for t in transforms])
    translation = np.mean([t[:3, 3] for t in transforms], axis=0)
    return handeye3d.make_transform(rotation, translation)


def jsonable(value):
    """numpy / Path → JSON 能写的东西。"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def load_cfg(manifest: Path) -> dict:
    """清单 + 臂几何修正 + 内参来源。比 handeye_web.load_manifest_cfg 多取 correction。"""
    doc = yaml.safe_load(Path(manifest).read_text(encoding="utf-8")) or {}
    cfg = hw.load_manifest_cfg(manifest)
    skills = [it for it in (doc.get("skill") or []) if isinstance(it, dict) and it.get("name") == "depth_grasp"]
    calib = ((skills[0].get("config") if skills else {}) or {}).get("calib") or {}
    correction = calib.get("correction") or {}
    zeros = list(correction.get("zeros_deg") or [0.0] * 6)
    offsets = [list(r) for r in (correction.get("offsets_m") or [[0.0] * 3] * 6)]
    while len(zeros) < 6:
        zeros.append(0.0)
    while len(offsets) < 6:
        offsets.append([0.0, 0.0, 0.0])
    cfg["correction"] = {"zeros_deg": zeros[:6], "offsets_m": offsets[:6], "source": correction.get("source", "manifest")}
    svcs = [it for it in (doc.get("service") or []) if isinstance(it, dict) and it.get("name") == "grasp_geometry"]
    svc_cfg = (svcs[0].get("config") if svcs else {}) or {}
    cfg["k_declared"] = bool(svc_cfg.get("camera_matrix"))
    cfg["k_manifest"] = np.asarray(svc_cfg.get("camera_matrix"), dtype=float).reshape(3, 3) if svc_cfg.get("camera_matrix") else None
    cfg["dist_manifest"] = np.asarray(svc_cfg.get("dist_coeffs") or [], dtype=float)
    cfg["urdf"] = str(hw.find_urdf(cfg))
    return cfg


def urdf_text(cfg: dict) -> tuple[str, Path]:
    path = hw.find_urdf(cfg)
    text = re.sub(r"<\?xml[^?]*\?>", "", Path(path).read_text(encoding="utf-8"), count=1)
    return text, Path(path)


# ------------------------------------------------------------------ 任务队列 ----

class Job:
    """一次长任务（采集/解算）放后台线程跑，网页轮询进度；同一时间只跑一个。"""

    def __init__(self, name: str, fn):
        self.name = name
        self.lines: list[str] = []
        self.result = None
        self.error = None
        self.running = True
        self.started = time.time()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, args=(fn,), daemon=True)
        self._thread.start()

    def log(self, message: str):
        with self._lock:
            self.lines.append(f"{time.strftime('%H:%M:%S')} {message}")
            if len(self.lines) > 400:
                del self.lines[:100]

    def _run(self, fn):
        try:
            self.result = fn(self.log)
        except Exception as exc:  # noqa: BLE001  后台任务的异常要在网页上看到，不能吞
            import traceback
            self.error = f"{type(exc).__name__}: {exc}"
            self.log("失败：" + self.error)
            self.log(traceback.format_exc(limit=3))
        finally:
            self.running = False

    def snapshot(self) -> dict:
        with self._lock:
            return {"name": self.name, "running": self.running,
                    "elapsed_s": round(time.time() - self.started, 1),
                    "lines": list(self.lines[-200:]), "result": jsonable(self.result),
                    "error": self.error}


# ------------------------------------------------------------------ 标定会话 ----

class Calib:
    def __init__(self, bridge, cfg: dict, board: charuco.Board, out_root: Path,
                 apply_allowed: bool = False, session_dir: Path | None = None):
        self.bridge = bridge
        self.cfg = cfg
        self.board = board
        self.apply_allowed = apply_allowed
        if session_dir is not None:
            # 续跑已有会话：保留样本/内参/报告，不新开目录
            self.dir = Path(session_dir)
            self.stamp = self.dir.name
        else:
            self.stamp = time.strftime("%Y%m%d_%H%M%S")
            self.dir = Path(out_root) / self.stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self.K = np.asarray(cfg["K"], dtype=float).reshape(3, 3)
        _dist = cfg.get("dist")
        self.dist = np.asarray([] if _dist is None else _dist, dtype=float).reshape(-1)
        self.correction = copy.deepcopy(cfg["correction"])
        self.model = arm_geometry.ArmModel(urdf_text(cfg)[0], ik_mod.END_LINK)
        self.limits = ik_mod.joint_limits_deg()
        self.samples: list[dict] = []
        self.reports: dict[str, str] = {}
        self.results: dict[str, dict] = {}
        self.final = None          # 最终解算结果（含 X）
        self.job: Job | None = None
        self.solve_cache: dict[str, dict] = {}
        self.last_solution: dict | None = None
        self.last_scale = 1.0
        self.image_size = (1280, 720)
        self.lock = threading.Lock()
        self.log_lines: list[str] = []
        self._load_state()

    # ---- 落盘/恢复：每采一条就写，断点不丢 ----
    def _state_path(self) -> Path:
        return self.dir / "session.json"

    def _save_state(self):
        data = {
            "stamp": self.stamp, "manifest": self.cfg["manifest"],
            "board": {"squares_x": self.board.squares_x, "squares_y": self.board.squares_y,
                      "square_len_m": self.board.square_len_m, "marker_len_m": self.board.marker_len_m,
                      "dictionary": self.board.dictionary_name},
            "K": self.K.tolist(), "dist": self.dist.tolist(),
            "correction": self.correction,
            "samples": self.samples,
            "reports": self.reports,
            "final": jsonable(self.final) if self.final else None,
        }
        tmp = self._state_path().with_suffix(".tmp")
        tmp.write_text(json.dumps(jsonable(data), ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(self._state_path())

    def _load_state(self):
        path = self._state_path()
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        self.samples = data.get("samples") or []
        self.reports = data.get("reports") or {}
        self.final = data.get("final")
        if data.get("K") is not None:
            self.K = np.asarray(data["K"], dtype=float)
        if data.get("dist") is not None:
            self.dist = np.asarray(data["dist"], dtype=float).reshape(-1)
        if data.get("correction"):
            self.correction = data["correction"]

    def log(self, message: str):
        line = f"{time.strftime('%H:%M:%S')} {message}"
        with self.lock:
            self.log_lines.append(line)
            if len(self.log_lines) > 500:
                del self.log_lines[:200]
        if self.job is not None:
            self.job.log(message)
        print(line, flush=True)

    # ---- 相机/板 ----
    def detect_board(self, color) -> dict | None:
        if color is not None:
            self.image_size = (int(color.shape[1]), int(color.shape[0]))
        """在 BGR 图上找 ChArUco 并解 T_cam_board。不合格的返回带 ok=False 的字典。"""
        if color is None:
            return None
        corners, ids = self.board.detect(color)
        if corners is None or ids is None or len(ids) == 0:
            return {"ok": False, "why": "没检出 ChArUco 角点", "n_corners": 0}
        transform, rms, note = charuco.estimate_pose(self.board, corners, ids, self.K, self.dist,
                                                     min_corners=GATES["min_corners"])
        view = charuco.viewing_angle_deg(transform) if transform is not None else float("nan")
        points = np.asarray(corners, dtype=float).reshape(-1, 2)
        h, w = color.shape[:2]
        margin = GATES["border_frac"]
        inside = bool(points[:, 0].min() >= w * margin and points[:, 0].max() <= w * (1 - margin)
                      and points[:, 1].min() >= h * margin and points[:, 1].max() <= h * (1 - margin))
        info = {"ok": transform is not None, "n_corners": int(len(ids)), "rms_px": float(rms),
                "view_deg": float(view), "inside": inside, "note": note,
                "corners": points.tolist(), "ids": [int(v) for v in np.asarray(ids).ravel()],
                "center_px": points.mean(axis=0).tolist(),
                "T_cam_board": None if transform is None else transform.tolist()}
        problems = []
        if transform is None:
            problems.append(note)
        else:
            if not inside:
                problems.append(f"板贴边（要留 {margin * 100:.0f}% 边）")
            if view > GATES["view_max_deg"]:
                problems.append(f"斜视 {view:.0f}° > {GATES['view_max_deg']:.0f}°")
            if rms > GATES["rms_max_px"]:
                problems.append(f"重投影 {rms:.2f}px > {GATES['rms_max_px']:.1f}px")
        info["ok"] = not problems
        info["why"] = "；".join(problems)
        return info

    # ---- 停稳 ----
    @staticmethod
    def _corner_steadiness(window: list) -> float:
        """窗口里角点相对自身均值的 RMS（按编号配对，取中位数）。与 calib/common/settle.py 同一判据。"""
        track: dict[int, list] = {}
        for pairs in window:
            if not pairs:
                continue
            for index, point in zip(pairs[0], pairs[1]):
                track.setdefault(int(index), []).append(point)
        need = max(3, int(len(window) * 0.5))
        deviations = []
        for points in track.values():
            if len(points) < need:
                continue
            array = np.asarray(points, dtype=float)
            deviations.append(float(np.sqrt(((array - array.mean(axis=0)) ** 2).sum(axis=1).mean())))
        return float(np.median(deviations)) if deviations else float("nan")

    def _settle(self, timeout_s: float | None = None) -> dict:
        timeout_s = timeout_s or GATES["settle_timeout_s"]
        start = time.time()
        last_stamp, frame_index = None, 0
        window, last_pairs, last_corners = [], None, None
        calmest_joint, calmest_corner = float("inf"), float("inf")
        while time.time() - start < timeout_s:
            color, _depth, stamp, _d = self.bridge.snap_frame()
            if color is None or stamp == last_stamp:
                time.sleep(0.01)
                continue
            last_stamp = stamp
            frame_index += 1
            joints = self.bridge.joints_deg_now()
            if joints is None:
                continue
            # 本机只有 4–6 fps，每帧都检测，否则 2 s 窗口里凑不出足够的配对帧
            pairs = None
            corners, ids = self.board.detect(color)
            if corners is not None:
                pairs = (np.asarray(ids).ravel(), np.asarray(corners).reshape(-1, 2))
                last_pairs, last_corners = pairs, (corners, ids)
            window.append((time.time(), np.asarray(joints, dtype=float), pairs or last_pairs))
            window = [item for item in window if time.time() - item[0] <= GATES["settle_window_s"]]
            if len(window) < GATES["settle_min_frames"]:
                continue
            if window[-1][0] - window[0][0] < GATES["settle_window_s"] * 0.7:
                continue
            joint_ptp = float(np.ptp(np.stack([item[1] for item in window]), axis=0).max())
            corner_rms = self._corner_steadiness([item[2] for item in window])
            calmest_joint = min(calmest_joint, joint_ptp)
            if not math.isnan(corner_rms):
                calmest_corner = min(calmest_corner, corner_rms)
            if joint_ptp <= GATES["settle_joint_deg"] and (math.isnan(corner_rms) or corner_rms <= GATES["settle_corner_px"]):
                # 停稳后现拍一张，用这一帧解位姿
                fresh = self.bridge.snap_frame()[0]
                if fresh is not None:
                    corners, ids = self.board.detect(fresh)
                    if corners is not None:
                        last_corners = (corners, ids)
                return {"ok": True, "color": fresh, "corners": last_corners[0] if last_corners else None,
                        "ids": last_corners[1] if last_corners else None,
                        "joint_ptp_deg": joint_ptp, "corner_rms_px": corner_rms,
                        "waited_s": time.time() - start}
        return {"ok": False, "why": "没停稳（关节最稳 %.3f° / 角点最稳 %.2fpx）" % (calmest_joint, calmest_corner),
                "joint_ptp_deg": calmest_joint, "corner_rms_px": calmest_corner,
                "waited_s": time.time() - start}

    # ---- 采集 ----
    def _base_joints(self) -> np.ndarray:
        """观察位夹进 URDF 限位（清单里的 J5=69.99° 正好贴着 ±69.90°，不夹会算出空区间）。"""
        lo = np.array([item[0] for item in self.limits], dtype=float) + 2.0
        hi = np.array([item[1] for item in self.limits], dtype=float) - 2.0
        return np.clip(np.asarray(self.cfg["observe_joints_deg"], dtype=float), lo, hi)

    def _prior(self):
        """用上一轮解出来的 (X, T_base_board) 预判板还在不在画面里；没有就先不筛。"""
        result = self.last_solution
        if not result or result.get("X") is None:
            return None
        samples = [s for s in self.samples if s.get("round") == result.get("round")]
        if len(samples) < 6:
            return None
        flanges = self.flanges(samples)
        boards = [np.asarray(s["T_cam_board"], dtype=float) for s in samples]
        poses = handeye3d.board_in_base(np.asarray(result["X"], dtype=float), flanges, boards)
        return np.asarray(result["X"], dtype=float), mean_transform(poses)

    def _predicted_ok(self, joints: np.ndarray, prior) -> bool:
        """按当前手眼解把板四角投到图像上：还在画面里、距离和斜视都合适才去动。"""
        X, T_base_board = prior
        correction = self.correction or kinematics.identity()
        corrected = kinematics.corrected_joints(np.asarray(joints, dtype=float), correction)
        T_flange = np.asarray(self.model.forward(correction["offsets_m"])(corrected), dtype=float)
        T_cam = np.linalg.inv(T_flange @ X)
        half_x = self.board.squares_x * self.board.square_len_m / 2.0
        half_y = self.board.squares_y * self.board.square_len_m / 2.0
        corners = np.array([[-half_x, -half_y, 0.0], [half_x, -half_y, 0.0],
                            [half_x, half_y, 0.0], [-half_x, half_y, 0.0]])
        ones = np.ones((4, 1))
        points = np.hstack([corners, ones])
        points = (T_cam @ (T_base_board @ points.T)).T[:, :3]
        if np.any(points[:, 2] < 0.30) or np.any(points[:, 2] > 1.00):
            return False
        width, height = self.image_size
        uv = (self.K @ points.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        center = uv.mean(axis=0)
        # 大角度机位允许板子露出一部分：中心落在中间 70% 区域、至少两个角在画面内即可，
        # 真正的门槛交给拍照后的 ≥30 角点检测。
        if not (0.15 * width <= center[0] <= 0.85 * width and 0.15 * height <= center[1] <= 0.85 * height):
            return False
        inside = ((uv[:, 0] >= 0.01 * width) & (uv[:, 0] <= 0.99 * width) &
                  (uv[:, 1] >= 0.01 * height) & (uv[:, 1] <= 0.99 * height))
        if int(inside.sum()) < 2:
            return False
        normal = T_cam[:3, :3] @ np.array([0.0, 0.0, 1.0])
        cosine = abs(float(normal @ np.array([0.0, 0.0, 1.0])))
        angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        return angle <= GATES["view_max_deg"]

    def _targets(self, round_index: int, count: int, prior=None) -> list[np.ndarray]:
        base = self._base_joints()
        delta = np.asarray(ROUNDS[round_index]["delta_deg"], dtype=float)
        lo_hard = np.array([item[0] for item in self.limits], dtype=float) + 2.0
        hi_hard = np.array([item[1] for item in self.limits], dtype=float) - 2.0
        from scipy.stats import qmc
        engine = qmc.Sobol(d=6, scramble=True, seed=20261112 + round_index * 97)
        targets: list[np.ndarray] = []
        # 板只有 180x135 mm、臂又短：大范围机位很容易把板甩出画面。
        # 先用名义扩散，预筛不够就按 0.75 / 0.55 / 0.42 / 0.32 缩小，并把实际用的比例记下来。
        chosen_scale = 0.0
        for scale in (1.0, 0.75, 0.55, 0.42, 0.32):
            lo = np.maximum(base - delta * scale, lo_hard)
            hi = np.minimum(base + delta * scale, hi_hard)
            if np.any(hi < lo - 0.001):
                continue
            draw = max(count * 3, 24)
            for target in lo + (hi - lo) * engine.random(draw):
                if prior is not None and not self._predicted_ok(target, prior):
                    continue
                if all(float(np.max(np.abs(target - other))) >= 2.5 for other in targets):
                    targets.append(target)
                if len(targets) >= count:
                    break
            if targets:
                chosen_scale = scale
            if len(targets) >= count:
                break
        self.last_scale = chosen_scale
        return targets

    def collect_round(self, round_index: int, count: int | None = None, log=None) -> dict:
        if self.bridge.dry:
            raise RuntimeError("--dry-run 下不采集（臂不会动）；去掉 --dry-run 再试")
        count = int(count or 18)
        base = self._base_joints()
        log = log or self.log
        log(f"{ROUNDS[round_index]['tag']} 开始：{count} 个机位，扩散 ±{ROUNDS[round_index]['delta_deg']}°")
        moved = self.bridge.move_joints_deg(base.tolist(), timeout=25)
        if not moved["ok"]:
            raise RuntimeError("回观察位失败：" + moved["msg"])
        time.sleep(0.3)
        prior = self._prior()
        targets = self._targets(round_index, count, prior)
        if prior is not None:
            log(f"  按上一轮手眼解预筛：{len(targets)} 个机位的板预计还在画面里"
                + (f"（扩散收缩到 {self.last_scale:.2f} 倍）" if self.last_scale and self.last_scale < 0.999 else ""))
        self.samples = [s for s in self.samples if s.get("round") != round_index]
        self.solve_cache.pop(str(round_index), None)
        got, tried, log_lines = 0, 0, []
        for target in targets:
            if got >= count:
                break
            tried += 1
            target = np.round(target, 2)
            result = self.bridge.move_joints_deg(target.tolist(), timeout=25)
            if not result["ok"]:
                log(f"  候选 {tried}：{np.round(target, 0).tolist()} 没到位，跳过（{result['msg']}）")
                continue
            settled = self._settle()
            if not settled["ok"]:
                log(f"  候选 {tried}：{settled['why']}，跳过")
                continue
            color = settled["color"]
            info = self.detect_board(color)
            if not info or not info.get("ok"):
                log(f"  候选 {tried}：{info.get('why') if info else '没有画面'}，跳过")
                continue
            joints = self.bridge.joints_deg_now()
            flange = self.bridge.flange()
            if joints is None or flange is None:
                log(f"  候选 {tried}：读不到臂状态，跳过")
                continue
            sample = {
                "round": round_index, "index": got, "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                "joints_deg": [float(v) for v in joints],
                "T_flange_nominal": flange.tolist(),
                "corners": info["corners"], "ids": info["ids"],
                "n_corners": info["n_corners"], "rms_px": info["rms_px"], "view_deg": info["view_deg"],
                "T_cam_board": info["T_cam_board"],
                "settle": {"joint_ptp_deg": settled["joint_ptp_deg"],
                           "corner_rms_px": settled["corner_rms_px"],
                           "waited_s": settled["waited_s"]},
                "K_used": self.K.tolist(), "dist_used": self.dist.tolist(),
            }
            self.samples.append(sample)
            got += 1
            self._save_state()
            log(f"  候选 {tried}：{info['n_corners']} 角点 / {info['rms_px']:.2f}px / 斜视 "
                f"{info['view_deg']:.0f}° / 停稳 {settled['waited_s']:.1f}s → 收录第 {got}/{count} 条")
        self._save_state()
        log(f"{ROUNDS[round_index]['tag']} 结束：收录 {got} 条（试了 {tried} 个候选）")
        return {"round": round_index, "collected": got, "tried": tried}

    # ---- 解算 ----
    def flanges(self, samples: list[dict], correction=None) -> list[np.ndarray]:
        correction = self.correction if correction is None else correction
        if kinematics.is_identity(correction):
            return [as_T(s["T_flange_nominal"]) for s in samples]
        fk = self.model.forward(np.asarray(correction["offsets_m"], dtype=float))
        return [fk(kinematics.corrected_joints(np.asarray(s["joints_deg"], dtype=float), correction))
                for s in samples]

    def _accept(self, checks: dict, spread: dict, loo: dict, across: dict, n: int) -> tuple[bool, list[str]]:
        fails = []
        if n < GATES["min_poses"]:
            fails.append(f"位姿只有 {n} 个，少于 {GATES['min_poses']} 个")
        if not checks.get("rotation_ok"):
            fails.append("转轴散布不足（X 的旋转不唯一）")
        if not checks.get("translation_ok"):
            fails.append("平移不可解（(R_A-I) 秩不够）")
        if spread["position_rms_m"] * 1000 > GATES["accept_pos_rms_mm"]:
            fails.append(f"板位置 RMS {spread['position_rms_m'] * 1000:.2f}mm > {GATES['accept_pos_rms_mm']:.1f}mm")
        if spread["position_max_m"] * 1000 > GATES["accept_pos_max_mm"]:
            fails.append(f"板位置最大 {spread['position_max_m'] * 1000:.2f}mm > {GATES['accept_pos_max_mm']:.1f}mm")
        if spread["angle_rms_deg"] > GATES["accept_angle_rms_deg"]:
            fails.append(f"板姿态 RMS {spread['angle_rms_deg']:.2f}° > {GATES['accept_angle_rms_deg']:.1f}°")
        if loo.get("n") and spread["position_rms_m"] > 0:
            if loo["position_spread_m"] > GATES["accept_loo_ratio"] * spread["position_rms_m"] * 1.0000001:
                fails.append(f"去一留一变化 {loo['position_spread_m'] * 1000:.2f}mm > "
                             f"{GATES['accept_loo_ratio']:.0f}x RMS")
        if across.get("position_spread_m") is not None:
            if across["position_spread_m"] * 1000 > GATES["accept_method_mm"]:
                fails.append(f"解法互差 {across['position_spread_m'] * 1000:.2f}mm > {GATES['accept_method_mm']:.1f}mm")
            if across["angle_spread_deg"] > GATES["accept_method_deg"]:
                fails.append(f"解法角度互差 {across['angle_spread_deg']:.3f}° > {GATES['accept_method_deg']:.1f}°")
        return (len(fails) == 0), fails

    def solve_samples(self, samples: list[dict], correction=None) -> dict:
        if len(samples) < 3:
            raise RuntimeError(f"样本只有 {len(samples)} 条，少于 3 条解不出 X")
        flanges = self.flanges(samples, correction)
        boards = [as_T(s["T_cam_board"]) for s in samples]
        per_method = {}
        for name in handeye3d.METHODS:
            try:
                X = handeye3d.solve(flanges, boards, method=name)
                spread = handeye3d.consistency(X, flanges, boards)
                score = spread["position_rms_m"] * 1000 + 3.0 * spread["angle_rms_deg"]
                per_method[name] = {"X": X, "spread": spread, "score": score}
            except (ValueError, RuntimeError, cv2.error):
                continue
        if not per_method:
            raise RuntimeError("四种解法都失败，检查输入")
        best = min(per_method, key=lambda name: per_method[name]["score"])
        X = per_method[best]["X"]
        spread = per_method[best]["spread"]
        checks = handeye3d.observability(flanges)
        loo = handeye3d.leave_one_out(flanges, boards, method=best)
        across = handeye3d.cross_method_spread(flanges, boards)
        report = handeye3d.format_report(X, flanges, boards, method=best)
        ok, fails = self._accept(checks, spread, loo, across, len(samples))
        return {"X": X, "method": best, "report": report, "checks": checks, "spread": spread,
                "loo": loo, "across": across, "n": len(samples), "ok": ok, "fails": fails,
                "per_method": {name: {"score": item["score"],
                                      "rms_mm": item["spread"]["position_rms_m"] * 1000,
                                      "angle_deg": item["spread"]["angle_rms_deg"]} for name, item in per_method.items()}}

    def drop_round(self, round_index: int) -> int:
        """丢掉某一轮的样本（相机/板动过之后旧轮次就作废了）。"""
        before = len(self.samples)
        self.samples = [s for s in self.samples if s.get("round") != round_index]
        self.solve_cache.pop(str(round_index), None)
        self.reports.pop(f"round{round_index + 1}", None)
        self.results.pop(str(round_index), None)
        if self.last_solution and self.last_solution.get("round") == round_index:
            self.last_solution = None
        self._save_state()
        return before - len(self.samples)

    def round_samples(self, round_index: int) -> list[dict]:
        return [s for s in self.samples if int(s.get("round", -1)) == int(round_index)]

    def solve_round(self, round_index: int, correction=None) -> dict:
        samples = self.round_samples(round_index)
        result = self.solve_samples(samples, correction=correction)
        self.reports[f"round{round_index + 1}"] = result["report"]
        result["round"] = round_index
        self.results[str(round_index)] = {k: jsonable(v) for k, v in result.items() if k not in ("X",)}
        self.last_solution = result
        self.solve_cache[str(round_index)] = result
        self._save_state()
        return result

    def choose_round(self) -> int:
        avail = []
        for index in range(len(ROUNDS)):
            if len(self.round_samples(index)) >= 8:
                if str(index) not in self.solve_cache:
                    try:
                        self.solve_round(index)
                    except Exception as exc:  # noqa: BLE001
                        self.log(f"{ROUNDS[index]['tag']} 解算失败：{exc}")
                        continue
                avail.append(index)
        if not avail:
            raise RuntimeError("还没有任何一轮攒够 8 条样本")
        return min(avail, key=lambda i: self.solve_cache[str(i)]["spread"]["position_rms_m"] * 1000
                   + 3.0 * self.solve_cache[str(i)]["spread"]["angle_rms_deg"])

    # ---- 内参共标 ----
    def calibrate_intrinsics(self, round_index: int | None = None, log=None) -> dict:
        """用同一批手眼照片标彩色内参；采纳后回到角点重解板位姿（位姿与运行时用同一份 K）。"""
        log = log or self.log
        samples = self.round_samples(round_index) if round_index is not None else self.samples
        if len(samples) < 6:
            raise RuntimeError(f"只有 {len(samples)} 张照片，少于 6 张，内参标不稳")
        detections = [(np.asarray(s["corners"], dtype=np.float64).reshape(-1, 1, 2),
                       np.asarray(s["ids"], dtype=np.int32).reshape(-1, 1)) for s in samples]
        color = self.bridge.snap_frame()[0]
        width, height = (color.shape[1], color.shape[0]) if color is not None else (1280, 720)
        result = charuco.calibrate_intrinsics(self.board, detections, (width, height))
        matrix = np.asarray(result["camera_matrix"], dtype=float)
        focal_diff = abs(matrix[0, 0] - self.K[0, 0]) / max(self.K[0, 0], 1e-9) * 100.0
        compare_text = charuco.compare_intrinsics(self.K, matrix)
        accept = bool(result["used"] >= 8 and result["rms"] <= 1.5 and focal_diff <= 5.0)
        text = [f"内参共标（{result['used']} 张，{result['stage']}，RMS {result['rms']:.3f} px）",
                f"焦距相对当前清单变化 {focal_diff:+.2f}%（采纳线 5%）",
                compare_text, "",
                "结论：" + ("采纳，板位姿已用新内参重解" if accept else "不采纳，保留当前内参")]
        report = "\n".join(text)
        log(report.replace("\n", " / "))
        self.reports["intrinsics"] = report
        self.results["intrinsics"] = {"used": result["used"], "rms": float(result["rms"]),
                                      "stage": result["stage"], "focal_diff_pct": focal_diff,
                                      "accept": accept, "camera_matrix": matrix.tolist(),
                                      "dist_coeffs": np.asarray(result["dist_coeffs"], dtype=float).reshape(-1).tolist()}
        if accept:
            self.K = matrix
            self.dist = np.asarray(result["dist_coeffs"], dtype=float).reshape(-1)
            self.bridge.cfg["K"] = self.K
            self.bridge.cfg["dist"] = self.dist
            bad = 0
            for sample in self.samples:
                pose, rms, _note = charuco.estimate_pose(
                    self.board, np.asarray(sample["corners"], dtype=np.float64).reshape(-1, 1, 2),
                    np.asarray(sample["ids"], dtype=np.int32).reshape(-1, 1),
                    self.K, self.dist, min_corners=GATES["min_corners"])
                if pose is None:
                    sample["pnp_recal_ok"] = False
                    bad += 1
                    continue
                sample["pnp_recal_ok"] = True
                sample["T_cam_board"] = pose.tolist()
                sample["rms_px"] = float(rms)
                sample["view_deg"] = float(charuco.viewing_angle_deg(pose))
                sample["K_used"] = self.K.tolist()
                sample["dist_used"] = self.dist.tolist()
            self.solve_cache.clear()
            self.log(f"内参已采纳；重解板位姿 {len(self.samples) - bad}/{len(self.samples)} 张成功")
        self._save_state()
        return {"report": report, "result": self.results["intrinsics"]}

    # ---- 臂几何重拟合 ----
    def fit_geometry(self, round_index: int | None = None, log=None) -> dict:
        log = log or self.log
        if round_index is None:
            # 臂几何是跟轮次无关的，默认把所有轮次的样本合起来拟合
            samples = list(self.samples)
            label = "全部轮次"
            if len(samples) < arm_geometry.MIN_POSES:
                available = [i for i in range(len(ROUNDS))
                             if len(self.round_samples(i)) >= arm_geometry.MIN_POSES]
                if not available:
                    raise RuntimeError(f"总共只有 {len(samples)} 条，少于 {arm_geometry.MIN_POSES} 条，拟合不了几何")
                round_index = self.choose_round()
                samples = self.round_samples(round_index)
                label = ROUNDS[round_index]["tag"]
        else:
            samples = self.round_samples(round_index)
            label = ROUNDS[round_index]["tag"]
        if len(samples) < arm_geometry.MIN_POSES:
            raise RuntimeError(f"{label} 只有 {len(samples)} 条，少于 {arm_geometry.MIN_POSES} 条")
        joints = np.array([s["joints_deg"] for s in samples], dtype=float)
        boards = [as_T(s["T_cam_board"]) for s in samples]
        start = None if kinematics.is_identity(self.correction) else self.correction
        result = arm_geometry.fit(joints, boards, self.model, start=start)
        self.reports["geometry"] = result["report"]
        self.results["geometry"] = {key: jsonable(result[key]) for key in
                                    ("ok", "before", "after", "heldout_before", "heldout_after", "improvement")}
        log(result["report"].replace("\n", " / "))
        if result["ok"]:
            self.correction = result["correction"]
            self.solve_cache.clear()
            self.log("几何修正已采用；后续解 X 与验证都用修正后的 FK")
        self._save_state()
        return {"report": result["report"], "ok": bool(result["ok"]),
                "correction": jsonable(result["correction"]), "result": self.results["geometry"]}

    # ---- 定稿 ----
    def finalize(self, round_index: int | None = None, log=None) -> dict:
        log = log or self.log
        index = self.choose_round() if round_index is None else int(round_index)
        samples = self.round_samples(index)
        if len(samples) < GATES["min_poses"]:
            raise RuntimeError(f"{ROUNDS[index]['tag']} 只有 {len(samples)} 条，少于 {GATES['min_poses']} 条，不下结论")
        result = self.solve_round(index)
        X = result["X"]
        flanges = self.flanges(samples)
        boards = [as_T(s["T_cam_board"]) for s in samples]
        board_in_base = mean_transform(handeye3d.board_in_base(X, flanges, boards))
        self.final = {
            "round": index, "method": result["method"], "X": X.tolist(),
            "board_in_base": board_in_base.tolist(),
            "spread": jsonable(result["spread"]), "ok": bool(result["ok"]), "fails": result["fails"],
            "n": result["n"], "correction": jsonable(self.correction),
            "intrinsics_used": bool("intrinsics" in self.results and self.results["intrinsics"].get("accept")),
        }
        self.reports["final"] = result["report"]
        self.results["final"] = {k: jsonable(v) for k, v in self.final.items()}
        self._write_artifacts()
        log(f"定稿：{ROUNDS[index]['tag']} {result['n']} 个位姿，解法 {result['method']}，"
            f"板位置 RMS {result['spread']['position_rms_m'] * 1000:.2f} mm / "
            f"姿态 RMS {result['spread']['angle_rms_deg']:.3f}° → {'通过' if result['ok'] else '未通过'}"
            + ("" if result["ok"] else "：" + "；".join(result["fails"])))
        self._save_state()
        return {"report": result["report"], "final": self.results["final"]}

    def _write_artifacts(self):
        if not self.final:
            return
        X = as_T(self.final["X"])
        np.save(self.dir / "gripper_camera.npy", X)
        np.save(self.dir / "board_in_base.npy", as_T(self.final["board_in_base"]))
        (self.dir / "handeye.json").write_text(
            json.dumps(jsonable(self.final), ensure_ascii=False, indent=2), encoding="utf-8")
        kin_dir = self.dir / "kinematics"
        kin_dir.mkdir(exist_ok=True)
        (kin_dir / "correction.json").write_text(
            json.dumps(jsonable(self.correction), ensure_ascii=False, indent=2), encoding="utf-8")
        if "geometry" in self.reports:
            (kin_dir / "report.txt").write_text(self.reports["geometry"] + "\n", encoding="utf-8")
        intr_dir = self.dir / "intrinsics"
        intr_dir.mkdir(exist_ok=True)
        np.save(intr_dir / "camera_matrix.npy", self.K)
        np.save(intr_dir / "dist_coeffs.npy", self.dist)
        if "intrinsics" in self.reports:
            (intr_dir / "report.txt").write_text(self.reports["intrinsics"] + "\n", encoding="utf-8")
        (self.dir / "yaml_block.yaml").write_text(self.yaml_block(with_intrinsics=True) + "\n", encoding="utf-8")
        report = ["# 3D 手眼标定会话 " + self.stamp, "",
                  f"清单：{self.cfg['manifest']}", f"板：{self.board.describe()}", "",
                  "## 定稿解算", self.reports.get("final", ""), ""]
        for index in range(len(ROUNDS)):
            if f"round{index + 1}" in self.reports:
                report += [f"## {ROUNDS[index]['tag']} 单独解算（不混合轮次）", self.reports[f"round{index + 1}"], ""]
        if "intrinsics" in self.reports:
            report += ["## 内参共标", self.reports["intrinsics"], ""]
        if "geometry" in self.reports:
            report += ["## 臂几何重拟合", self.reports["geometry"], ""]
        if self.verify_samples:
            report += ["## 新位姿验证", self.verify_report(), ""]
        if "verify" in self.reports:
            report += ["## 交互验证", self.reports["verify"], ""]
        report += ["## 清单片段", self.yaml_block(with_intrinsics=True), ""]
        (self.dir / "report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")

    # ---- 验证（新位姿，不参与解算） ----
    verify_samples: list[dict] = []

    def verify_collect(self, count: int = 5, log=None) -> dict:
        if not self.final:
            raise RuntimeError("先定稿（解算）再验证")
        log = log or self.log
        if self.bridge.dry:
            raise RuntimeError("--dry-run 下不采集")
        X = as_T(self.final["X"])
        base = self._base_joints()
        patterns = [
            [8, -7, 6, 14, -9, 16], [-9, 8, -5, -12, 8, -18], [5, 11, -8, 9, -12, 11],
            [-6, -9, 9, -15, 5, -13], [11, 4, -9, 17, -6, 21],
        ]
        self.verify_samples = []
        for index in range(max(1, int(count))):
            delta = np.asarray(patterns[index % len(patterns)], dtype=float)
            target = np.clip(base + delta, [item[0] + 2 for item in self.limits],
                             [item[1] - 2 for item in self.limits])
            moved = self.bridge.move_joints_deg(target.tolist(), timeout=25)
            if not moved["ok"]:
                log(f"  验证位姿 {index + 1} 没到位，跳过")
                continue
            settled = self._settle()
            if not settled["ok"]:
                log(f"  验证位姿 {index + 1}：{settled['why']}，跳过")
                continue
            info = self.detect_board(settled["color"])
            if not info or not info.get("ok"):
                log(f"  验证位姿 {index + 1}：{info.get('why') if info else '没有画面'}，跳过")
                continue
            sample = {"joints_deg": [float(v) for v in self.bridge.joints_deg_now()],
                      "T_cam_board": info["T_cam_board"], "n_corners": info["n_corners"],
                      "rms_px": info["rms_px"], "view_deg": info["view_deg"]}
            self.verify_samples.append(sample)
            log(f"  验证位姿 {index + 1}：{info['n_corners']} 角点，{info['rms_px']:.2f}px，收录")
        if not self.verify_samples:
            raise RuntimeError("一个验证位姿都没采到")
        spread = handeye3d.consistency(
            X, self.flanges(self.verify_samples), [as_T(s["T_cam_board"]) for s in self.verify_samples])
        text = [f"新位姿验证（{len(self.verify_samples)} 个，未参与解算）",
                f"  板位置散布：RMS {spread['position_rms_m'] * 1000:.2f} mm / 最大 {spread['position_max_m'] * 1000:.2f} mm"
                f"（预算 {GATES['accept_pos_max_mm']:.0f} mm）",
                f"  板姿态散布：RMS {spread['angle_rms_deg']:.3f}° / 最大 {spread['angle_max_deg']:.3f}°",
                "  结论：" + ("通过" if spread["position_max_m"] * 1000 <= GATES["accept_pos_max_mm"] else "超预算")]
        self.verify_spread = spread
        self.reports["verify"] = "\n".join(text)
        log(text[0] + " / " + text[1] + " / " + text[3])
        self._save_state()
        return {"report": self.reports["verify"], "spread": jsonable(spread), "n": len(self.verify_samples)}

    def verify_report(self) -> str:
        return self.reports.get("verify", "")

    def verify_point(self, u: int, v: int, move: bool = False, log=None) -> dict:
        """点画面上的点：像素→相机→base，算悬停目标；可选真的移过去并报残差。"""
        if not self.final:
            raise RuntimeError("先定稿再验证")
        log = log or self.log
        self.bridge.cfg["K"] = self.K
        self.bridge.cfg["dist"] = self.dist
        p_cam, depth_m, depth_mm, age = self.bridge.pixel_to_cam(int(u), int(v))
        joints = self.bridge.joints_deg_now()
        if joints is None:
            raise RuntimeError("读不到关节角")
        if kinematics.is_identity(self.correction):
            T_flange = self.bridge.flange()
        else:
            T_flange = self.model.forward(np.asarray(self.correction["offsets_m"], dtype=float))(
                kinematics.corrected_joints(np.asarray(joints, dtype=float), self.correction))
        T_base_cam = T_flange @ as_T(self.final["X"])
        p_base = (T_base_cam @ np.r_[p_cam, 1.0])[:3]
        result = {"pixel": [int(u), int(v)], "depth_m": float(depth_m), "frame_age_s": round(float(age), 2),
                  "P_cam": p_cam.tolist(), "P_base": p_base.tolist()}
        if move:
            target = p_base + np.array([0.0, 0.0, 0.02])
            moved = self.bridge.move_tip(target)
            result["move"] = moved
            result["hover_target"] = target.tolist()
            log(f"验证点 ({u},{v})：深度 {depth_m * 1000:.0f}mm，base "
                f"{np.round(p_base, 3).tolist()}，悬停 20mm 上方 → {moved['msg']}")
        else:
            log(f"验证点 ({u},{v})：深度 {depth_m * 1000:.0f}mm，base {np.round(p_base, 3).tolist()}")
        self.reports["verify"] = (self.reports.get("verify", "") + "\n" +
                                  f"点像素 ({u},{v}) → base {np.round(p_base, 4).tolist()}"
                                  + (f"，悬停 {result['move']['msg']}" if move else "")).strip()
        self._save_state()
        return result

    # ---- 产物 ----
    def yaml_block(self, X=None, with_intrinsics: bool = False) -> str:
        X = as_T(X if X is not None else (self.final or {}).get("X") or np.eye(4))
        corr = self.correction
        lines = ["# skill.depth_grasp.config.calib", "      gripper_camera:"]
        for row in X:
            lines.append("        - [" + ", ".join(f"{float(v):.9f}" for v in row) + "]")
        lines += ["      correction:", "        zeros_deg: ["
                  + ", ".join(f"{float(v):.9f}" for v in corr["zeros_deg"]) + "]", "        offsets_m:"]
        for row in corr["offsets_m"]:
            lines.append("          - [" + ", ".join(f"{float(v):.9f}" for v in row) + "]")
        if with_intrinsics:
            lines += ["", "# service.grasp_geometry.config（内参共标后）", "      camera_matrix:"]
            for row in self.K:
                lines.append("        - [" + ", ".join(f"{float(v):.6f}" for v in row) + "]")
            lines.append("      dist_coeffs: [" + ", ".join(f"{float(v):.6f}" for v in self.dist) + "]")
        return "\n".join(lines)

    def apply_manifest(self) -> dict:
        if not self.apply_allowed:
            raise RuntimeError("启动时没给 --apply，不写清单（把报告里的片段手动粘过去）")
        if not self.final:
            raise RuntimeError("先定稿再写清单")
        path = Path(self.cfg["manifest"])
        original = path.read_text(encoding="utf-8")
        patched = patch_manifest(original, as_T(self.final["X"]), self.correction,
                                 self.K if self.final.get("intrinsics_used") else None,
                                 self.dist if self.final.get("intrinsics_used") else None)
        # 写回前重读一次，防止另一个会话在这期间改了清单
        if path.read_text(encoding="utf-8") != original:
            raise RuntimeError("清单在处理期间被别的会话改了，已放弃写入；重新解算后再试")
        backup = path.with_suffix(path.suffix + ".bak-" + time.strftime("%Y%m%d_%H%M%S"))
        backup.write_text(original, encoding="utf-8")
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(patched, encoding="utf-8")
        yaml.safe_load(patched)  # 语法自检，失败抛异常，不覆盖
        tmp.replace(path)
        self.reports["apply"] = f"已写入 {path}（备份 {backup.name}）"
        self.log(self.reports["apply"])
        self._save_state()
        return {"ok": True, "path": str(path), "backup": str(backup)}


# ------------------------------------------------------------ 清单打补丁 ----

def _fmt_row(row) -> str:
    return "[" + ", ".join(f"{float(v):.9f}" for v in row) + "]"


def _patch_matrix_block(lines: list[str], key: str, rows: list[list[float]]) -> list[str]:
    """把 key: 下面紧跟的 len(rows) 行 "- [...]" 换掉（保留原来的缩进与行尾注释）。"""
    pattern = re.compile(r"^(\s*)" + re.escape(key) + r":\s*(#.*)?$")
    for index, line in enumerate(lines):
        match = pattern.match(line)
        if not match:
            continue
        start = index + 1
        while start < len(lines) and not lines[start].strip():
            start += 1
        if start + len(rows) > len(lines):
            raise ValueError(f"清单里 {key}: 后面没有 {len(rows)} 行数据")
        for offset in range(len(rows)):
            if not lines[start + offset].lstrip().startswith("- ["):
                raise ValueError(f"清单里 {key}: 后面不是 {len(rows)} 行 - [...]，不敢改")
        indent = lines[start][:len(lines[start]) - len(lines[start].lstrip())]
        replacement = [indent + "- " + _fmt_row(row) for row in rows]
        return lines[:start] + replacement + lines[start + len(rows):]
    raise ValueError(f"清单里找不到 {key}: 字段")


def _patch_list_line(lines: list[str], key: str, values) -> list[str]:
    pattern = re.compile(r"^(\s*)" + re.escape(key) + r":\s*\[[^\]]*\](\s*#.*)?$")
    hits = [index for index, line in enumerate(lines) if pattern.match(line)]
    if len(hits) != 1:
        raise ValueError(f"清单里 {key}: [...] 行找到 {len(hits)} 处，期望 1 处，不敢改")
    index = hits[0]
    match = pattern.match(lines[index])
    indent, comment = match.group(1), match.group(2) or ""
    lines[index] = indent + key + ": [" + ", ".join(f"{float(v):.9f}" for v in values) + "]" + comment
    return lines


def _patch_list_line(lines: list[str], key: str, values) -> list[str]:
    pattern = re.compile(r"^(\s*)" + re.escape(key) + r":\s*\[[^\]]*\]\s*$")
    hits = [index for index, line in enumerate(lines) if pattern.match(line)]
    if len(hits) != 1:
        raise ValueError(f"清单里 {key}: [...] 行找到 {len(hits)} 处，期望 1 处，不敢改")
    index = hits[0]
    indent = pattern.match(lines[index]).group(1)
    lines[index] = indent + key + ": [" + ", ".join(f"{float(v):.9f}" for v in values) + "]"
    return lines


def patch_manifest(text: str, X, correction: dict, K=None, dist=None) -> str:
    """把解算结果写回清单文本：gripper_camera 4x4 → correction → （可选）内参。"""
    lines = text.splitlines()
    lines = _patch_matrix_block(lines, "gripper_camera", as_T(X).tolist())
    lines = _patch_list_line(lines, "zeros_deg", [float(v) for v in correction["zeros_deg"]])
    lines = _patch_matrix_block(lines, "offsets_m", [[float(v) for v in row] for row in correction["offsets_m"]])
    if K is not None:
        lines = _patch_matrix_block(lines, "camera_matrix", np.asarray(K, dtype=float).reshape(3, 3).tolist())
        lines = _patch_list_line(lines, "dist_coeffs", np.asarray(dist, dtype=float).reshape(-1).tolist())
    patched = "\n".join(lines) + "\n"
    # 自检：解析回来，确认真要写的东西都真的写进去了
    doc = yaml.safe_load(patched)
    skills = [it for it in (doc.get("skill") or []) if it.get("name") == "depth_grasp"]
    got = skills[0]["config"]["calib"]["gripper_camera"]
    if not np.allclose(np.asarray(got, dtype=float), as_T(X), atol=1e-7):
        raise ValueError("打补丁后 gripper_camera 对不上")
    got_corr = skills[0]["config"]["calib"]["correction"]
    if not np.allclose(np.asarray(got_corr["zeros_deg"], dtype=float), correction["zeros_deg"], atol=1e-6):
        raise ValueError("打补丁后 zeros_deg 对不上")
    if not np.allclose(np.asarray(got_corr["offsets_m"], dtype=float), correction["offsets_m"], atol=1e-8):
        raise ValueError("打补丁后 offsets_m 对不上")
    if K is not None:
        svcs = [it for it in (doc.get("service") or []) if it.get("name") == "grasp_geometry"]
        if not np.allclose(np.asarray(svcs[0]["config"]["camera_matrix"], dtype=float), np.asarray(K).reshape(3, 3),
                           atol=1e-5):
            raise ValueError("打补丁后 camera_matrix 对不上")
    return patched


# ---------------------------------------------------------------- 网页 ----

def state_of(calib: Calib) -> dict:
    def brief(sample):
        return {"round": int(sample.get("round", -1)), "index": int(sample.get("index", -1)),
                "n_corners": int(sample.get("n_corners", 0)),
                "rms_px": round(float(sample.get("rms_px", 0.0)), 2),
                "view_deg": round(float(sample.get("view_deg", 0.0)), 1),
                "joints_deg": [round(float(v), 1) for v in sample.get("joints_deg", [])]}
    rounds = []
    for index, spec in enumerate(ROUNDS):
        solved = calib.results.get(str(index)) or {}
        rounds.append({"tag": spec["tag"], "delta_deg": spec["delta_deg"],
                       "count": len(calib.round_samples(index)),
                       "solved": bool(solved),
                       "rms_mm": round(float(solved.get("spread", {}).get("position_rms_m", 0.0)) * 1000, 2) if solved else None,
                       "angle_deg": round(float(solved.get("spread", {}).get("angle_rms_deg", 0.0)), 3) if solved else None,
                       "ok": solved.get("ok") if solved else None,
                       "fails": solved.get("fails") or []})
    return {"session": calib.stamp, "dir": str(calib.dir), "dry": bool(calib.bridge.dry),
            "ready": calib.bridge.ready(),
            "job": calib.job.snapshot() if calib.job is not None else None,
            "rounds": rounds,
            "samples": [brief(s) for s in calib.samples[-40:]],
            "reports": calib.reports,
            "final": calib.results.get("final"),
            "correction": calib.correction,
            "k": {"fx": round(float(calib.K[0, 0]), 3), "fy": round(float(calib.K[1, 1]), 3),
                  "cx": round(float(calib.K[0, 2]), 3), "cy": round(float(calib.K[1, 2]), 3)},
            "dist": [round(float(v), 4) for v in calib.dist],
            "yaml": calib.yaml_block(with_intrinsics=True),
            "verify": {"n": len(calib.verify_samples), "report": calib.reports.get("verify", ""),
                       "spread": jsonable(getattr(calib, "verify_spread", None))},
            "observe": [round(float(v), 1) for v in calib.cfg["observe_joints_deg"]],
            "limits": [[round(float(lo), 1), round(float(hi), 1)] for lo, hi in calib.limits],
            "manifest": calib.cfg["manifest"]}


PAGE = """<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<title>3D 手眼标定</title>
<style>
body{font-family:ui-sans-serif,system-ui,"Noto Sans CJK SC",sans-serif;margin:16px;background:#111;color:#ddd}
h1{font-size:18px;margin:0 0 8px}
.bar{margin:8px 0}
button{background:#2b6;border:0;color:#011;padding:6px 10px;margin:2px;border-radius:4px;cursor:pointer;font-size:13px}
button.alt{background:#468}.bar button:disabled{opacity:.4;cursor:default}
.grid{display:flex;gap:16px;flex-wrap:wrap}
img{background:#000;border:1px solid #333}
pre{background:#1a1a1a;border:1px solid #333;padding:8px;overflow:auto;font-size:12px;line-height:1.35}
#log{height:220px}
.card{border:1px solid #333;padding:8px;border-radius:4px;min-width:260px}
table{border-collapse:collapse;font-size:12px}
td,th{border-bottom:1px solid #333;padding:3px 8px;text-align:left}
.ok{color:#4d4}.bad{color:#f66}.dim{color:#888}
</style></head><body>
<h1>3D 手眼标定 · 固定 ChArUco + AX=XB（渐进三轮）</h1>
<div id="top" class="dim">连接中…</div>
<div class="bar">
  <button onclick="act('collect',{round:0})">采集 R1</button>
  <button onclick="act('collect',{round:1})">采集 R2</button>
  <button onclick="act('collect',{round:2})">采集 R3</button>
  <button class="alt" onclick="act('solve',{})">解算各轮</button>
  <button class="alt" onclick="act('intrinsics',{})">内参共标</button>
  <button class="alt" onclick="act('geometry',{})">几何重拟合</button>
  <button onclick="act('finalize',{})">定稿</button>
  <button onclick="act('verify',{count:5})">新位姿验证</button>
  <button onclick="act('apply',{})">写清单</button>
  <button class="alt" onclick="act('hold',{})">急停</button>
</div>
<div class="grid">
  <div><img id="view" width="640" height="480"></div>
  <div class="card"><div id="rounds"></div></div>
</div>
<h3>清单片段</h3><pre id="yaml"></pre>
<h3>报告</h3><pre id="report" style="max-height:260px"></pre>
<h3>日志</h3><pre id="log"></pre>
<script>
let lastLines=0;
async function act(action, extra){
  const body = Object.assign({action:action}, extra||{});
  const r = await fetch('/api/act',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const j = await r.json();
  if(!j.ok) alert(j.error||'失败');
}
function esc(s){return String(s==null?'':s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function render(s){
  const ready=s.ready||{};
  document.getElementById('top').innerHTML =
    '会话 '+s.session+' · '+(s.dry?'<b class="bad">DRY-RUN</b>':'真实动作')+
    ' · 相机 '+(ready.color?'<span class="ok">OK</span>':'<span class="bad">无</span>')+
    ' · 深度 '+(ready.depth?'<span class="ok">OK</span>':'<span class="bad">无</span>')+
    ' · 臂 '+(ready.arm?'<span class="ok">OK</span>':'<span class="bad">无</span>')+
    ' · fx '+s.k.fx+' · 观察位 '+s.observe.join(',');
  let h='<table><tr><th>轮</th><th>扩散°</th><th>样本</th><th>板 RMS mm</th><th>姿态°</th><th>状态</th></tr>';
  for(const r of s.rounds){
    h+='<tr><td>'+r.tag+'</td><td>'+r.delta_deg.join(',')+'</td><td>'+r.count+'</td>'+
       '<td>'+(r.rms_mm==null?'-':r.rms_mm)+'</td><td>'+(r.angle_deg==null?'-':r.angle_deg)+'</td>'+
       '<td>'+(r.solved?(r.ok?'<span class="ok">通过</span>':'<span class="bad">'+esc((r.fails||[]).join('；'))+'</span>'):'-')+'</td></tr>';
  }
  h+='</table>';
  if(s.final){h+='<div style="margin-top:6px">定稿：'+esc(JSON.stringify(s.final.spread))+'</div>'}
  document.getElementById('rounds').innerHTML=h;
  document.getElementById('yaml').textContent=s.yaml||'';
  const fin=s.reports&&s.reports.final?s.reports.final:'';
  const ver=s.verify&&s.verify.report?s.verify.report:'';
  document.getElementById('report').textContent=(fin?fin+'\\n\\n':'')+ver;
  const job=s.job;
  const log=document.getElementById('log');
  if(job){log.textContent=job.lines.join('\\n');log.scrollTop=log.scrollHeight;}
  if(job&&!job.running&&job.error){document.getElementById('top').innerHTML+=' · <b class="bad">任务失败：'+esc(job.error)+'</b>'}
}
async function tick(){try{const r=await fetch('/api/state');render(await r.json())}catch(e){}}
setInterval(tick,1500);tick();
setInterval(()=>{const i=document.getElementById('view');i.src='/api/frame.jpg?t='+Date.now()},2000);
</script></body></html>"""


def make_app(calib: Calib):
    from flask import Flask, Response, jsonify, request

    app = Flask("handeye3d_web")
    app.config["JSON_AS_ASCII"] = False

    def start_job(name: str, fn):
        if calib.job is not None and calib.job.running:
            return jsonify({"ok": False, "error": f"还有任务在跑：{calib.job.name}"})
        calib.job = Job(name, fn)
        return jsonify({"ok": True, "job": name})

    @app.get("/")
    def index():
        return Response(PAGE, mimetype="text/html")

    @app.get("/api/state")
    def api_state():
        return jsonify(state_of(calib))

    @app.get("/api/frame.jpg")
    def api_frame():
        color = calib.bridge.snap_frame()[0]
        if color is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "no camera frame", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 200), 2)
            ok, buf = cv2.imencode(".jpg", blank)
            return Response(buf.tobytes(), mimetype="image/jpeg")
        info = calib.detect_board(color)
        canvas = color.copy()
        if info and info.get("n_corners"):
            pts = np.asarray(info["corners"], dtype=float).reshape(-1, 2).astype(int)
            for u, v in pts:
                cv2.circle(canvas, (int(u), int(v)), 3, (0, 255, 0), -1)
        text = "no ChArUco"
        if info:
            text = (f"{info.get('n_corners', 0)} pts  rms {info.get('rms_px', 0):.2f}px  "
                    f"view {info.get('view_deg', 0):.0f}deg  {'OK' if info.get('ok') else info.get('why', '')}")
        cv2.putText(canvas, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if info and info.get("ok") else (0, 0, 255), 2)
        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return Response(buf.tobytes(), mimetype="image/jpeg")

    @app.post("/api/act")
    def api_act():
        payload = request.get_json(silent=True) or {}
        action = payload.get("action") or request.args.get("action")
        raw_round = payload.get("round", request.args.get("round"))
        round_index = None if raw_round in (None, "", "null") else int(raw_round)
        count = int(payload.get("count") or request.args.get("count") or 18)
        try:
            if action == "collect":
                if round_index is None:
                    raise RuntimeError("collect 需要 round")
                return start_job(f"采集 {ROUNDS[round_index]['tag']}",
                                 lambda log: calib.collect_round(round_index, count, log))
            if action == "solve":
                def run(log):
                    solved = []
                    indexes = [round_index] if round_index is not None else list(range(len(ROUNDS)))
                    for index in indexes:
                        if len(calib.round_samples(index)) >= 3:
                            calib.solve_round(index)
                            solved.append(index)
                    if not solved:
                        raise RuntimeError("没有任何一轮有 >=3 条样本")
                    return {"solved": solved, "chosen": calib.choose_round()}
                return start_job("解算各轮", run)
            if action == "intrinsics":
                return start_job("内参共标", lambda log: calib.calibrate_intrinsics(round_index, log))
            if action == "drop_round":
                if round_index is None:
                    raise RuntimeError("drop_round 需要 round")
                dropped = calib.drop_round(round_index)
                return jsonify({"ok": True, "dropped": dropped})
            if action == "geometry":
                return start_job("几何重拟合", lambda log: calib.fit_geometry(round_index, log))
            if action == "finalize":
                return start_job("定稿", lambda log: calib.finalize(round_index, log))
            if action == "verify":
                return start_job("新位姿验证", lambda log: calib.verify_collect(count, log))
            if action == "verify_point":
                u = int(payload.get("u", request.args.get("u", 0)))
                v = int(payload.get("v", request.args.get("v", 0)))
                move = bool(payload.get("move", request.args.get("move") in ("1", "true", "yes")))
                return jsonify({"ok": True, "result": jsonable(calib.verify_point(u, v, move))})
            if action == "apply":
                return jsonify({"ok": True, "result": calib.apply_manifest()})
            if action == "hold":
                calib.bridge.hold()
                return jsonify({"ok": True, "result": "已发急停（保持当前位姿）"})
            if action == "detect":
                info = calib.detect_board(calib.bridge.snap_frame()[0])
                info = dict(info or {})
                info.pop("corners", None)
                info.pop("ids", None)
                return jsonify({"ok": True, "result": info})
            return jsonify({"ok": False, "error": f"未知动作 {action}"})
        except Exception as exc:  # noqa: BLE001  网页要看到错误，不能 500 黑洞
            import traceback
            traceback.print_exc()
            return jsonify({"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    return app


# ---------------------------------------------------------------- 自检 ----

SELFTEST_MANIFEST = """
skill:
  - name: depth_grasp
    config:
      calib:
        gripper_camera:
          - [1.0, 0.0, 0.0, 0.0]
          - [0.0, 1.0, 0.0, 0.0]
          - [0.0, 0.0, 1.0, 0.0]
          - [0.0, 0.0, 0.0, 1.0]
        correction:
          zeros_deg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          offsets_m:
            - [0.0, 0.0, 0.0]
            - [0.0, 0.0, 0.0]
            - [0.0, 0.0, 0.0]
            - [0.0, 0.0, 0.0]
            - [0.0, 0.0, 0.0]
            - [0.0, 0.0, 0.0]
service:
  - name: grasp_geometry
    config:
      camera_matrix:
        - [900.0, 0.0, 640.0]
        - [0.0, 900.0, 360.0]
        - [0.0, 0.0, 1.0]
      dist_coeffs: [0.1, -0.2, 0.0, 0.0, 0.1]
"""


def _random_rotation(rng, max_deg: float) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = math.radians(rng.uniform(-max_deg, max_deg))
    return cv2.Rodrigues(axis * angle)[0]


def selftest() -> int:
    print("== 1. handeye3d 合成数据反解 ==")
    rng = np.random.default_rng(20261112)
    T_board = handeye3d.make_transform(cv2.Rodrigues(np.array([0.3, -1.2, 0.8]))[0],
                                       np.array([0.45, -0.02, -0.065]))
    X_true = handeye3d.make_transform(cv2.Rodrigues(np.array([1.35, 0.25, -0.4]))[0],
                                      np.array([-0.0024, 0.0021, 0.0421]))
    g2b, t2c = [], []
    for _ in range(16):
        T = handeye3d.make_transform(_random_rotation(rng, 25.0),
                                     np.array([0.30, 0.0, 0.25]) + rng.normal(scale=0.04, size=3))
        g2b.append(T)
        t2c.append(np.linalg.inv(X_true) @ np.linalg.inv(T) @ T_board)
    X_hat = handeye3d.solve(g2b, t2c, method="PARK")
    spread = handeye3d.consistency(X_hat, g2b, t2c)
    err_mm = float(np.linalg.norm(X_hat[:3, 3] - X_true[:3, 3]) * 1000)
    angle = handeye3d.rotation_angle_deg(X_hat[:3, :3] @ X_true[:3, :3].T)
    obs = handeye3d.observability(g2b)
    print(f"   X 平移误差 {err_mm:.4f} mm / 旋转误差 {angle:.5f}°；板位置 RMS {spread['position_rms_m']*1000:.4f} mm")
    print(f"   可观测性：转轴散布 {obs['axis_spread']:.3f}，平移 σmin {obs['translation_sigma_min']:.2f}，"
          f"rotation_ok={obs['rotation_ok']} translation_ok={obs['translation_ok']}")
    bad = []
    if err_mm > 0.05:
        bad.append("合成数据 X 反解误差 > 0.05 mm")
    if spread["position_rms_m"] * 1000 > 0.01:
        bad.append("合成数据板散布 > 0.01 mm")
    if not (obs["rotation_ok"] and obs["translation_ok"]):
        bad.append("合成数据可观测性没通过")

    print("== 2. 清单打补丁 ==")
    X_patch = handeye3d.make_transform(np.eye(3), np.array([0.01, -0.02, 0.03]))
    corr = {"zeros_deg": [1, 2, 3, 4, 5, 6],
            "offsets_m": [[0.001 * i, 0.002 * i, 0.003 * i] for i in range(6)]}
    K = np.array([[910.0, 0, 640.0], [0, 911.0, 360.0], [0, 0, 1.0]])
    dist = np.array([0.11, -0.22, 0.001, 0.002, 0.05])
    patched = patch_manifest(SELFTEST_MANIFEST, X_patch, corr, K, dist)
    doc = yaml.safe_load(patched)
    got = doc["skill"][0]["config"]["calib"]["gripper_camera"]
    print(f"   gripper_camera 回读一致：{np.allclose(np.asarray(got), X_patch)}")
    if not np.allclose(np.asarray(got), X_patch):
        bad.append("清单补丁回读不一致")

    print("== 3. 关节零点往返 ==")
    joints = np.array([3.0, 40.0, -60.0, 8.0, 60.0, 10.0])
    corrected = kinematics.corrected_joints(joints, {"zeros_deg": [0.5, -0.2, 0.3, 4.0, 1.5, 0.0]})
    back = kinematics.command_joints(corrected, {"zeros_deg": [0.5, -0.2, 0.3, 4.0, 1.5, 0.0]})
    print(f"   corrected={np.round(corrected, 3).tolist()} 回扣一致：{np.allclose(back, joints)}")
    if not np.allclose(back, joints):
        bad.append("零点往返不一致")

    print("== 4. 记录文件读写 ==")
    sample = {"a": np.arange(3), "b": Path("/tmp/x"), "c": {"d": np.float64(1.5)}}
    text = json.dumps(jsonable(sample))
    print(f"   jsonable → {text}")
    if "0" not in text:
        bad.append("jsonable 坏了")

    if bad:
        print("自检失败：\n  - " + "\n  - ".join(bad))
        return 1
    print("自检通过。")
    return 0


# ---------------------------------------------------------------- 只读探测 ----

def probe(cfg: dict, board: charuco.Board) -> int:
    import rclpy
    rclpy.init(args=None)
    bridge = hw.Bridge(cfg, dry_run=True, node_name="handeye3d_probe")
    try:
        ready = {}
        for _ in range(100):
            ready = bridge.ready()
            if all(ready.values()):
                break
            time.sleep(0.1)
        print(f"ready: {ready}")
        print(f"清单：{cfg['manifest']}")
        print(f"相机：{cfg['rgb_topic']} / {cfg['depth_topic']}")
        print(f"内参：fx={cfg['K'][0,0]:.3f} fy={cfg['K'][1,1]:.3f} cx={cfg['K'][0,2]:.3f} cy={cfg['K'][1,2]:.3f}"
              f"  dist={np.round(np.asarray(cfg['dist']), 4).tolist()}")
        print(f"标定板：{board.describe()}")
        print(f"观察位：{np.round(cfg['observe_joints_deg'], 2).tolist()}")
        limits = ik_mod.joint_limits_deg()
        print(f"URDF 限位：{[[round(lo, 1), round(hi, 1)] for lo, hi in limits]}")
        joints = bridge.joints_deg_now()
        print(f"当前关节：{None if joints is None else np.round(joints, 2).tolist()}")
        color = bridge.snap_frame()[0]
        if color is None:
            print("没有画面")
            return 1
        corners, ids = board.detect(color)
        if corners is None:
            print("当前画面没有 ChArUco 角点")
            return 1
        pose, rms, note = charuco.estimate_pose(board, corners, ids, cfg["K"], cfg["dist"],
                                                min_corners=GATES["min_corners"])
        if pose is None:
            print(f"板位姿解不出来：{note}")
            return 1
        print(f"板：{len(ids)} 角点，RMS {rms:.3f}px，斜视 {charuco.viewing_angle_deg(pose):.1f}°，"
              f"t_cam={(np.round(pose[:3,3], 4)*1000).tolist()} mm")
        print("探测通过（没有动臂）。")
        return 0
    finally:
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------- main ----

def main() -> int:
    ap = argparse.ArgumentParser(description="3D 手眼标定网页（固定 ChArUco + AX=XB + 渐进三轮）")
    ap.add_argument("--port", type=int, default=8771)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--manifest", default=str(ROOT / "robonix_manifest.nx.yaml"))
    ap.add_argument("--squares", default="12x9", help="ChArUco 格子数（横x竖，数的是格子不是角点）")
    ap.add_argument("--square-mm", type=float, default=15.0)
    ap.add_argument("--marker-mm", type=float, default=None, help="码边长（默认 0.75 x 格边长）")
    ap.add_argument("--dict", default="DICT_5X5_100")
    ap.add_argument("--per-round", type=int, default=18)
    ap.add_argument("--out", default=str(Path.home() / ".robonix" / "calib3d"))
    ap.add_argument("--dry-run", action="store_true", help="只算不动作；采集/验证会拒绝")
    ap.add_argument("--apply", action="store_true", help="允许把结果写回清单（默认只出片段）")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--probe", action="store_true", help="只读探测：相机/臂/板，不动臂")
    ap.add_argument("--session", default="", help="续跑指定会话目录（已有 session.json）")
    ap.add_argument("--resume", action="store_true", help="续跑 --out 下最新的会话")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    cfg = load_cfg(Path(args.manifest))
    cfg["urdf"] = str(hw.find_urdf(cfg))
    for key, value in (cfg.get("env") or {}).items():
        if isinstance(value, str) and key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "LYNX_ROBOT_DOMAIN_ID"):
            os.environ.setdefault(key, value)
    squares_x, squares_y = (int(v) for v in args.squares.lower().split("x"))
    square_m = args.square_mm / 1000.0
    board = charuco.Board(squares_x, squares_y, square_m,
                          (args.marker_mm / 1000.0) if args.marker_mm else square_m * 0.75, args.dict)

    if args.probe:
        return probe(cfg, board)

    import rclpy
    rclpy.init(args=None)
    bridge = hw.Bridge(cfg, dry_run=args.dry_run, node_name="handeye3d_web")
    out_root = Path(args.out).expanduser()
    session_dir = None
    if args.session:
        session_dir = Path(args.session).expanduser()
    elif args.resume:
        existing = sorted([p for p in out_root.glob("2026*") if (p / "session.json").exists()])
        if not existing:
            raise SystemExit(f"{out_root} 下没有可续跑的会话")
        session_dir = existing[-1]
    calib = Calib(bridge, cfg, board, out_root, apply_allowed=args.apply, session_dir=session_dir)
    app = make_app(calib)

    print("=" * 70, flush=True)
    print(f"3D 手眼标定网页  http://<NX-IP>:{args.port}/   （或 ssh -L {args.port}:127.0.0.1:{args.port}）")
    print(f"清单：{args.manifest}")
    print(f"板：{board.describe()}")
    print(f"内参：fx={cfg['K'][0,0]:.3f} fy={cfg['K'][1,1]:.3f} cx={cfg['K'][0,2]:.3f} cy={cfg['K'][1,2]:.3f}"
          f"（来自 {'清单' if cfg.get('k_declared') else '兜底值'}）")
    print(f"会话目录：{calib.dir}")
    print(f"模式：{'DRY-RUN（不动臂）' if args.dry_run else '真实动作'}"
          f"   写清单：{'允许（--apply）' if args.apply else '只出片段'}")
    print("=" * 70, flush=True)
    try:
        app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)
    finally:
        # 退出时先尽量把臂停在原地，然后硬退出：rclpy 的 spin 线程与解释器析构一起走会 abort
        try:
            bridge.hold()
        except Exception:  # noqa: BLE001
            pass
        os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
