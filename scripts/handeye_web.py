#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手眼标定网页 —— 点哪够哪 + 多视角手眼解算

两种模式（页面左上角切换）：

  A 点哪够哪：在实时画面上点一个点，爪尖移动到该 3D 点（先到上方悬停，按 d 下探）。
             空格记录一条样本（点击像素 / 深度 / 相机系 3D / 当前 flange 位姿 / 爪尖实际位置 / 误差）。
             这种样本只能量固定偏差，解不出新的手眼矩阵。

  B 多视角手眼：点桌面上同一个固定特征（胶带十字、盒子角、ArUco 角点都行）→
             按「换观察位」让臂换一个视角 → 在新视角下再点同一特征 → 空格记录。
             攒够 3 组以上（位姿差异要够大）后按 ESC，用「未知世界点 + 交替 Kabsch」
             解出 T_flange_camera，输出残差与可直接粘回清单的 4x4。

按键：空格=记录   ESC=保存并退出   d=下探到点击点   h=回悬停   方向键=XY 微调 2mm
      n=换观察位（模式 B）   0=回 home（全零关节）

安全：工作空间软限位 x[0.10,0.58] y[-0.38,0.38]；z 下限取清单里的 table_height_m-5mm，
      点到更低的表面（地面/桌面）时会自动把下限放宽到该表面以下 5mm；单次位移 >0.6m 拒绝。

已在本机验证（2026-11-11）：A 模式 点击→悬停(残差 2.0mm)→下探(2.3mm)→记录(爪尖离点击点 4.6mm)；
B 模式 5 个视角（j1±14°、j6±25°、j4+12°，位姿差异 50°）解出 T_flange_camera 残差 5.4mm，
      与清单里的台架标定值差 78.8mm / 7.2°（清单隐含的 5 点世界点散布 13.6mm，比解算残差大一个量级
      → 说明相机现在的安装和台架标定确实不一样）。

启动（NX 上，栈要先起来）：
    cd ~/robot-deep_robotics-lynx_s10
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=43            # 或者不设置，脚本会从清单 env 里取
    python3 scripts/handeye_web.py --port 8770
浏览器：
    http://<NX-IP>:8770/                （同一局域网）
    ssh -L 8770:127.0.0.1:8770 nvidia@<NX-IP> 然后开 http://127.0.0.1:8770/

只算不动（验证页面/坐标，不动机械臂）：加 --dry-run
"""

from __future__ import annotations

import argparse
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
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[1]

# 工作空间软限位（base_link 系，米）。表在 base 下方约 6.5cm，z 下限由 table_height_m 决定。
BOUNDS = {"x": (0.10, 0.58), "y": (-0.38, 0.38), "z": (-0.20, 0.50)}


# ---------------------------------------------------------------- 配置 ----

def load_manifest_cfg(path: Path) -> dict:
    """从部署清单里取相机话题/内参、手眼初值、TCP、观察位。"""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}

    def _by_name(items, name):
        for it in items or []:
            if isinstance(it, dict) and it.get("name") == name:
                return it
        return {}

    prims = doc.get("primitive") or []
    svcs = doc.get("service") or []
    skills = doc.get("skill") or []
    cam_cfg = _by_name(prims, "realsense_camera").get("config") or {}
    svc_cfg = _by_name(svcs, "grasp_geometry").get("config") or {}
    sk_cfg = _by_name(skills, "depth_grasp").get("config") or {}
    calib = sk_cfg.get("calib") or {}

    cam_name = cam_cfg.get("camera_name", "camera_435i")
    k = svc_cfg.get("camera_matrix")
    Tfc = calib.get("gripper_camera")
    tip = calib.get("tip_in_flange")

    cfg = {
        "rgb_topic": svc_cfg.get("rgb_topic") or f"/{cam_name}/color/image_raw",
        "depth_topic": svc_cfg.get("depth_topic") or f"/{cam_name}/aligned_depth_to_color/image_raw",
        "K": np.asarray(k, dtype=float).reshape(3, 3) if k else np.array(
            [[902.218237, 0, 657.76298], [0, 901.975088, 365.019295], [0, 0, 1]]),
        "dist": np.asarray(svc_cfg.get("dist_coeffs") or [], dtype=float),
        "depth_units_mm": float(svc_cfg.get("depth_units_mm", 1.0)),
        "T_flange_camera": np.asarray(Tfc, dtype=float).reshape(4, 4) if Tfc else np.eye(4),
        "tip_in_flange": np.asarray(tip if tip else [0.0, 0.0, 0.14287], dtype=float),
        "observe_joints_deg": list(calib.get("observe_joints_deg") or [0.0] * 6),
        "table_height_m": sk_cfg.get("table_height_m"),
        "env": doc.get("env") or {},
        "manifest": str(path),
    }
    return cfg


def find_urdf(cfg: dict) -> Path:
    """找抓取技能包里自带的 piper URDF（FK 与技能同一套模型）。"""
    rel = "grasp_skill/calib/common/robot/urdf/piper/piper_description.urdf"
    for cand in (ROOT / "rbnx-boot" / "cache" / "skill-pick-depth-grasp-rbnx" / rel,
                 Path("/tmp/dg") / rel):
        if cand.exists():
            return cand
    cache = ROOT / "rbnx-boot" / "cache"
    if cache.is_dir():
        for cand in cache.glob("*/" + rel):
            return cand
    raise SystemExit("找不到 piper_description.urdf（先用 ./start.sh 或 rbnx build 把技能包拉下来）")


def build_chain(urdf_path: Path):
    import kinpy
    content = re.sub(r"<\?xml[^?]*\?>", "", Path(urdf_path).read_text(encoding="utf-8"), count=1)
    return kinpy.build_serial_chain_from_urdf(content, "link6")


# ------------------------------------------------------------- 数学工具 ----

def pose_to_T(p) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    if not any(q):
        return T
    T[:3, :3] = R.from_quat(q).as_matrix()
    return T


def pose_msg(xyz, quat):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in quat)
    return p


def T_to_pose_fields(T):
    return ([float(v) for v in T[:3, 3]],
            [float(v) for v in R.from_matrix(T[:3, :3]).as_quat()])


def kabsch(P: np.ndarray, Q: np.ndarray):
    """求 R,t 使 ||R P_i + t - Q_i|| 最小。"""
    Pc, Qc = P.mean(0), Q.mean(0)
    H = (P - Pc).T @ (Q - Qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return Rm, Qc - Rm @ Pc


def solve_handeye(P_list, T_list, init_T: np.ndarray):
    """多视角点同一固定世界点 → 解 T_flange_camera。

    约束：T_i^{-1} X = R P_i + t（同一个世界点 X 在 flange 系的坐标）
    交替迭代：固定 (R,t) 求 X；固定 X 用 Kabsch 求 (R,t)。
    """
    Qs = [np.linalg.inv(T) for T in T_list]
    Rm = init_T[:3, :3].copy()
    t = init_T[:3, 3].copy()

    def to_base(P, T, Rm, t):
        p_f = Rm @ P + t
        return (T @ np.array([p_f[0], p_f[1], p_f[2], 1.0]))[:3]

    X = np.mean([to_base(P, T, Rm, t) for P, T in zip(P_list, T_list)], axis=0)
    for _ in range(200):
        Q = np.array([(Qi @ np.array([X[0], X[1], X[2], 1.0]))[:3] for Qi in Qs])
        Rm, t = kabsch(np.array(P_list), Q)
        Xn = np.mean([to_base(P, T, Rm, t) for P, T in zip(P_list, T_list)], axis=0)
        if np.linalg.norm(Xn - X) < 1e-10:
            X = Xn
            break
        X = Xn
    res = [float(np.linalg.norm(Rm @ P + t - (Qi @ np.array([X[0], X[1], X[2], 1.0]))[:3]))
           for P, Qi in zip(P_list, Qs)]
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T, X, res


def board_in_base_spread(T_flanges, T_cam_boards, T_fc):
    """用某个 T_fc 把每帧板位姿换到 base 系，返回 (平移离散 mm, 旋转离散 deg, 均值板位姿)。
    真值一致时这两项应接近 0，是判断标定质量最直接的指标。"""
    Bs = [Tf @ T_fc @ Tb for Tf, Tb in zip(T_flanges, T_cam_boards)]
    ts = np.array([B[:3, 3] for B in Bs])
    t_spread = float(np.linalg.norm(ts - ts.mean(axis=0), axis=1).max() * 1000.0)
    Rm = Bs[0][:3, :3]
    r_spread = float(max(rot_deg(B[:3, :3], Rm) for B in Bs))
    return t_spread, r_spread


def solve_handeye_axxb(T_flanges, T_cam_boards):
    """cv2.calibrateHandEye（AX=XB，用整块板的 6 自由度位姿，不用单点）。

    对每种方法都算 X 与 X^{-1}（两种约定），用"板在 base 下的一致性"挑最可信的那个。
    返回 [(方法, 平移离散mm, 旋转离散deg, 4x4)]，按离散度从小到大。
    """
    import cv2
    R_g2b = [T[:3, :3] for T in T_flanges]
    t_g2b = [T[:3, 3].reshape(3, 1) for T in T_flanges]
    R_t2c = [T[:3, :3] for T in T_cam_boards]
    t_t2c = [T[:3, 3].reshape(3, 1) for T in T_cam_boards]
    methods = [("TSAI", cv2.CALIB_HAND_EYE_TSAI), ("PARK", cv2.CALIB_HAND_EYE_PARK),
               ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD), ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
               ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)]
    out = []
    for name, m in methods:
        try:
            Rc, tc = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
        except Exception:  # noqa: BLE001
            continue
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = Rc, tc.reshape(3)
        for tag, M in ((name, X), (name + "-inv", np.linalg.inv(X))):
            t_sp, r_sp = board_in_base_spread(T_flanges, T_cam_boards, M)
            out.append((tag, t_sp, r_sp, M))
    out.sort(key=lambda r: (r[1] + r[2] * 3.0))
    return out


def rot_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return float(np.degrees(np.linalg.norm(R.from_matrix(Ra.T @ Rb).as_rotvec())))


# ---------------------------------------------------------------- 桥接 ----

def detect_charuco(gray, board, detector, K, dist):
    """在灰度图里找 ChArUco 内角点并解板面位姿，返回板心在相机系的点（纯函数，便于离线测试）。"""
    import cv2
    corners, ids, _, _ = detector.detectBoard(gray)
    if corners is None or ids is None or len(ids) == 0:
        return None
    obj_all = board.getChessboardCorners()
    idx = ids.reshape(-1).astype(int)
    obj = np.asarray(obj_all, dtype=np.float64)[idx]
    img_pts = corners.reshape(-1, 2).astype(np.float64)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE,
                                             reprojectionError=4.0, iterationsCount=200,
                                             confidence=0.995)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    # 板心固定用“全部内角点”的均值，不能用本次检出的子集——不同视角漏检的角点不同，
    # 用子集均值会让每帧对应板面上不同的物理点，把手眼解的旋转拉偏几度。
    p_board = np.asarray(obj_all, dtype=np.float64).mean(axis=0)
    P_cam = (R @ p_board + tvec.reshape(3)).astype(float)
    proj, _ = cv2.projectPoints(p_board.reshape(1, 3), rvec, tvec, K, dist)
    u, v = proj.reshape(2)
    # 重投影误差
    reproj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(reproj.reshape(-1, 2) - img_pts, axis=1)
    return {"n_corners": int(len(idx)), "n_inliers": int(0 if inl is None else len(inl)),
            "center": [float(u), float(v)], "P_cam": [float(x) for x in P_cam],
            "R": R.tolist(), "t": [float(x) for x in tvec.reshape(3)],
            "rms_px": float(np.sqrt(np.mean(err ** 2))), "n_obj": int(len(obj_all))}


class Bridge:
    """rclpy 节点 + 运动学 + 动作下发。回调里只存最新数据，逻辑在调用线程里做。"""

    def __init__(self, cfg: dict, dry_run: bool, step_m: float = 0.012, hz: float = 12.0,
                 node_name: str = "handeye_web"):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from geometry_msgs.msg import Pose

        self._Pose = Pose
        self._JointState = JointState
        self.cfg = cfg
        self.dry = dry_run
        self.step_m = step_m
        self.hz = hz
        # z 下限：优先用清单里的桌面高度（base 系负值），否则用工作空间下限；
        # 点击到更低的地面/桌面时会用 note_surface() 自动放宽 5mm。
        th = cfg.get("table_height_m")
        self.z_lo = (float(th) - 0.005) if th is not None else BOUNDS["z"][0]
        self.lock = threading.Lock()
        self.move_lock = threading.Lock()
        self.color = None
        self.color_enc = "bgr8"
        self.color_stamp = 0.0
        self.depth = None
        self.depth_stamp = 0.0
        self.joints_rad = None
        self.joints_deg = None
        self.joints_stamp = 0.0
        self.end_pose_T = None
        self.end_pose_stamp = 0.0
        self.chain = build_chain(Path(cfg["urdf"]))
        self.tip = cfg["tip_in_flange"]
        self.T_fc = cfg["T_flange_camera"]

        self.node = Node(node_name)
        self.pose_pub = self.node.create_publisher(Pose, "/arm/pos_command", 1)
        self.joint_pub = self.node.create_publisher(JointState, "/arm/joint_command", 1)
        self.node.create_subscription(Image, cfg["rgb_topic"], self._on_color, qos_profile_sensor_data)
        self.node.create_subscription(Image, cfg["depth_topic"], self._on_depth, qos_profile_sensor_data)
        self.node.create_subscription(JointState, "/arm/joint_states_single", self._on_joints, 10)
        self.node.create_subscription(Pose, "/arm/end_pose", self._on_pose, 10)
        self._spin = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin.start()

    # ---- 回调 ----
    def _on_color(self, msg):
        try:
            h, w = msg.height, msg.width
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                img = buf.reshape(h, w, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                img = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:
                return
            with self.lock:
                self.color = img
                self.color_enc = msg.encoding
                self.color_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            print(f"[color] {exc}", flush=True)

    def _on_depth(self, msg):
        try:
            h, w = msg.height, msg.width
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            with self.lock:
                self.depth = buf.reshape(h, w).copy()
                self.depth_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            print(f"[depth] {exc}", flush=True)

    def _on_joints(self, msg):
        try:
            pos = list(msg.position[:7])
            while len(pos) < 6:
                pos.append(0.0)
            with self.lock:
                self.joints_rad = np.array(pos[:6], dtype=float)
                self.joints_deg = np.degrees(self.joints_rad)
                self.joints_stamp = time.time()
        except Exception:  # noqa: BLE001
            pass

    def _on_pose(self, msg):
        with self.lock:
            self.end_pose_T = pose_to_T(msg)
            self.end_pose_stamp = time.time()

    # ---- 读数 ----
    def ready(self) -> dict:
        now = time.time()
        with self.lock:
            return {
                "color": self.color is not None and now - self.color_stamp < 3.0,
                "depth": self.depth is not None and now - self.depth_stamp < 3.0,
                "arm": self.joints_rad is not None and now - self.joints_stamp < 3.0,
            }

    def flange(self):
        with self.lock:
            q = None if self.joints_rad is None else self.joints_rad.copy()
        if q is None:
            return None
        Tm = self.chain.forward_kinematics(q).matrix()
        T = np.eye(4)
        T[:3, :3] = Tm[:3, :3]
        T[:3, 3] = Tm[:3, 3]
        return T

    def tip_xyz(self):
        T = self.flange()
        return None if T is None else (T @ np.r_[self.tip, 1.0])[:3]

    def joints_deg_now(self):
        with self.lock:
            return None if self.joints_deg is None else self.joints_deg.copy()

    def snap_frame(self):
        with self.lock:
            return (None if self.color is None else self.color.copy(),
                    None if self.depth is None else self.depth.copy(),
                    self.color_stamp, self.depth_stamp)

    # ---- 3D ----
    def pixel_to_cam(self, u: int, v: int, win: int = 4):
        _, depth, _, dstamp = self.snap_frame()
        if depth is None:
            raise RuntimeError("没有深度图")
        h, w = depth.shape
        u, v = int(np.clip(u, 0, w - 1)), int(np.clip(v, 0, h - 1))
        y0, y1 = max(0, v - win), min(h, v + win + 1)
        x0, x1 = max(0, u - win), min(w, u + win + 1)
        # depth_units_mm=1.0 表示深度图单位是毫米；统一换成米再做有效区间过滤
        patch_m = depth[y0:y1, x0:x1].astype(np.float64) * self.cfg["depth_units_mm"] / 1000.0
        valid = patch_m[(patch_m > 0.10) & (patch_m < 1.5)]
        if valid.size == 0:
            raise RuntimeError("这个点没有有效深度（换个位置或把目标放远一点）")
        d = float(np.median(valid))
        d_mm = d * 1000.0
        K = self.cfg["K"]
        x = (u - K[0, 2]) / K[0, 0] * d
        y = (v - K[1, 2]) / K[1, 1] * d
        return np.array([x, y, d]), d, d_mm, (time.time() - dstamp)

    def cam_to_base(self, P_cam: np.ndarray, T_flange: np.ndarray) -> np.ndarray:
        T = T_flange @ self.T_fc
        return (T @ np.r_[P_cam, 1.0])[:3]

    # ---- 标定板（ArUco） ----
    def setup_aruco(self, dict_name: str = "DICT_4X4_50", marker_id: int = 0, size_mm: float = 100.0):
        import cv2
        self._cv2 = cv2
        self.aruco_id = int(marker_id)
        self.aruco_size_m = float(size_mm) / 1000.0
        d = getattr(cv2.aruco, dict_name, None)
        if d is None:
            raise SystemExit(f"未知 ArUco 字典 {dict_name}（用 DICT_4X4_50 之类）")
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.board_kind = "aruco"
        self.aruco_det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(d), params)
        print(f"[aruco] 字典 {dict_name}  目标 id={self.aruco_id}  边长 {size_mm:.1f} mm", flush=True)

    def setup_charuco(self, squares_x: int, squares_y: int, square_mm: float, dict_name: str = "DICT_5X5_100"):
        import cv2
        self._cv2 = cv2
        self.board_kind = "charuco"
        d = getattr(cv2.aruco, dict_name, None)
        if d is None:
            raise SystemExit(f"未知 ArUco 字典 {dict_name}")
        self.charuco_board = cv2.aruco.CharucoBoard((int(squares_x), int(squares_y)),
                                                    float(square_mm) / 1000.0,
                                                    float(square_mm) / 1000.0 * 0.75,
                                                    cv2.aruco.getPredefinedDictionary(d))
        self.charuco_det = cv2.aruco.CharucoDetector(self.charuco_board)
        print(f"[charuco] {squares_x}x{squares_y} 格，格子 {square_mm:.1f} mm，{dict_name}"
              f"（内角点 {self.charuco_board.getChessboardCorners().shape[0]} 个）", flush=True)

    def detect_marker(self, want_id: int = None):
        """返回标定板中心在相机系的 3D 点（深度法 + PnP 法互相校验）。"""
        if getattr(self, "board_kind", "aruco") == "charuco":
            color, _, cstamp, _ = self.snap_frame()
            if color is None:
                return None
            import cv2
            gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
            r = detect_charuco(gray, self.charuco_board, self.charuco_det, self.cfg["K"], self.cfg["dist"])
            if r is None:
                return None
            info = {"board": "charuco", "marker_id": -1, "center": r["center"],
                    "n_corners": r["n_corners"], "rms_px": round(r["rms_px"], 3),
                    "P_cam": r["P_cam"], "P_cam_pnp": r["P_cam"], "source": "charuco",
                    "R_cam_board": r["R"], "t_cam_board": r["t"],
                    "frame_age": round(time.time() - cstamp, 2)}
            try:
                Pd, d, _, _ = self.pixel_to_cam(int(round(r["center"][0])), int(round(r["center"][1])))
                info["P_cam_depth"] = [float(x) for x in Pd]
                info["depth_m"] = round(float(d), 4)
                diff = float(np.linalg.norm(Pd - np.asarray(r["P_cam"])))
                info["depth_pnp_diff_mm"] = round(diff * 1000, 1)
                if diff > 0.05:
                    info["warn"] = f"深度与 PnP 差 {diff*1000:.0f}mm（板子不平/斜看/内参偏）"
            except RuntimeError as exc:
                info["depth_err"] = str(exc)
            return info
        if not hasattr(self, "aruco_det"):
            return None
        cv2 = self._cv2
        color, _, cstamp, _ = self.snap_frame()
        if color is None:
            return None
        gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.aruco_det.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return None
        ids = ids.reshape(-1)
        wid = self.aruco_id if want_id is None else int(want_id)
        pick, best = None, -1.0
        for i, mid in enumerate(ids):
            if wid >= 0 and int(mid) != wid:
                continue
            area = abs(cv2.contourArea(corners[i].reshape(-1, 2)))
            if area > best:
                best, pick = area, i
        if pick is None:
            return None
        pts = corners[pick].reshape(4, 2)
        cu, cv_ = pts.mean(axis=0)
        info = {"marker_id": int(ids[pick]),
                "center": [float(cu), float(cv_)],
                "frame_age": round(time.time() - cstamp, 2),
                "size_mm": self.aruco_size_m * 1000.0}
        P_depth = None
        try:
            P_depth, d, _, _ = self.pixel_to_cam(int(round(cu)), int(round(cv_)))
            info["P_cam_depth"] = [float(x) for x in P_depth]
            info["depth_m"] = round(float(d), 4)
        except RuntimeError as exc:
            info["P_cam_depth"] = None
            info["depth_err"] = str(exc)
        hs = self.aruco_size_m / 2.0
        objp = np.array([[-hs, hs, 0.0], [hs, hs, 0.0], [hs, -hs, 0.0], [-hs, -hs, 0.0]])
        P_pnp = None
        try:
            ok, _rv, tv = cv2.solvePnP(objp, pts.astype(np.float64), self.cfg["K"],
                                       self.cfg["dist"], flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                P_pnp = tv.reshape(3).astype(float)
                info["P_cam_pnp"] = [float(x) for x in P_pnp]
        except Exception as exc:  # noqa: BLE001
            info["pnp_err"] = str(exc)
        diff = None
        if P_depth is not None and P_pnp is not None:
            diff = float(np.linalg.norm(P_depth - P_pnp))
            info["depth_pnp_diff_mm"] = round(diff * 1000, 1)
        if P_depth is not None:
            info["P_cam"] = [float(x) for x in P_depth]
            info["source"] = "depth"
            if diff is not None and diff > 0.05:
                info["warn"] = f"深度与 PnP 差 {diff*1000:.0f}mm（印刷尺寸不准/内参偏/斜看）"
        elif P_pnp is not None:
            info["P_cam"] = [float(x) for x in P_pnp]
            info["source"] = "pnp"
        else:
            return None
        return info

    # ---- 运动 ----
    def _publish_flange(self, flange_xyz, quat):
        self.pose_pub.publish(pose_msg(flange_xyz, quat))

    def note_surface(self, z: float):
        """把点击观察到的表面高度（base 系）当作安全下限：允许到表面以下 5mm。"""
        z = float(z)
        if z - 0.005 < self.z_lo:
            self.z_lo = z - 0.005
            print(f"[bounds] 观察到表面 z={z:.3f} m，z 下限放宽到 {self.z_lo:.3f}", flush=True)

    def move_tip(self, target_tip, quat=None, timeout=12.0):
        """把爪尖从当前位置直线插值挪到 target_tip（保持/指定姿态）。"""
        with self.move_lock:
            T = self.flange()
            if T is None:
                return {"ok": False, "msg": "还读不到机械臂关节角"}
            if quat is None:
                quat = R.from_matrix(T[:3, :3]).as_quat()
            target_tip = np.asarray(target_tip, dtype=float)
            for axis, (lo, hi) in BOUNDS.items():
                val = float(target_tip["xyz".index(axis)])
                if axis == "z":
                    lo = self.z_lo
                if not (lo <= val <= hi):
                    return {"ok": False, "msg": f"目标 {axis}={val:.3f} 超出限位 [{lo:.3f},{hi}]"}
            start = (T @ np.r_[self.tip, 1.0])[:3]
            dist = float(np.linalg.norm(target_tip - start))
            if dist > 0.6:
                return {"ok": False, "msg": f"单次位移 {dist:.2f} m 太大，拒绝"}
            Rm = R.from_quat(quat).as_matrix()
            n = max(2, int(math.ceil(dist / self.step_m)))
            plan = [start + (target_tip - start) * (i / n) for i in range(1, n + 1)]
            if self.dry:
                return {"ok": True, "msg": f"[dry] 不动作，目标 {np.round(target_tip,3).tolist()}，{n} 步"}
            for p in plan:
                self._publish_flange(p - Rm @ self.tip, quat)
                time.sleep(1.0 / self.hz)
            for _ in range(3):
                self._publish_flange(target_tip - Rm @ self.tip, quat)
                time.sleep(0.1)
            flange_target = target_tip - Rm @ self.tip
            t0 = time.time()
            err = None
            while time.time() - t0 < timeout:
                with self.lock:
                    Tn = self.end_pose_T
                if Tn is not None:
                    err = float(np.linalg.norm(Tn[:3, 3] - flange_target))
                    if err <= 0.004:
                        return {"ok": True, "msg": f"到位（残差 {err*1000:.1f} mm）", "err_mm": err * 1000}
                time.sleep(0.1)
            tail = "未知" if err is None else f"{err*1000:.1f} mm"
            return {"ok": False, "msg": f"没到位（残差 {tail}）：可能不可达 / 臂未使能 / 被限位挡住"}

    def move_joints_deg(self, target_deg, velocity: int = 20, timeout: float = 12.0):
        with self.move_lock:
            if self.dry:
                return {"ok": True, "msg": f"[dry] 不动作，关节 {np.round(target_deg,1).tolist()}"}
            msg = self._JointState()
            msg.name = [f"joint{i+1}" for i in range(6)]
            msg.position = [float(math.radians(a)) for a in target_deg]
            msg.velocity = [0.0] * 7
            self.joint_pub.publish(msg)
            time.sleep(0.2)
            self.joint_pub.publish(msg)
            t0 = time.time()
            while time.time() - t0 < timeout:
                cur = self.joints_deg_now()
                if cur is not None and float(np.max(np.abs(cur - np.asarray(target_deg)))) < 2.0:
                    return {"ok": True, "msg": "到位"}
                time.sleep(0.1)
            return {"ok": False, "msg": "关节没到位（可能超限位或臂未使能）"}

    def hold(self):
        """急停：反复发当前位姿，让 SDK 停在原地。"""
        T = self.flange()
        if T is None or self.dry:
            return
        xyz, quat = T_to_pose_fields(T)
        for _ in range(10):
            self._publish_flange(xyz, quat)
            time.sleep(0.05)


# ---------------------------------------------------------------- 会话 ----

class Session:
    def __init__(self, bridge: Bridge, cfg: dict, hover_m: float):
        self.b = bridge
        self.cfg = cfg
        self.hover_m = hover_m
        self.lock = threading.Lock()
        self.mode = "A"
        self.samples = []
        self.last_click = None
        self.candidate = None
        self.obs_index = 0
        self.msg = "就绪"
        self.session_stamp = time.strftime("%Y%m%d_%H%M%S")
        self.live_path = Path.home() / ".robonix" / "handeye_sessions" / "live.json"
        self._restore_live()

    # ---- 断点保护：每记一条就落盘，服务重启/误关也不会丢 ----
    def _persist(self):
        try:
            self.live_path.parent.mkdir(parents=True, exist_ok=True)
            with self.lock:
                data = {"stamp": self.session_stamp, "mode": self.mode,
                        "obs_index": self.obs_index, "samples": self.samples}
            tmp = self.live_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
            tmp.replace(self.live_path)
        except Exception as exc:
            print(f"[persist] {exc}", flush=True)

    def _restore_live(self):
        try:
            if not self.live_path.exists():
                return
            data = json.loads(self.live_path.read_text(encoding="utf-8"))
            samples = data.get("samples") or []
            if not samples:
                return
            self.samples = samples
            self.mode = data.get("mode") or "A"
            self.obs_index = int(data.get("obs_index") or 0)
            self.session_stamp = data.get("stamp") or self.session_stamp
            self.msg = f"已恢复上次未保存的 {len(samples)} 条样本（模式 {self.mode}，{self.session_stamp}）"
            print(f"[restore] {self.msg}", flush=True)
        except Exception as exc:
            print(f"[restore] {exc}", flush=True)

    # 观察位：以清单里的 observe_joints_deg 为基准，换几个视角
    OBS_OFFSETS = [
        {},                        # 0: 标定观察位（清单里的 observe_joints_deg）
        {0: 14.0}, {0: -14.0},     # 1/2: 绕基座转，视角平移
        {5: 25.0}, {5: -25.0},     # 3/4: 绕末端滚转，视角旋转（手眼最好用）
        {3: 12.0}, {3: -12.0},     # 5/6: 腕部俯仰
        {0: 10.0, 5: 20.0},        # 7: 混合
        {0: -10.0, 5: -20.0},
    ]

    def obs_pose_deg(self, idx: int):
        base = np.asarray(self.cfg["observe_joints_deg"], dtype=float).copy()
        for j, delta in self.OBS_OFFSETS[idx % len(self.OBS_OFFSETS)].items():
            base[j] += delta
        return base

    def click(self, u: int, v: int, allow_motion: bool):
        color, _, cstamp, _ = self.b.snap_frame()
        if color is None:
            return {"ok": False, "msg": "还没有画面"}
        h, w = color.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return {"ok": False, "msg": f"像素 ({u},{v}) 超出画面 {w}x{h}"}
        try:
            P_cam, d, d_mm, age = self.b.pixel_to_cam(u, v)
        except RuntimeError as exc:
            # 点失败必须把上一次的目标清掉，否则 record() 会拿旧目标凑出一条样本
            with self.lock:
                self.last_click = None
                self.candidate = None
            return {"ok": False, "msg": str(exc)}
        T = self.b.flange()
        if T is None:
            with self.lock:
                self.last_click = None
                self.candidate = None
            return {"ok": False, "msg": "还读不到机械臂关节角"}
        P_base = self.b.cam_to_base(P_cam, T)
        # 只把“够得着的表面”当成安全下限，避免点到远处地面/墙面把限位放太松
        if (BOUNDS["x"][0] <= P_base[0] <= BOUNDS["x"][1]
                and BOUNDS["y"][0] <= P_base[1] <= BOUNDS["y"][1] and d < 0.8):
            self.b.note_surface(float(P_base[2]))
        info = {
            "pixel": [u, v],
            "depth_m": round(d, 4),
            "P_cam": [round(float(x), 4) for x in P_cam],
            "P_base": [round(float(x), 4) for x in P_base],
            "frame_age": round(age, 2),
        }
        with self.lock:
            self.last_click = dict(info, T_flange=T.copy(), P_cam=P_cam.copy(), P_base=P_base.copy(),
                                   quat=R.from_matrix(T[:3, :3]).as_quat())
            self.candidate = None
        if self.mode == "B":
            with self.lock:
                self.candidate = dict(info, T_flange=T.copy(), P_cam=P_cam.copy(),
                                      joints_deg=self.b.joints_deg_now())
            return {"ok": True, "mode": "B", "msg": "已取点，记得点同一特征；再按「换观察位」", **info}
        if not allow_motion:
            return {"ok": True, "mode": "A", "msg": "只算不动作", **info}
        target = P_base
        hover = P_base + np.array([0.0, 0.0, self.hover_m])
        res = self.b.move_tip(hover)
        with self.lock:
            self.msg = res["msg"]
        return {"ok": res["ok"], "mode": "A", "msg": res["msg"], **info,
                "hover_m": round(float(self.hover_m), 3)}

    def descend(self):
        with self.lock:
            lc = self.last_click
        if not lc:
            return {"ok": False, "msg": "先点一个点"}
        if self.mode != "A":
            return {"ok": False, "msg": "模式 B 不下探"}
        res = self.b.move_tip(lc["P_base"], quat=lc["quat"])
        with self.lock:
            self.msg = res["msg"]
        return res

    def hover(self):
        with self.lock:
            lc = self.last_click
        if not lc:
            return {"ok": False, "msg": "先点一个点"}
        target = lc["P_base"] + np.array([0.0, 0.0, self.hover_m])
        res = self.b.move_tip(target, quat=lc["quat"])
        with self.lock:
            self.msg = res["msg"]
        return res

    def jog(self, dx_mm: float, dy_mm: float, dz_mm: float):
        tip = self.b.tip_xyz()
        if tip is None:
            return {"ok": False, "msg": "还读不到机械臂"}
        target = tip + np.array([dx_mm, dy_mm, dz_mm]) / 1000.0
        res = self.b.move_tip(target)
        with self.lock:
            self.msg = res["msg"]
        return res

    def record(self):
        with self.lock:
            mode = self.mode
            lc = self.last_click
            cand = self.candidate
        T = self.b.flange()
        tip = self.b.tip_xyz()
        jd = self.b.joints_deg_now()
        if T is None or tip is None or jd is None:
            return {"ok": False, "msg": "还读不到机械臂状态"}
        if mode == "A":
            if not lc:
                return {"ok": False, "msg": "先点一个点再记录"}
            err = float(np.linalg.norm(tip - lc["P_base"]))
            sample = {
                "mode": "A", "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                "pixel": lc["pixel"], "depth_m": lc["depth_m"], "P_cam": [float(x) for x in lc["P_cam"]],
                "P_base_target": [float(x) for x in lc["P_base"]],
                "T_flange": T.tolist(), "joints_deg": [float(x) for x in jd],
                "tip_now": [float(x) for x in tip], "err_mm": round(err * 1000, 2),
            }
        else:
            if not cand:
                return {"ok": False, "msg": "先在画面上点一下桌面上的固定目标"}
            sample = {
                "mode": "B", "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                "pixel": cand["pixel"], "depth_m": cand["depth_m"],
                "P_cam": [float(x) for x in cand["P_cam"]],
                "T_flange": cand["T_flange"].tolist(),
                "joints_deg": [float(x) for x in (cand["joints_deg"] if cand["joints_deg"] is not None else jd)],
                "R_cam_board": cand.get("R_cam_board"),
                "t_cam_board": cand.get("t_cam_board"),
            }
        with self.lock:
            self.samples.append(sample)
            n = len(self.samples)
            if mode == "B":
                self.candidate = None
            self.msg = f"已记录第 {n} 条"
        self._persist()
        extra = ""
        if mode == "A":
            extra = f"（爪尖离点击点 {sample['err_mm']:.1f} mm）"
        return {"ok": True, "msg": f"已记录第 {n} 条{extra}", "count": n, "mode": mode}

    def set_mode(self, mode: str):
        if mode not in ("A", "B"):
            return {"ok": False, "msg": f"未知模式 {mode}"}
        with self.lock:
            self.mode = mode
            self.candidate = None
            self.last_click = None
            dropped = len(self.samples)
            self.samples = []
            self.msg = "模式 A：点哪够哪" if mode == "A" else "模式 B：多视角点同一固定目标"
            if dropped:
                self.msg += f"（已清空之前 {dropped} 条样本，A/B 样本不能混算）"
        self._persist()
        return {"ok": True, "mode": mode, "msg": self.msg, "dropped": dropped}

    def home(self):
        res = self.b.move_joints_deg([0.0] * 6)
        with self.lock:
            self.msg = "回 home：" + res["msg"]
        return res

    def observe(self, action: str):
        if self.mode != "B" and action != "home":
            return {"ok": False, "msg": "切到模式 B 才能换观察位"}
        if action == "home":
            self.obs_index = 0
        else:
            self.obs_index += 1
        target = self.obs_pose_deg(self.obs_index)
        self._persist()
        res = self.b.move_joints_deg(target)
        return {"ok": res["ok"], "msg": f"观察位 #{self.obs_index}：{res['msg']}",
                "joints_deg": [round(float(x), 2) for x in target]}

    # ---- 标定板 ----
    def board(self):
        info = self.b.detect_marker()
        if info is None:
            return {"ok": False, "msg": "画面里没看到标定板（确认 id/字典/尺寸，且板子完整在画面内、别反光）"}
        T = self.b.flange()
        jd = self.b.joints_deg_now()
        if T is None or jd is None:
            return {"ok": False, "msg": "还读不到机械臂状态"}
        P_cam = np.asarray(info["P_cam"], dtype=float)
        with self.lock:
            self.candidate = {"pixel": [round(info["center"][0]), round(info["center"][1])],
                              "depth_m": info.get("depth_m"), "P_cam": P_cam,
                              "T_flange": T, "joints_deg": jd, "source": info["source"],
                              "R_cam_board": info.get("R_cam_board"),
                              "t_cam_board": info.get("t_cam_board")}
        if info.get("board") == "charuco":
            msg = f"ChArUco 板已就位：{info['n_corners']} 个角点，重投影 {info['rms_px']:.2f} px"
        else:
            msg = f"标定板 id{info['marker_id']} 已就位（{info['source']}）"
        return {"ok": True, "msg": msg, **info}

    def auto_collect(self, count: int = 8, settle_s: float = 1.6, save_after: bool = True):
        """自动换视角 + 自动检测标定板 + 自动记录；最后直接解算。"""
        if self.mode != "B":
            return {"ok": False, "msg": "先切到模式 B 再自动采集"}
        count = max(3, min(int(count), 16))
        lines = []
        got = 0
        for _ in range(count):
            self.obs_index += 1
            target = self.obs_pose_deg(self.obs_index)
            tag = f"位姿#{self.obs_index}"
            res = self.b.move_joints_deg(target)
            if not res.get("ok"):
                lines.append(f"{tag} 没到位，跳过（{res.get('msg')}）")
                continue
            time.sleep(max(0.6, float(settle_s)))
            info = None
            for _ in range(8):
                info = self.b.detect_marker()
                if info:
                    break
                time.sleep(0.35)
            if not info:
                lines.append(f"{tag} 没看到标定板，跳过")
                continue
            T = self.b.flange()
            jd = self.b.joints_deg_now()
            if T is None or jd is None:
                lines.append(f"{tag} 读不到臂状态，跳过")
                continue
            P_cam = np.asarray(info["P_cam"], dtype=float)
            sample = {"mode": "B", "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                      "pixel": [round(info["center"][0]), round(info["center"][1])],
                      "depth_m": info.get("depth_m"),
                      "P_cam": [float(x) for x in P_cam],
                      "T_flange": T.tolist(),
                      "joints_deg": [float(x) for x in jd],
                      "R_cam_board": info.get("R_cam_board"),
                      "t_cam_board": info.get("t_cam_board"),
                      "board": {"id": info["marker_id"], "source": info["source"],
                                "diff_mm": info.get("depth_pnp_diff_mm")}}
            with self.lock:
                self.samples.append(sample)
                n = len(self.samples)
                self.msg = f"自动采集：第 {n} 条"
            self._persist()
            got += 1
            lines.append(f"{tag} 收录第 {n} 条（{info['source']}"
                         + (f"，深度/PnP 差 {info['depth_pnp_diff_mm']}mm" if info.get("depth_pnp_diff_mm") is not None else "") + "）")
        out = {"ok": True, "collected": got, "lines": lines, "count": len(self.samples)}
        if save_after and self.samples:
            rep = self.save(exit_after=False)
            out["report"] = rep["report"]
            out["dir"] = rep["dir"]
            out["rms_mm"] = rep.get("rms_mm")
        elif got == 0:
            out["ok"] = False
            out["msg"] = "一条都没采到；检查标定板是否在观察位视野里（先点「检测标定板」看能不能看到）"
        return out

    # ---- 保存 / 解算 ----
    def save(self, exit_after: bool = True):
        stamp = self.session_stamp
        out = Path.home() / ".robonix" / "handeye_sessions" / stamp
        if out.exists():
            stamp = time.strftime("%Y%m%d_%H%M%S")
            out = Path.home() / ".robonix" / "handeye_sessions" / stamp
        out.mkdir(parents=True, exist_ok=True)
        with self.lock:
            samples = [s for s in self.samples if s.get("mode") == self.mode]
            mode = self.mode
        report = [f"# 手眼标定会话 {stamp}",
                  f"清单: {self.cfg['manifest']}",
                  f"模式: {mode}", f"样本: {len(samples)}", ""]
        result = {"samples": len(samples), "dir": str(out)}
        if mode == "A":
            errs = [s["err_mm"] for s in samples]
            if errs:
                report.append(f"点到爪尖误差: mean {np.mean(errs):.1f} mm, max {np.max(errs):.1f} mm")
                report.append("（模式 A 只能看偏差；要解手眼矩阵请用模式 B）")
            result["err_mean_mm"] = round(float(np.mean(errs)), 2) if errs else None
        if mode == "B":
            if len(samples) >= 3:
                P_list = [np.array(s["P_cam"]) for s in samples]
                T_list = [np.array(s["T_flange"]) for s in samples]
                Ts = [T[:3, :3] for T in T_list]
                spread = max(rot_deg(Ts[i], Ts[j]) for i in range(len(Ts)) for j in range(i + 1, len(Ts)))
                T_new, X, res = solve_handeye(P_list, T_list, self.cfg["T_flange_camera"])
                rms = float(np.sqrt(np.mean(np.square(res))))
                if spread < 8.0:
                    report.append(f"⚠ 位姿差异只有 {spread:.1f}°，解不稳；多换几个角度再录")
                report.append(f"位姿差异 max {spread:.1f}°")
                if len(samples) == 3:
                    report.append("⚠ 只有 3 组：9 个约束正好等于 9 个未知数，残差必然≈0，"
                                  "解出来的矩阵只作参考；至少 5 组才有真正的残差检验")
                report.append(f"残差 RMS {rms*1000:.1f} mm（每点 {', '.join(f'{r*1000:.1f}' for r in res)} mm）")
                med = float(np.median(res))
                if med > 0:
                    bad = [i + 1 for i, r in enumerate(res) if r > 3.0 * med and r * 1000 > 8.0]
                    if bad:
                        report.append(f"⚠ 第 {', '.join(map(str, bad))} 条残差偏大，可能点到别的特征了；建议删掉重采")
                loo_rms = None
                if len(samples) >= 6:
                    # 留一交叉验证：用其余样本解，预测被留出的那条，衡量“样本够不够/有没有过拟合”
                    loo = []
                    for i in range(len(samples)):
                        Pi = [p for j, p in enumerate(P_list) if j != i]
                        Ti = [t for j, t in enumerate(T_list) if j != i]
                        T_i, X_i, _ = solve_handeye(Pi, Ti, self.cfg["T_flange_camera"])
                        w = (T_list[i] @ np.r_[T_i[:3, :3] @ P_list[i] + T_i[:3, 3], 1.0])[:3]
                        loo.append(float(np.linalg.norm(w - X_i)))
                    loo_rms = float(np.sqrt(np.mean(np.square(loo))))
                    report.append(f"留一验证 RMS {loo_rms*1000:.1f} mm（最大 {max(loo)*1000:.1f} mm）"
                                  f" —— 这是“没用这条样本解算、只用它检验”的误差")
                    if loo_rms > 2.0 * max(rms, 1e-6):
                        report.append("⚠ 留一误差比拟合残差大 2 倍以上：样本可能有离群点，或视角/目标不够多样")
                # --- AX=XB 交叉验证：用整块板的 6DoF 位姿，约束比单点多 ---
                note = None
                board_spread_used = None
                if all(s.get("R_cam_board") and s.get("t_cam_board") for s in samples):
                    Tbs = []
                    for s in samples:
                        Tb = np.eye(4)
                        Tb[:3, :3] = np.asarray(s["R_cam_board"], dtype=float)
                        Tb[:3, 3] = np.asarray(s["t_cam_board"], dtype=float)
                        Tbs.append(Tb)
                    cands = solve_handeye_axxb(T_list, Tbs)
                    if cands:
                        t_p, r_p = board_in_base_spread(T_list, Tbs, T_new)
                        report.append("")
                        report.append("板在 base 下的离散度（越小越可信）：")
                        for tag, t_sp, r_sp, M in cands[:5]:
                            report.append(f"  AX=XB {tag:<14} {t_sp:6.2f} mm / {r_sp:5.2f}°")
                        report.append(f"  点法（上面残差那套）    {t_p:6.2f} mm / {r_p:5.2f}°")
                        best = min(cands, key=lambda r: r[1] + r[2] * 3.0)
                        if best[1] + best[2] * 3.0 < t_p + r_p * 3.0:
                            T_new = best[3]
                            board_spread_used = (best[1], best[2])
                            note = (f"✅ 采用 AX=XB({best[0]})：板位散 {best[1]:.2f} mm / {best[2]:.2f}°"
                                    f"（点法 {t_p:.2f} mm / {r_p:.2f}°），矩阵已换成它")
                        else:
                            board_spread_used = (t_p, r_p)
                            note = (f"保留点法：板位散 {t_p:.2f} mm / {r_p:.2f}°"
                                    f"（AX=XB 最好 {best[1]:.2f} mm / {best[2]:.2f}°，{best[0]}）")
                        report.append(note)
                report.append("")
                report.append("新 T_flange_camera:")
                for row in T_new:
                    report.append("  " + " ".join(f"{v: .9f}" for v in row))
                report.append("")
                report.append("旧值:")
                for row in self.cfg["T_flange_camera"]:
                    report.append("  " + " ".join(f"{v: .9f}" for v in row))
                delta_t = np.linalg.norm(T_new[:3, 3] - self.cfg["T_flange_camera"][:3, 3])
                delta_r = rot_deg(T_new[:3, :3], self.cfg["T_flange_camera"][:3, :3])
                report.append(f"平移差 {delta_t*1000:.1f} mm / 旋转差 {delta_r:.2f}°")
                yaml_block = "      gripper_camera:\n" + "\n".join(
                    "        - [" + ", ".join(f"{v:.9f}" for v in row) + "]" for row in T_new)
                report.append("")
                report.append("清单片段（manifests ... skill.depth_grasp.config.calib）:")
                report.append(yaml_block)
                (out / "handeye.json").write_text(json.dumps({
                    "T_flange_camera": T_new.tolist(), "world_point": X.tolist(),
                    "rms_mm": rms * 1000, "per_sample_mm": [r * 1000 for r in res],
                    "loo_rms_mm": None if loo_rms is None else loo_rms * 1000,
                    "spread_deg": spread, "yaml": yaml_block,
                    "board_spread_used_mm": None if board_spread_used is None else round(board_spread_used[0], 2),
                    "board_spread_used_deg": None if board_spread_used is None else round(board_spread_used[1], 2),
                    "solver_note": note,
                }, ensure_ascii=False, indent=2), encoding="utf-8")
                result.update({"rms_mm": round(rms * 1000, 2), "spread_deg": round(spread, 1),
                               "loo_rms_mm": None if loo_rms is None else round(loo_rms * 1000, 2),
                               "T_flange_camera": T_new.tolist()})
            else:
                report.append("样本不足 3 条，未解算")
        (out / "samples.json").write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
        text = "\n".join(report) + "\n"
        (out / "report.txt").write_text(text, encoding="utf-8")
        result["report"] = text
        print("\n" + text, flush=True)
        with self.lock:
            self.samples = []
            self.last_click = None
            self.candidate = None
            self.session_stamp = time.strftime("%Y%m%d_%H%M%S")
        try:
            self.live_path.unlink()
        except FileNotFoundError:
            pass
        if not exit_after:
            result["report"] = text + f"\n（已保存到 {out}，服务继续运行）"
        return result


# ---------------------------------------------------------------- HTTP ----

PAGE = r"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>手眼标定</title><style>
:root{color-scheme:dark}body{margin:0;background:#111;color:#ddd;font:14px/1.5 system-ui,"Noto Sans CJK SC",sans-serif}
#top{display:flex;gap:12px;align-items:center;padding:8px 12px;background:#1b1b1b;flex-wrap:wrap}
#wrap{position:relative;display:inline-block;margin:8px 12px;cursor:crosshair}
#video{max-width:96vw;max-height:70vh;display:block;border:1px solid #333;cursor:crosshair}
#bar{display:flex;gap:8px;padding:6px 12px;flex-wrap:wrap}
button{background:#2a2a2a;color:#ddd;border:1px solid #444;border-radius:6px;padding:6px 12px;cursor:pointer;font-size:13px}
button:hover{background:#333}button.on{background:#2d5b8a;border-color:#4a8ac8}
button.stop{background:#7a1f1f;border-color:#a33}button.save{background:#1f5a2a;border-color:#3a8}
#hint{color:#888;padding:4px 12px;font-size:12px}
#msg{padding:8px 12px;min-height:20px;color:#7fd}
.badge{padding:2px 10px;border-radius:10px;background:#264;color:#cfe}
#stat{color:#9ab}
label{color:#9ab}
input[type=number]{width:56px;background:#222;color:#ddd;border:1px solid #444;border-radius:4px;padding:5px 6px}
</style></head><body>
<div id="top">
  <span class="badge" id="mode">模式 A · 点哪够哪</span>
  <button id="bA">A 点哪够哪</button><button id="bB">B 多视角手眼</button>
  <label><input type="checkbox" id="allow" checked> 允许动作</label>
  <span id="stat">连接中…</span>
</div>
<div id="wrap"><img id="video" src="/stream"></div>
<div id="bar">
  <button class="stop" id="stop">停止</button>
  <button id="descend">下探 (d)</button><button id="hover">回悬停 (h)</button>
  <button id="obsNext">换观察位 (n)</button><button id="obsHome">回观察位</button>
  <button id="homeBtn">回 home (0)</button>
  <span style="color:#666">|</span>
  <button id="boardBtn">检测标定板</button>
  <input id="nauto" type="number" value="8" min="3" max="16" title="自动采集组数">
  <button id="autoBtn">自动采集</button>
  <span style="color:#666">|</span>
  <button data-jog="1,0,0">X+1</button><button data-jog="-1,0,0">X-1</button>
  <button data-jog="0,1,0">Y+1</button><button data-jog="0,-1,0">Y-1</button>
  <button data-jog="0,0,1">Z+1</button><button data-jog="0,0,-1">Z-1</button>
  <button data-jog="10,0,0">X+10</button><button data-jog="-10,0,0">X-10</button>
  <button data-jog="0,0,10">Z+10</button><button data-jog="0,0,-10">Z-10</button>
  <span style="color:#666">|</span>
  <button class="save" id="saveKeep">保存 (S)</button>
  <button class="save" id="save">保存并退出 (ESC)</button>
</div>
<div id="hint">空格=记录  S=保存（不停）  ESC=保存并退出  d=下探到点击点  h=回悬停  n=换观察位  0=回 home  方向键=XY 微调 2mm · 模式 A 记录“点击点 vs 爪尖实际位置”，模式 B 记录“同一固定目标在不同视角下的相机系 3D 点 + flange 位姿”<br>有 ArUco 标定板时：切到 B → 先「检测标定板」确认能看到 → 填组数 →「自动采集」（自动换 8 个视角、识别、记录、解算、保存，全程约 40 秒）</div>
<div id="msg"></div>
<script>
const $=s=>document.querySelector(s);
let allowMove=true;
async function api(p,body){const r=await fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});return await r.json();}
function flash(o){const m=typeof o==='string'?o:(o.msg||'');$('#msg').textContent=m;if(o&&o.detail)$('#msg').textContent+= ' | '+o.detail;}
async function refresh(){try{const s=await (await fetch('/api/state')).json();
 $('#mode').textContent = s.mode==='A' ? '模式 A · 点哪够哪' : '模式 B · 多视角手眼';
 $('#stat').textContent = `样本 ${s.samples} · 相机 ${s.ready.color?'OK':'×'} · 深度 ${s.ready.depth?'OK':'×'} · 臂 ${s.ready.arm?'OK':'×'} · ` +
   (s.tip?`tcp (${s.tip.map(v=>v.toFixed(3)).join(', ')})`:'tcp -') + (s.dry?' · DRY':'');
 $('#bA').classList.toggle('on',s.mode==='A'); $('#bB').classList.toggle('on',s.mode==='B');
}catch(e){}}
$('#video').onclick=async e=>{const img=e.target, r=img.getBoundingClientRect();
 const u=Math.round((e.clientX-r.left)*img.naturalWidth/r.width), v=Math.round((e.clientY-r.top)*img.naturalHeight/r.height);
 flash(await api('/api/click',{u,v,allow:allowMove})); refresh();};
$('#bA').onclick=async()=>{flash(await api('/api/mode',{mode:'A'}));refresh();};
$('#bB').onclick=async()=>{flash(await api('/api/mode',{mode:'B'}));refresh();};
$('#allow').onchange=e=>{allowMove=e.target.checked;};
$('#stop').onclick=async()=>{flash(await api('/api/stop'));};
$('#descend').onclick=async()=>{flash(await api('/api/descend'));refresh();};
$('#hover').onclick=async()=>{flash(await api('/api/hover'));refresh();};
$('#obsNext').onclick=async()=>{flash(await api('/api/observe',{action:'next'}));refresh();};
$('#obsHome').onclick=async()=>{flash(await api('/api/observe',{action:'home'}));refresh();};
$('#homeBtn').onclick=async()=>{flash(await api('/api/home'));refresh();};
$('#boardBtn').onclick=async()=>{const r=await api('/api/board');$('#msg').textContent=r.msg||JSON.stringify(r);refresh();};
$('#autoBtn').onclick=async()=>{
  $('#msg').textContent='自动采集中：会连续换视角并动臂，请留手在急停旁边…';
  const n=Number($('#nauto').value||8);
  const r=await api('/api/auto',{count:n});
  if(r.report){$('#msg').textContent=r.report;}
  else{$('#msg').textContent=(r.lines||[]).join('  ·  ')||r.msg||'完成';}
  refresh();};
document.querySelectorAll('[data-jog]').forEach(b=>b.onclick=async()=>{const [x,y,z]=b.dataset.jog.split(',').map(Number);
 flash(await api('/api/jog',{dx:x,dy:y,dz:z}));refresh();});
$('#saveKeep').onclick=async()=>{const r=await api('/api/save',{exit:false});$('#msg').textContent=r.report||r.msg||'已保存（继续运行）';refresh();};
$('#save').onclick=async()=>{const r=await api('/api/save',{exit:true});$('#msg').textContent=r.report||r.msg||'已保存并退出';};
document.addEventListener('keydown',async e=>{
 if(e.key===' '){e.preventDefault();flash(await api('/api/record'));refresh();}
 else if(e.key==='Escape'){e.preventDefault();const r=await api('/api/save',{exit:true});$('#msg').textContent=r.report||'已保存并退出';}
 else if(e.key==='s'||e.key==='S'){e.preventDefault();const r=await api('/api/save',{exit:false});$('#msg').textContent=r.report||'已保存（继续运行）';refresh();}
 else if(e.key==='d'||e.key==='D'){flash(await api('/api/descend'));refresh();}
 else if(e.key==='h'||e.key==='H'){flash(await api('/api/hover'));refresh();}
 else if(e.key==='n'||e.key==='N'){flash(await api('/api/observe',{action:'next'}));refresh();}
 else if(e.key==='0'){flash(await api('/api/home'));refresh();}
 else if(e.key==='ArrowLeft'){e.preventDefault();flash(await api('/api/jog',{dx:2}));}
 else if(e.key==='ArrowRight'){e.preventDefault();flash(await api('/api/jog',{dx:-2}));}
 else if(e.key==='ArrowUp'){e.preventDefault();flash(await api('/api/jog',{dy:2}));}
 else if(e.key==='ArrowDown'){e.preventDefault();flash(await api('/api/jog',{dy:-2}));}
});
setInterval(refresh,1000);refresh();
</script></body></html>"""


def make_app(session: Session, bridge: Bridge):
    from flask import Flask, Response, jsonify, request

    app = Flask(__name__)

    @app.get("/")
    def index():
        return Response(PAGE, mimetype="text/html")

    @app.get("/stream")
    def stream():
        def gen():
            while True:
                color, _, _, _ = bridge.snap_frame()
                if color is None:
                    time.sleep(0.05)
                    continue
                img = color.copy()
                with session.lock:
                    mode, n = session.mode, len(session.samples)
                    lc = session.last_click
                    cand = session.candidate
                hud = f"{'B handeye' if mode == 'B' else 'A reach'}  samples={n}"
                if bridge.dry:
                    hud += "  [DRY]"
                cv2.putText(img, hud, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                mark = cand or lc
                if mark:
                    u, v = mark["pixel"]
                    cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
                if not ok:
                    continue
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(buf)).encode() + b"\r\n\r\n" + buf.tobytes() + b"\r\n")
                time.sleep(0.07)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/state")
    def state():
        with session.lock:
            mode, n = session.mode, len(session.samples)
        tip = bridge.tip_xyz()
        tip_px = None
        # 把爪尖投影回画面：用当前关节位姿 + 标定好的 T_flange_camera。
        # 标定准的话，这个点应该正好压在画面上看到的夹爪指尖上（用于自检）。
        if tip is not None:
            T = bridge.flange()
            if T is not None:
                try:
                    p_cam = np.linalg.inv(T @ bridge.T_fc) @ np.r_[tip, 1.0]
                    if p_cam[2] > 0.03:
                        uv = np.asarray(bridge.cfg["K"], dtype=float) @ (p_cam[:3] / p_cam[2])
                        tip_px = [round(float(uv[0]), 1), round(float(uv[1]), 1)]
                except Exception:
                    tip_px = None
        return jsonify({
            "mode": mode, "samples": n, "ready": bridge.ready(), "dry": bridge.dry,
            "z_lo": round(float(bridge.z_lo), 3),
            "tip": None if tip is None else [round(float(x), 4) for x in tip],
            "tip_px": tip_px,
            "msg": session.msg,
        })

    @app.post("/api/click")
    def click():
        d = request.get_json(force=True, silent=True) or {}
        return jsonify(session.click(int(d.get("u", 0)), int(d.get("v", 0)),
                                     bool(d.get("allow", True))))

    @app.post("/api/record")
    def record():
        return jsonify(session.record())

    @app.post("/api/descend")
    def descend():
        return jsonify(session.descend())

    @app.post("/api/hover")
    def hover():
        return jsonify(session.hover())

    @app.post("/api/jog")
    def jog():
        d = request.get_json(force=True, silent=True) or {}
        return jsonify(session.jog(float(d.get("dx", 0)), float(d.get("dy", 0)), float(d.get("dz", 0))))

    @app.post("/api/mode")
    def mode():
        d = request.get_json(force=True, silent=True) or {}
        return jsonify(session.set_mode(str(d.get("mode", "A"))))

    @app.post("/api/observe")
    def observe():
        d = request.get_json(force=True, silent=True) or {}
        return jsonify(session.observe(str(d.get("action", "next"))))

    @app.post("/api/home")
    def home():
        return jsonify(session.home())

    @app.post("/api/stop")
    def stop():
        bridge.hold()
        return jsonify({"ok": True, "msg": "已发急停（保持当前位姿）"})

    @app.post("/api/board")
    def board():
        return jsonify(session.board())

    @app.post("/api/auto")
    def auto():
        d = request.get_json(force=True, silent=True) or {}
        return jsonify(session.auto_collect(int(d.get("count", 8)),
                                            float(d.get("settle_s", 1.6)),
                                            bool(d.get("save", True))))

    @app.post("/api/save")
    def save():
        d = request.get_json(force=True, silent=True) or {}
        exit_after = bool(d.get("exit", True))
        res = session.save(exit_after=exit_after)
        if exit_after:
            def _bye():
                time.sleep(0.8)
                os._exit(0)
            threading.Timer(0.2, _bye).start()
        return jsonify(res)

    return app


# ---------------------------------------------------------------- main ----

def main():
    ap = argparse.ArgumentParser(description="手眼标定网页（点哪够哪 + 多视角手眼）")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--manifest", default=str(ROOT / "robonix_manifest.nx.yaml"))
    ap.add_argument("--dry-run", action="store_true", help="只算不动作")
    ap.add_argument("--hover-mm", type=float, default=30.0)
    ap.add_argument("--step-mm", type=float, default=12.0, help="插值步长")
    ap.add_argument("--hz", type=float, default=12.0)
    ap.add_argument("--dict", default="DICT_4X4_50", help="ArUco 字典名")
    ap.add_argument("--marker-id", type=int, default=0, help="标定板 id，-1 = 任意")
    ap.add_argument("--marker-mm", type=float, default=100.0, help="标定板黑色外框实际边长（毫米）")
    ap.add_argument("--board", choices=["aruco", "charuco"], default="aruco", help="标定板类型")
    ap.add_argument("--squares", default="12x9", help="ChArUco 格子数（横x竖）")
    ap.add_argument("--square-mm", type=float, default=15.0, help="ChArUco 单格边长（毫米）")
    ap.add_argument("--charuco-dict", default="DICT_5X5_100", help="ChArUco 用的码表")
    args = ap.parse_args()

    cfg = load_manifest_cfg(Path(args.manifest))
    for k, v in (cfg.get("env") or {}).items():
        if isinstance(v, str) and k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "LYNX_ROBOT_DOMAIN_ID"):
            os.environ.setdefault(k, v)
    cfg["urdf"] = str(find_urdf(cfg))

    import rclpy
    rclpy.init(args=None)
    bridge = Bridge(cfg, dry_run=args.dry_run, step_m=args.step_mm / 1000.0, hz=args.hz)
    if args.board == "charuco":
        sx, sy = (int(x) for x in args.squares.lower().split("x"))
        bridge.setup_charuco(sx, sy, args.square_mm, args.charuco_dict)
    else:
        bridge.setup_aruco(args.dict, args.marker_id, args.marker_mm)
    sess = Session(bridge, cfg, hover_m=args.hover_mm / 1000.0)
    app = make_app(sess, bridge)

    print("=" * 68, flush=True)
    print(f"手眼标定网页  http://<NX-IP>:{args.port}/   （或 ssh -L {args.port}:127.0.0.1:{args.port}）")
    print(f"画面: {cfg['rgb_topic']}")
    print(f"深度: {cfg['depth_topic']}")
    print(f"内参 fx={cfg['K'][0,0]:.3f} fy={cfg['K'][1,1]:.3f} cx={cfg['K'][0,2]:.3f} cy={cfg['K'][1,2]:.3f}")
    print(f"TCP: {np.round(cfg['tip_in_flange'], 5).tolist()}   handeye 初值平移: "
          f"{np.round(cfg['T_flange_camera'][:3, 3], 4).tolist()}")
    print(f"观察位(deg): {np.round(cfg['observe_joints_deg'], 1).tolist()}")
    if args.board == "charuco":
        print(f"标定板: ChArUco {args.squares} 格 x {args.square_mm:.1f} mm，{args.charuco_dict}（自动采集靠它）")
    else:
        print(f"标定板: {args.dict} id={args.marker_id} 边长 {args.marker_mm:.1f} mm（自动采集靠它）")
    print(f"模式: {'DRY-RUN（不动臂）' if args.dry_run else '真实动作'}   悬停 {args.hover_mm:.0f} mm")
    print("=" * 68, flush=True)
    try:
        app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)
    finally:
        try:
            bridge.node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
