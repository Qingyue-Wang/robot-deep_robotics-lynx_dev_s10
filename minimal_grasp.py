#!/usr/bin/env python3
# Description: **最小可行性验证脚本** —— 两个观察位姿 + 多观察位扫描 → 抓胡萝卜。
#
# 这是给"名字-only、不管在地面还是桌面"方案做的真机验证器：不经过 rbnx、不起技能，
# 只用 CAN 直连臂 + 腕上相机 + 已跑着的抓取视觉服务（127.0.0.1:8772），把整条链路
# 「换观察位找目标 → 找到就按现有抓取几何抓一次」跑通，验证方案可行。
#
# 流程：
#   标称观察位看一眼 → 没找到就用标称位自己那帧深度估支撑面、围它扫 4 个位姿
#   → 还没找到 → 走到第二个观察位看一眼 → 没找到就同样围它扫 4 个位姿
#   → 找到 → 按 grasp.plan / motion.plan_reach 算抓取 → 悬停 → 下爪 → 合爪 → 抬起
#
# 检测器两种（本脚本自带，方便离线/在线对比）：
#   --detector color : 红/粉色 + 长宽比 ≥1.5 的色块（胡萝卜是红色长条，草莓是圆的会被排除）
#                      然后把这个框交给视觉服务量三维（跳过 VLM，快、不依赖网络）
#   --detector vlm   : 直接把目标名字给视觉服务（正式方案的做法，但要等 VLM）
#
# 安全：只做"使能 → 走到位 → 拍照/合爪/抬手"，不碰底盘；出错就停手（保持使能），
# 不会硬合爪。默认结束停在抬起位、夹爪闭合；--park 才回零位失能。
#
# 记录：**每一次"观察"和"扫描"都落盘**（--out 目录，默认 /tmp/minimal_grasp_<时间>）：
#   looks/NN_<label>_color.jpg     彩色原图
#   looks/NN_<label>_depth.png     16 位原始深度（毫米）
#   looks/NN_<label>_annotated.jpg 标注图（检测框 / NOT FOUND）
#   looks/NN_<label>_meta.json     关节角、相机位姿、bbox、找到没、原因、时间
#   looks/index.json               全部帧的索引
#   looks/contact_sheet.jpg        全部帧拼一张总览
#   found_annotated.jpg            找到目标那一帧的标注图（找到时才有）
import argparse
import json
import os
import sys
import time

# ---- 依赖路径：指向当前部署包里那份（和 rbnx 用的是同一份代码/标定） ----
DEPLOY = "/home/dog/.local/share/robonix-console/deployments/default"
PKG = DEPLOY + "/packages/skill-s10-grasp-dev-rbnx"
sys.path.insert(0, PKG)
sys.path.insert(0, PKG + "/grasp_skill/calib")
sys.path.insert(0, DEPLOY + "/host-services/grasp-vision")
sys.path.insert(0, DEPLOY + "/payloads")

import cv2
import numpy as np
import yaml

import pyrealsense2 as rs
from common.robot.direct import DirectArm, ARM_STATUS_TEXT, CTRL_MODE_TEXT, TEACH_STATUS_TEXT
from grasp_skill.calib.calib_io import Calibration
from grasp_skill import observe_scan
import grasp
import motion
import vision_client

MANIFEST = DEPLOY + "/robonix_manifest.yaml"
HOVER_SETTLE_M = 0.005          # 悬停到位判据（米），与 pipeline 一致
DESCEND_ABORT_M = 0.015         # 下爪差超过它就判定"没到位、不合爪"（与 pipeline 一致）
RED_ASPECT_MIN = 1.5            # 胡萝卜是长条；草莓是圆，靠这个排除
RED_MIN_AREA = 500


def load_config():
    manifest = yaml.safe_load(open(MANIFEST, encoding="utf-8"))
    for entry in manifest.get("skill") or []:
        if entry.get("name") == "s10_grasp_dev":
            return entry.get("config") or {}
    raise SystemExit("清单里没有 s10_grasp_dev")


def jstr(joints):
    return "[%s]" % ", ".join("%.2f" % v for v in joints)


# ---------------------------------------------------------------------------
# 视觉：色块框（跳过 VLM 的那条路） / 直接 VLM
# ---------------------------------------------------------------------------

def red_carrot_box(color):
    """红/粉色、长宽比 ≥1.5 的色块 → [x0, y0, x1, y1, angle]；找不到返回 None。"""
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = (cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
            | cv2.inRange(hsv, (168, 90, 70), (180, 255, 255)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < RED_MIN_AREA:
            continue
        (cx, cy), (w, h), angle = cv2.minAreaRect(contour)
        long_side, short_side = max(w, h), max(min(w, h), 1.0)
        if long_side / short_side < RED_ASPECT_MIN:
            continue                       # 圆的东西（草莓别针）不要
        if long_side < short_side:         # 让 angle 表示"长轴相对图像 +x"的转角
            angle += 90.0
        points = cv2.boxPoints(((cx, cy), (w, h), angle)).astype(float)
        box = [points[:, 0].min(), points[:, 1].min(),
               points[:, 0].max(), points[:, 1].max(), angle]
        score = area * (long_side / short_side)
        if best is None or score > best[0]:
            best = (score, box)
    return None if best is None else best[1]


def measure(color, depth, camera_matrix, dist_coeffs, args):
    """看一眼：返回视觉服务的结果（相机系）或抛 ValueError（这一眼没量出来）。"""
    if args.detector == "vlm":
        return vision_client.locate(color, depth, camera_matrix, dist_coeffs,
                                    target=args.target, timeout_s=args.vlm_timeout)
    box = red_carrot_box(color)
    if box is None:
        raise ValueError("画面里没有红色长条目标")
    return vision_client.locate(color, depth, camera_matrix, dist_coeffs,
                                target="", box=box, timeout_s=args.vlm_timeout)


def to_base(measurement, camera_pose):
    """视觉结果（相机系）→ 基座系。点要平移+旋转，方向只旋转，宽度是标量。"""
    pose = np.asarray(camera_pose, dtype=float)
    rotation = pose[:3, :3]
    def point(value):
        return (pose @ np.r_[np.asarray(value, dtype=float), 1.0])[:3]
    return {
        "top_point": point(measurement["top_point"]),
        "plane": {"point": point(measurement["support"]["point"]),
                  "normal": rotation @ np.asarray(measurement["support"]["normal"], float)},
        "long_axis": rotation @ np.asarray(measurement["long_axis"], float),
        "closing": rotation @ np.asarray(measurement["closing"], float),
        "footprint_width": measurement["footprint_width"],
        "bbox": measurement["bbox"],
        "request_id": measurement.get("request_id", ""),
    }


# ---------------------------------------------------------------------------
# 搜索：两个观察位 + 每锚点周围扫
# ---------------------------------------------------------------------------

def search(arm, camera, calib, config, anchors, args, out_dir):
    """观察 + 扫描搜索。每一次"看"都落盘：彩色/深度/标注/元数据，最后拼一张总览图。"""
    started = time.monotonic()
    deadline = started + args.budget_s
    merged = observe_scan.merged_config(config.get("observe_scan") or {})
    attempts = []
    tried_poses = []
    looks_dir = os.path.join(out_dir, "looks")
    os.makedirs(looks_dir, exist_ok=True)
    looks = []

    def capture():
        for _ in range(6):
            camera.wait_for_frames(timeout_ms=5000)
        pair = rs.align(rs.stream.color).process(camera.wait_for_frames(timeout_ms=5000))
        color = np.asanyarray(pair.get_color_frame().get_data())
        depth = np.asanyarray(pair.get_depth_frame().get_data())
        return color, depth

    def budget_left():
        return deadline - time.monotonic()

    def _contact_sheet():
        """把目前所有观察/扫描帧拼成一张，回看时不用一张张翻。"""
        if not looks:
            return
        thumb_w, thumb_h, label_h = 480, 270, 26
        cols = min(3, len(looks))
        rows = (len(looks) + cols - 1) // cols
        sheet = np.full((rows * (thumb_h + label_h), cols * thumb_w, 3), 255, np.uint8)
        for i, look in enumerate(looks):
            row, col = divmod(i, cols)
            image = cv2.imread(look["color_path"])
            if image is None:
                continue
            image = cv2.resize(image, (thumb_w, thumb_h))
            y = row * (thumb_h + label_h)
            x = col * thumb_w
            sheet[y + label_h:y + label_h + thumb_h, x:x + thumb_w] = image
            cv2.putText(sheet, look["label"], (x + 6, y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(looks_dir, "contact_sheet.jpg"), sheet)

    def save_look(label, anchor_name, color, depth, camera_pose, joints, measured, why):
        """一次观察/扫描的全部记录：原图、深度、标注图、元数据（追加进 index.json）。"""
        index = len(looks) + 1
        stem = "%02d_%s" % (index, label)
        color_path = os.path.join(looks_dir, stem + "_color.jpg")
        depth_path = os.path.join(looks_dir, stem + "_depth.png")
        cv2.imwrite(color_path, color)
        cv2.imwrite(depth_path, depth)                 # uint16 原始毫米深度
        annotated = color.copy()
        bbox = None
        if measured is not None:
            bbox = [float(v) for v in (measured.get("bbox") or [])][:5]
            if len(bbox) >= 4:
                x0, y0, x1, y1 = [int(round(v)) for v in bbox[:4]]
                cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 3)
                angle = bbox[4] if len(bbox) > 4 else 0.0
                cv2.putText(annotated, "target angle=%.0f" % angle,
                            (x0, max(26, y0 - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(annotated, "NOT FOUND", (24, 48), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (0, 0, 255), 3, cv2.LINE_AA)
        annotated_path = os.path.join(looks_dir, stem + "_annotated.jpg")
        cv2.imwrite(annotated_path, annotated)
        meta = {
            "index": index, "label": label, "anchor": anchor_name,
            "joints_deg": [round(float(v), 2) for v in joints],
            "camera_pose": np.round(np.asarray(camera_pose, float), 5).tolist(),
            "found": measured is not None,
            "bbox": None if bbox is None else [round(v, 1) for v in bbox],
            "why": None if measured is not None else why,
            "color_path": color_path, "depth_path": depth_path,
            "annotated_path": annotated_path,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(os.path.join(looks_dir, stem + "_meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2, default=_json_default)
        looks.append(meta)
        with open(os.path.join(looks_dir, "index.json"), "w", encoding="utf-8") as f:
            json.dump(looks, f, ensure_ascii=False, indent=2, default=_json_default)
        _contact_sheet()
        print("    记录：%s（%s）" % (os.path.basename(annotated_path),
                                     "找到" if measured is not None else "未找到"), flush=True)
        return meta

    def try_once(label, anchor_name):
        """在当前位置拍一帧、量一次；这一帧（观察或扫描）全部落盘。"""
        color, depth = capture()
        joints = arm.joints()
        camera_pose = np.asarray(calib.camera(joints), float).reshape(4, 4)
        try:
            measured = measure(color, depth, np.asarray(calib.camera_matrix, float),
                               np.asarray(calib.dist_coeffs, float).reshape(-1), args)
        except Exception as error:                 # noqa: BLE001
            why = "没量出来：%s" % error
            meta = save_look(label, anchor_name, color, depth, camera_pose, joints, None, why)
            return None, why, depth, camera_pose, meta
        base = to_base(measured, camera_pose)
        meta = save_look(label, anchor_name, color, depth, camera_pose, joints, measured, "ok")
        base["look"] = meta
        return base, "ok", depth, camera_pose, meta

    for anchor_index, (anchor_name, anchor_joints) in enumerate(anchors):
        if budget_left() <= 0:
            attempts.append({"anchor": anchor_name, "skipped": "预算用尽"})
            break
        print("\n[观察位 %d/%d] %s  %s" % (anchor_index + 1, len(anchors), anchor_name,
                                          jstr(anchor_joints)), flush=True)
        arm.move_joints([float(v) for v in anchor_joints], speed=args.speed,
                        timeout_s=90, tolerance_deg=1.5)
        time.sleep(0.8)
        tried_poses.append([float(v) for v in anchor_joints])
        label = "observe_%d" % (anchor_index + 1)
        look, why, depth, camera_pose, meta = try_once(label, anchor_name)
        attempts.append({"anchor": anchor_name, "view": "anchor", "look": meta["index"],
                         "ok": bool(look), "why": None if look else why,
                         "image": meta["annotated_path"]})
        print("  锚点自己看一眼：%s" % ("找到了！" if look else why), flush=True)
        if look:
            return look, attempts, time.monotonic() - started, looks

        # 它自己那一帧的支撑面（地面/桌面），用来规划围它的扫描位
        surface_z = observe_scan.estimate_surface_z(depth, camera_pose,
                                                    np.asarray(calib.camera_matrix, float))
        if surface_z is None:
            attempts[-1]["surface"] = None
            print("  这一帧估不出支撑面高度，跳过围它扫", flush=True)
            continue
        attempts[-1]["surface"] = round(float(surface_z), 4)
        print("  这一帧估的支撑面高度：%.4f m" % surface_z, flush=True)
        try:
            plan = observe_scan.plan_views(calib, np.array(anchor_joints, float),
                                           float(surface_z), merged)
        except Exception as error:                 # noqa: BLE001
            attempts[-1]["plan_error"] = str(error)
            print("  规划观察位出错：%s（跳过这个锚点）" % error, flush=True)
            continue
        views = plan.get("views") or []
        print("  围它规划出 %d 个扫描位" % len(views), flush=True)
        for view_index, view in enumerate(views):
            if budget_left() <= 0:
                attempts.append({"anchor": anchor_name, "view": view_index,
                                 "skipped": "预算用尽"})
                break
            target = [float(v) for v in view["joints_deg"]]
            if any(max(abs(a - b) for a, b in zip(target, pose)) < 1.0 for pose in tried_poses):
                continue                       # 已经看过的位姿，不重复走
            tried_poses.append(target)
            print("  [%s #%d] 走 %s" % (anchor_name, view_index + 1, jstr(target)), flush=True)
            arm.move_joints(target, speed=args.speed, timeout_s=90, tolerance_deg=1.5)
            time.sleep(0.8)
            label = "scan_%d_%d" % (anchor_index + 1, view_index + 1)
            look, why, _, _, meta = try_once(label, anchor_name)
            attempts.append({"anchor": anchor_name, "view": view_index + 1,
                             "look": meta["index"], "ok": bool(look),
                             "why": None if look else why,
                             "image": meta["annotated_path"]})
            print("    %s" % ("找到了！" if look else why), flush=True)
            if look:
                return look, attempts, time.monotonic() - started, looks
    return None, attempts, time.monotonic() - started, looks


# ---------------------------------------------------------------------------
# 抓取
# ---------------------------------------------------------------------------

def do_grasp(arm, calib, measurement, args):
    servo = lambda joints: (arm.move_joints([float(v) for v in joints], speed=args.speed,
                                            timeout_s=90, tolerance_deg=1.5), arm.joints())[1]
    top = np.asarray(measurement["top_point"], dtype=float)
    normal = np.asarray(measurement["plane"]["normal"], dtype=float)
    normal /= np.linalg.norm(normal)
    print("\n[抓取] 顶面（基座系）：%s  法线：%s" % (np.round(top, 3).tolist(),
                                                np.round(normal, 3).tolist()))
    plan = grasp.plan(top, measurement["plane"], measurement["long_axis"],
                      measurement["closing"], measurement["footprint_width"], calib.gripper)
    if not plan["ok"]:
        print("  计划没通过：%s" % "；".join(plan["problems"]))
        return {"ok": False, "stage": "plan", "problems": plan["problems"], "plan": plan}
    print("  物高 %.1f mm，下爪 %.1f mm，悬停爪尖 %s" % (
        plan["object_height_m"] * 1000, (plan["descent_m"] or 0) * 1000,
        np.round(plan["hover_tip"], 3).tolist()))

    reach = motion.plan_reach(calib, plan, tilt_limit_deg=args.tilt_limit, n_starts=8,
                             lateral_tol_m=args.lateral_tol_mm / 1000.0,
                             along_tol_m=args.along_tol_mm / 1000.0)
    if args.tilt_limit > grasp.MAX_OBLIQUITY_DEG or args.lateral_tol_mm > grasp.LATERAL_TOL_M * 1000:
        print("  （已放宽：倾斜上限 %.0f°，横向容差 %.0f mm）"
              % (args.tilt_limit, args.lateral_tol_mm))
    print("  可达解：%s（倾斜 %.1f°，闭合方向偏 %.0f°）" % (
        "有" if reach["ok"] else "没有：" + reach.get("why", ""),
        reach.get("tilt_deg", 0), reach.get("closing_offset_deg", 0)))
    if not reach["ok"]:
        need = needs_base_move(top, plan, reach, args.tilt_limit)
        print("  够不着 → 建议底盘/姿态调整：%s" % need["advice"])
        return {"ok": False, "stage": "reach", "problems": [reach.get("why", "")],
                "needs_base_move": need, "plan": plan}
    rotation = np.asarray(reach["solved"]["rotation"], dtype=float)

    settled = grasp.descend_settled_m(calib.gripper, plan.get("footprint_width_m"))

    if args.direct:
        # 不悬停、直接下探（斜抓时悬停姿态可能根本站不住；这是"先看看效果"的试验模式）
        print("  试验模式：跳过悬停，直接下探到抓取点（请盯急停）", flush=True)
        down = motion.move_tip(calib, servo, plan["grasp_tip"], rotation,
                               rounds=2, n_starts=8, settled_m=settled)
        print("  直接下探：%s，误差 %.1f mm（判据 %.1f mm）" % (
            "到位" if down["ok"] else "没到位", (down["error_m"] or 0) * 1000, settled * 1000))
        if (down["error_m"] or 0.0) > DESCEND_ABORT_M:
            print("  ！误差过大，不合爪（避免夹到旁边/压到椅面）")
            return {"ok": False, "stage": "direct-descend", "descend": _small(down), "plan": plan}
        hover_joints = list(measurement.get("look", {}).get("joints_deg")
                            or arm.joints())          # 抬起就原路退回找到它的那个观察位
        lift_note = "原路退回观察位"
    else:
        # 1) 悬停（物体上方 10 cm）
        hover = motion.move_tip(calib, servo, plan["hover_tip"], rotation,
                                rounds=3, n_starts=8, tolerance_m=HOVER_SETTLE_M)
        print("  悬停：%s，误差 %.1f mm" % ("到位" if hover["ok"] else "没到位",
                                          (hover["error_m"] or 0) * 1000))
        if not hover["ok"]:
            return {"ok": False, "stage": "hover", "hover": _small(hover), "plan": plan}
        hover_joints = list(hover["joints_deg"])
        # 2) 下爪
        down = motion.move_tip(calib, servo, plan["grasp_tip"], rotation,
                               rounds=2, n_starts=8, settled_m=settled)
        print("  下爪：%s，误差 %.1f mm（判据 %.1f mm）" % (
            "到位" if down["ok"] else "没到位", (down["error_m"] or 0) * 1000, settled * 1000))
        if (down["error_m"] or 0.0) > DESCEND_ABORT_M:
            return {"ok": False, "stage": "descend", "descend": _small(down), "plan": plan}
        lift_note = "回悬停位"

    # 3) 合爪：分两步，让指头慢慢贴上去；读实际开口判断有没有夹到东西
    opening_before = arm.gripper_open_0to1()
    arm.set_gripper(0.02, effort=3000)
    time.sleep(1.0)
    arm.close_gripper(effort=3000)
    time.sleep(1.5)
    opening_after = arm.gripper_open_0to1()
    print("  合爪：开度 %.3f → %.3f" % (opening_before, opening_after))
    if opening_after < 0.02:
        print("  ！开口合到底了，可能没夹到东西（胡萝卜比这粗）")

    # 4) 抬起 = 回到悬停关节角（direct 模式：原路退回观察位）
    arm.move_joints(hover_joints, speed=args.speed, timeout_s=90, tolerance_deg=1.5)
    time.sleep(0.5)
    opening_lift = arm.gripper_open_0to1()
    state = arm.joints()
    held = 0.02 < opening_lift < 0.9
    print("  抬起：开度 %.3f，关节 %s → %s" % (
        opening_lift, jstr(hover_joints), jstr(state)))
    return {"ok": bool(held), "stage": "lift", "opening_before": opening_before,
            "opening_after_close": opening_after, "opening_after_lift": opening_lift,
            "held_guess": bool(held), "hover_joints_deg": [round(v, 2) for v in hover_joints],
            "tip_m": np.round(plan["hover_tip"], 4).tolist()}


def needs_base_move(top_point_base, plan, reach, tilt_limit_deg):
    """够不着时给 pilot 的结构化建议：物体在哪、底座往哪挪、要不要换姿态。

    这次真机实测：桌面相对臂基座 +0.29 m、物体在基座前 0.566 m 时，竖直抓取 IK 差
    70–110 mm（只有 40–60° 斜着进才有解）；把同样的点挪到基座高度附近（桌面相对 z≈0），
    x=0.45 m 就能 0 误差竖直抓。所以"桌子上抓不到"通常不是算法问题，是**底座高度/距离**。
    """
    top = np.asarray(top_point_base, dtype=float)
    radius = float(np.hypot(top[0], top[1]))
    forward = max(0.0, radius - 0.45)          # 让物体落到基座前 ~0.45 m
    reasons = []
    if top[2] > 0.15:
        reasons.append("目标高于臂基座 %.2f m（桌面太高）：建议先站起来抬升基座，"
                       "或换低一点的桌面" % top[2])
    if forward > 0.02:
        reasons.append("目标在基座前 %.2f m（太远）：建议朝目标方向前进约 %.2f m"
                       % (radius, forward))
    if not reasons:
        reasons.append("物体在可达边缘，但找不到满足倾斜上限 %.0f° 的姿态："
                       "可尝试放低/挪近，或临时放宽 --tilt-limit" % tilt_limit_deg)
    return {"object_base_m": np.round(top, 3).tolist(),
            "radius_m": round(radius, 3),
            "height_above_base_m": round(float(top[2]), 3),
            "forward_m": round(forward, 3),
            "tilt_deg_tried": reach.get("tilt_deg"),
            "advice": "；".join(reasons)}


def _small(move_result):
    return {k: move_result.get(k) for k in ("ok", "rounds", "error_m")}


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (list, tuple)):
        return list(value)
    raise TypeError("not JSON serializable: %s" % type(value).__name__)


# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="最小验证：两个观察位 + 扫描 → 找并抓胡萝卜。不经过 rbnx。")
    parser.add_argument("--target", default="胡萝卜")
    parser.add_argument("--detector", choices=("color", "vlm"), default="color")
    parser.add_argument("--budget-s", type=float, default=60.0, help="搜索时间预算（秒）")
    parser.add_argument("--max-views", type=int, default=4, help="每个锚点最多扫几个位姿")
    parser.add_argument("--speed", type=int, default=20, help="臂速度百分比")
    parser.add_argument("--tilt-limit", type=float, default=grasp.MAX_OBLIQUITY_DEG,
                        help="工具轴允许偏离支撑面法线的最大角度（度）；默认 12（与抓取流程一致）")
    parser.add_argument("--lateral-tol-mm", type=float, default=grasp.LATERAL_TOL_M * 1000,
                        help="允许的横向偏差（毫米）；默认 5，试斜着抓时可放宽到 10~12")
    parser.add_argument("--along-tol-mm", type=float, default=grasp.ALONG_TOL_M * 1000,
                        help="允许的沿接近方向偏差（毫米）；默认 10")
    parser.add_argument("--direct", action="store_true",
                        help="跳过悬停、直接下探到抓取点（斜抓悬停不可达时用；有蹭面风险，需人盯急停）")
    parser.add_argument("--search-only", action="store_true", help="只找不抓")
    parser.add_argument("--park", action="store_true", help="结束回零位并失能")
    parser.add_argument("--vlm-timeout", type=float, default=150.0)
    parser.add_argument("--out", default="", help="结果图片/JSON 目录")
    args = parser.parse_args()

    out_dir = args.out or ("/tmp/minimal_grasp_%s" % time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print("输出目录：%s" % out_dir)
    print("检测器：%s；目标：%s；搜索预算：%.0f s；最多 %d 个扫描位/锚点"
          % (args.detector, args.target, args.budget_s, args.max_views))

    config = load_config()
    calib = Calibration.from_numbers(dict(config["calib"]))
    observe_cfg = dict(config.get("observe_scan") or {})
    observe_cfg["max_views"] = args.max_views
    nominal = [float(v) for v in config["calib"]["observe_joints_deg"]]
    anchors = [("标称观察位", nominal)]
    for index, joints in enumerate(observe_cfg.get("anchors_deg") or [], start=1):
        anchors.append(("第二个观察位 %d" % index, [float(v) for v in joints]))
    print("观察位共 %d 个：%s" % (len(anchors), [name for name, _ in anchors]))

    # 相机
    camera = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    rs_cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    camera.start(rs_cfg)

    arm = DirectArm()
    time.sleep(0.4)
    st = arm.status()
    print("臂状态：arm=%s ctrl=%s teach=%s" % (
        ARM_STATUS_TEXT.get(st.arm_status, hex(st.arm_status)),
        CTRL_MODE_TEXT.get(st.ctrl_mode, hex(st.ctrl_mode)),
        TEACH_STATUS_TEXT.get(st.teach_status, hex(st.teach_status))))
    if st.arm_status != 0x00 or st.teach_status == 0x01:
        print("！！当前状态不适合自动动臂（示教/异常），先退出示教再试。")
        return 2
    print("使能 + 切 CAN 控制 …")
    if not arm.enable_and_take_control(timeout_s=20, speed=args.speed):
        print("！！使能/切控制失败")
        return 3
    arm.set_gripper(1.0, effort=5000)          # 先张开，别让手指挡镜头
    time.sleep(0.8)

    result = {"target": args.target, "detector": args.detector,
              "anchors": [{"name": n, "joints_deg": [round(v, 2) for v in j]}
                          for n, j in anchors]}
    search_start = time.monotonic()
    measurement = None
    attempts = []
    looks = []
    try:
        measurement, attempts, search_time, looks = search(arm, camera, calib, config,
                                                           anchors, args, out_dir)
    finally:
        pass
    result["search"] = {"elapsed_s": round(time.monotonic() - search_start, 2),
                        "attempts": attempts, "found": bool(measurement),
                        "look_count": len(looks),
                        "looks_dir": os.path.join(out_dir, "looks"),
                        "looks": looks}
    if measurement is not None:
        result["found"] = {
            "top_point_base_m": np.round(measurement["top_point"], 4).tolist(),
            "plane_normal_base": np.round(
                np.asarray(measurement["plane"]["normal"]) /
                np.linalg.norm(measurement["plane"]["normal"]), 4).tolist(),
            "footprint_width_m": measurement["footprint_width"],
            "request_id": measurement.get("request_id", ""),
        }
        if measurement.get("look"):
            src = measurement["look"]["annotated_path"]
            dst = os.path.join(out_dir, "found_annotated.jpg")
            try:
                import shutil
                shutil.copy(src, dst)
            except Exception:                      # noqa: BLE001
                pass
        print("\n== 找到目标（搜索 %.1f s，记录 %d 帧）=="
              % (result["search"]["elapsed_s"], result["search"]["look_count"]))
        print("  顶面（基座系）%s，闭合方向宽 %s" % (
            np.round(measurement["top_point"], 3).tolist(),
            None if measurement["footprint_width"] is None
            else "%.1f mm" % (measurement["footprint_width"] * 1000)))
        if not args.search_only:
            result["grasp"] = do_grasp(arm, calib, measurement, args)
    else:
        print("\n== 没找到（搜索 %.1f s）==" % result["search"]["elapsed_s"])
        result["grasp"] = None

    # 收尾
    if args.park:
        print("\n回零并失能 …")
        arm.home_and_disable(timeout_s=60)
    camera.stop()
    result_path = os.path.join(out_dir, "result.json")
    json.dump(result, open(result_path, "w"), ensure_ascii=False, indent=2,
              default=_json_default)
    print("\n结果写入 %s" % result_path)
    if measurement is None:
        return 1
    grasp_result = result.get("grasp")
    return 0 if (grasp_result and grasp_result.get("ok")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
