#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Switch DWB between forward-only and reverse-only from the plan direction.

The rule is the one requested for the Lynx S10: look at the angle between the
robot heading and the planned path a short lookahead away. Within 90 degrees
the robot drives forward; beyond 90 degrees it backs up along the path instead
of turning around. DWB samples both directions, but its velocity range is
narrowed here so only the intended sign is considered:

    forward: FollowPath.min_vel_x =  0.00, max_vel_x = 0.35
    reverse: FollowPath.min_vel_x = -0.15, max_vel_x = 0.00

The 90-degree boundary uses hysteresis (reverse at 100 degrees, back to
forward at 80 degrees) so the wheeled quadruped does not chatter between the
two modes. DWB's RotateToGoal critic still performs the final in-place
rotation to the goal yaw.

The velocity parameters are dynamic (dwb_plugins::KinematicsHandler), so this
node only calls /controller_server/set_parameters; no controller/critic
plugin is needed. Mapping and the front-only /scanner/scan are untouched.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def _apply_ros_env(manifest_path: str) -> None:
    """Use the same ROS domain/RMW as the stack (manifest ``env:`` block)."""
    try:
        with open(manifest_path, encoding="utf-8") as handle:
            env = (yaml.safe_load(handle) or {}).get("env") or {}
    except OSError as error:
        print(f"[drive_direction] cannot read manifest {manifest_path}: {error}",
              file=sys.stderr)
        return
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
        value = env.get(key)
        if value is not None:
            os.environ[key] = str(value)


def _load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _yaw_of(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.w * quaternion.z +
                             quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y +
                                   quaternion.z * quaternion.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DirectionGovernor(Node):
    """Read /plan + TF and keep the DWB velocity range in the wanted direction."""

    def __init__(self, cfg: dict) -> None:
        super().__init__("drive_direction")
        self.plan_topic = str(cfg.get("plan_topic", "/plan"))
        self.base_frame = str(cfg.get("base_frame", "base_link"))
        self.controller_node = str(
            cfg.get("controller_node", "/controller_server")).rstrip("/")
        self.plugin = str(cfg.get("controller_plugin", "FollowPath"))
        self.lookahead_m = float(cfg.get("lookahead_m", 0.5))
        self.forward_enter_deg = float(cfg.get("forward_enter_deg", 80.0))
        self.reverse_enter_deg = float(cfg.get("reverse_enter_deg", 100.0))
        self.min_target_m = float(cfg.get("min_target_m", 0.15))
        self.rate_hz = float(cfg.get("rate_hz", 10.0))
        self.plan_timeout_s = float(cfg.get("plan_timeout_s", 2.0))
        self.ranges = {
            "forward": (float(cfg.get("forward_min_vel_x", 0.0)),
                        float(cfg.get("forward_max_vel_x", 0.35))),
            "reverse": (float(cfg.get("reverse_min_vel_x", -0.15)),
                        float(cfg.get("reverse_max_vel_x", 0.0))),
        }
        # RotationShimController threshold: forward keeps the normal
        # pre-rotation; reverse disables it with a value above pi so the
        # primary DWB controller receives the path and backs up.
        self.thresholds = {
            "forward": float(cfg.get("forward_angular_dist_threshold", 0.30)),
            "reverse": float(cfg.get("reverse_angular_dist_threshold", 3.15)),
        }
        self.mode_hold_s = float(cfg.get("mode_hold_s", 3.0))
        # Inside this radius to the plan's last pose the shim is forced off so
        # DWB's RotateToGoal is the only thing rotating the robot to goal yaw.
        self.near_goal_m = float(cfg.get("near_goal_m", 0.35))
        self.goal_status_topic = str(
            cfg.get("goal_status_topic", "/navigate_to_pose/_action/status"))

        self._plan: Path | None = None
        self._plan_at = 0.0
        self._plan_pending = False
        self._goal_active = False
        self._state = "forward"       # direction currently applied to DWB
        self._applied: str | None = None
        self._applied_shim: float | None = None
        self._last_mode_change = 0.0
        self._pending = None
        self._warn_at: dict[str, float] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Path, self.plan_topic, self._on_plan, 10)
        self.create_subscription(
            GoalStatusArray, self.goal_status_topic, self._on_goal_status, 10)
        self._client = self.create_client(
            SetParameters, f"{self.controller_node}/set_parameters")
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)
        self.get_logger().info(
            "drive_direction ready: %s -> %s "
            "(lookahead=%.2fm, reverse>=%.0fdeg, forward<=%.0fdeg, "
            "mode_hold=%.1fs, near_goal=%.2fm) "
            "forward=%s reverse=%s shim_thresholds=%s"
            % (self.plan_topic, self.controller_node, self.lookahead_m,
               self.reverse_enter_deg, self.forward_enter_deg,
               self.mode_hold_s, self.near_goal_m,
               self.ranges["forward"], self.ranges["reverse"],
               self.thresholds))

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warn_at.get(key, 0.0) >= 5.0:
            self._warn_at[key] = now
            self.get_logger().warning(message)

    def _on_plan(self, msg: Path) -> None:
        if msg.poses:
            self._plan = msg
            self._plan_at = time.monotonic()
            # Evaluate the direction once per path, not every 10 Hz tick.
            self._plan_pending = True

    def _on_goal_status(self, msg: GoalStatusArray) -> None:
        # STATUS_ACCEPTED=1, STATUS_EXECUTING=2. While a navigation goal is
        # active, a stale /plan must NOT fail-safe back to forward: at the goal
        # the plan stops updating, and re-enabling forward + shim would fight
        # RotateToGoal and make the robot spin on the spot.
        active = any(status.status in (1, 2) for status in msg.status_list)
        if active != self._goal_active:
            self._goal_active = active
            self.get_logger().info(f"navigation goal active: {active}")

    def _near_goal(self, robot: tuple[float, float, float] | None) -> bool:
        if robot is None or self._plan is None or not self._plan.poses:
            return False
        last = self._plan.poses[-1].pose.position
        return math.hypot(last.x - robot[0], last.y - robot[1]) <= self.near_goal_m

    def _robot_pose(self, frame: str) -> tuple[float, float, float] | None:
        if not frame:
            self._warn("no-frame", "plan has an empty header.frame_id")
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                frame, self.base_frame, Time())
        except Exception as error:  # tf2 raises several exception types
            self._warn(f"tf:{frame}",
                       f"cannot look up {frame} -> {self.base_frame}: {error}")
            return None
        translation = transform.transform.translation
        yaw = _yaw_of(transform.transform.rotation)
        return (translation.x, translation.y, yaw)

    def _path_angle(self, robot: tuple[float, float, float]) -> float | None:
        """Angle from the robot heading to the path, sampled AHEAD of the robot.

        Project the robot onto the polyline first, then walk ``lookahead_m``
        along the path from that projection. The previous version started at
        the plan's first pose and took the first pose farther than the
        lookahead, so once the robot moved past the start of an older plan
        (the BT reuses a path for up to 10 s) that point sat BEHIND the robot,
        reported ~180 deg, and flipped this node forward/reverse every second
        (2026-11-20 14:42 run: angles 17-176 deg, reverse in 35% of cycles).
        """
        if self._plan is None or not self._plan.poses:
            return None
        rx, ry, ryaw = robot
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self._plan.poses
        ]
        if len(points) == 1:
            target = points[0]
        else:
            best_d2 = math.inf
            best_i = 0
            best_t = 0.0
            for i in range(len(points) - 1):
                ax, ay = points[i]
                bx, by = points[i + 1]
                vx, vy = bx - ax, by - ay
                seg2 = vx * vx + vy * vy
                if seg2 <= 1e-12:
                    t = 0.0
                else:
                    t = ((rx - ax) * vx + (ry - ay) * vy) / seg2
                t = max(0.0, min(1.0, t))
                px, py = ax + t * vx, ay + t * vy
                d2 = (px - rx) ** 2 + (py - ry) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
                    best_t = t
            ax, ay = points[best_i]
            bx, by = points[best_i + 1]
            cx = ax + best_t * (bx - ax)
            cy = ay + best_t * (by - ay)
            remaining = self.lookahead_m
            target = None
            i = best_i
            while i < len(points) - 1:
                nx, ny = points[i + 1]
                step = math.hypot(nx - cx, ny - cy)
                if step >= remaining:
                    ratio = 0.0 if step <= 1e-9 else remaining / step
                    target = (cx + (nx - cx) * ratio, cy + (ny - cy) * ratio)
                    break
                remaining -= step
                cx, cy = nx, ny
                i += 1
            if target is None:
                target = points[-1]
        dx = target[0] - rx
        dy = target[1] - ry
        if math.hypot(dx, dy) < self.min_target_m:
            return None
        return _wrap(math.atan2(dy, dx) - ryaw)

    def _tick(self) -> None:
        if self._pending is not None:
            return
        now = time.monotonic()
        fresh = (self._plan is not None and
                 now - self._plan_at <= self.plan_timeout_s)

        if not fresh:
            if self._goal_active:
                # The plan stops updating at the goal. Hold the direction and
                # force the shim off near the goal so RotateToGoal owns the
                # final yaw.
                robot = None
                if self._plan is not None:
                    robot = self._robot_pose(self._plan.header.frame_id)
                self._apply(
                    self._state, "goal active, stale plan",
                    shim_override=3.15 if self._near_goal(robot) else None)
            else:
                self._apply("forward", "no fresh plan", force=True)
            return

        robot = self._robot_pose(self._plan.header.frame_id)
        if robot is None:
            self._apply("forward", "no robot pose", force=True)
            return
        near = self._near_goal(robot)

        if not self._plan_pending:
            # Between 1 Hz plans only the near-goal shim gate can change.
            self._apply(self._state, "near-goal gate",
                        shim_override=3.15 if near else None)
            return

        self._plan_pending = False
        angle = self._path_angle(robot)
        if angle is None:
            self._apply(self._state, "short path / at goal",
                        shim_override=3.15 if near else None)
            return

        degrees = math.degrees(abs(angle))
        desired = self._state
        if self._state == "forward" and degrees >= self.reverse_enter_deg:
            desired = "reverse"
        elif self._state == "reverse" and degrees <= self.forward_enter_deg:
            desired = "forward"
        if (desired != self._state and
                now - self._last_mode_change < self.mode_hold_s):
            desired = self._state
        self._apply(desired, f"angle={degrees:.1f}deg",
                    shim_override=3.15 if near else None)

    def _apply(self, desired: str, reason: str, force: bool = False,
               shim_override: float | None = None) -> None:
        shim = (self.thresholds[desired] if shim_override is None
                else float(shim_override))
        if (desired == self._state and self._applied == desired and
                self._applied_shim == shim):
            return
        if (not force and desired != self._state and
                time.monotonic() - self._last_mode_change < self.mode_hold_s):
            return
        if not self._client.service_is_ready():
            self._warn("service",
                       f"{self.controller_node}/set_parameters not ready yet")
            return
        values = self.ranges[desired]
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=f"{self.plugin}.min_vel_x",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(values[0]))),
            Parameter(
                name=f"{self.plugin}.max_vel_x",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(values[1]))),
            Parameter(
                name=f"{self.plugin}.angular_dist_threshold",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=shim)),
        ]
        self._pending = self._client.call_async(request)
        self._pending.add_done_callback(
            lambda future: self._on_set(future, desired, reason, shim))

    def _on_set(self, future, desired: str, reason: str, shim: float) -> None:
        self._pending = None
        try:
            response = future.result()
            ok = bool(response) and all(
                result.successful for result in response.results)
        except Exception as error:  # noqa: BLE001 — log, keep retrying
            ok = False
            self.get_logger().warning(f"set_parameters raised: {error}")
        if not ok:
            self._warn("set-failed",
                       f"failed to apply {desired} velocity range; will retry")
            return
        previous = self._state
        self._state = desired
        self._applied = desired
        self._applied_shim = shim
        if desired != previous:
            self._last_mode_change = time.monotonic()
        self.get_logger().info(
            "drive direction %s -> %s (%s) min_vel_x=%.2f max_vel_x=%.2f "
            "shim_angular_dist_threshold=%.2f"
            % (previous, desired, reason, self.ranges[desired][0],
               self.ranges[desired][1], shim))

    def restore_forward(self) -> None:
        """Best-effort fail-safe: leave DWB forward-only on exit.

        SIGTERM may already have invalidated the rclpy context by the time the
        finally block runs; every call here can raise in that state, so the
        whole block is guarded and the failure is silent.
        """
        if not rclpy.ok():
            return
        try:
            if not self._client.service_is_ready():
                return
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name=f"{self.plugin}.min_vel_x",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(self.ranges["forward"][0]))),
                Parameter(
                    name=f"{self.plugin}.max_vel_x",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(self.ranges["forward"][1]))),
                Parameter(
                    name=f"{self.plugin}.angular_dist_threshold",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(self.thresholds["forward"]))),
            ]
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info(
                "drive_direction exit: restored forward range")
        except Exception:  # noqa: BLE001 — shutdown path
            return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Choose DWB forward/reverse range from the plan direction.")
    parser.add_argument(
        "--manifest", default="robonix_manifest.yaml",
        help="stack manifest; its env: ROS_DOMAIN_ID/RMW are applied")
    parser.add_argument(
        "--config", default="config/drive_direction.yaml",
        help="direction governor config")
    args, ros_args = parser.parse_known_args(argv)
    _apply_ros_env(args.manifest)
    cfg = _load_yaml(args.config)
    rclpy.init(args=ros_args or None)
    node = DirectionGovernor(cfg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.restore_forward()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
