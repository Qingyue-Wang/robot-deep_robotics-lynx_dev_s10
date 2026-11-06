#!/usr/bin/env python3
# Description: odom 安全闸 —— odom 不健康时把底盘停住并取消导航目标。
#
# 问题：FAST-LIO 崩塌时，`lynx_fastlio` 会把坏的 /fastlio/odom 和 TF 抑制掉，
# 于是 /odom 直接停更。但 nav2 不会因为 odom 停更自己停：它会拿最后一帧位姿继续
# 算控制量，机器人就朝着"记忆里的方向"一直走。
#
# 这个节点坐在导航服务的最终速度和底盘之间：
#
#     nav2 velocity guard --/cmd_vel_service--> [odom_guard] --/cmd_vel--> lynx_quadruped
#
# 为此清单里 nav2 服务的 velocity_output_topic 要设成 /cmd_vel_service，本节点是
# /cmd_vel 的唯一发布者：
#   - /odom 健康 → 原样转发导航速度；
#   - /odom 停更超过 stale_timeout_s、出现 NaN/inf、单步跳变或速度不可能 → 持续发零速，
#     并向 /navigate_to_pose/_action/cancel_goal 发"取消全部目标"（零 UUID 即 cancel all）；
#   - odom 连续 recover_s 秒正常后放行；导航目标不会自动恢复，需要重新下发。
#
# 不覆盖 pilot 直接调 chassis/move（那条路走 /NAV_CMD，不经过 /cmd_vel）。
#
# 用法（start.sh 会自动拉起；也可手动跑）：
#   source /opt/ros/humble/setup.bash
#   python3 scripts/odom_guard.py --manifest robonix_manifest.yaml \
#       --config config/odom_guard.yaml
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import yaml


def _load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _apply_ros_env(manifest_path: str) -> None:
    """Use the same ROS domain/RMW as the stack (manifest `env:` block).

    Must run before rclpy.init(); the domain id is read when the middleware
    initialises, not at import time.
    """
    try:
        env = (_load_yaml(manifest_path).get("env") or {})
    except OSError as error:
        print(f"[odom_guard] cannot read manifest {manifest_path}: {error}",
              file=sys.stderr)
        return
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
        value = env.get(key)
        if value is not None:
            os.environ[key] = str(value)


class OdomMonitor:
    """Pure odometry health state machine (no ROS imports, unit-testable)."""

    def __init__(self, *, stale_timeout_s: float, max_jump_m: float,
                 max_speed_mps: float, max_yaw_jump_rad: float,
                 recover_s: float):
        self.stale_timeout_s = float(stale_timeout_s)
        self.max_jump_m = float(max_jump_m)
        self.max_speed_mps = float(max_speed_mps)
        self.max_yaw_jump_rad = float(max_yaw_jump_rad)
        self.recover_s = float(recover_s)
        self._last_t: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._fault_reason: str | None = None
        self._fault_at = 0.0

    def observe(self, x: float, y: float, yaw: float, now: float) -> str | None:
        """Feed one odom sample. Returns a fault reason when this sample is bad."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return self._fault(f"non-finite odom pose ({x!r}, {y!r}, {yaw!r})", now)

        reason = None
        if self._last_t is not None and self._last_pose is not None:
            gap = now - self._last_t
            lx, ly, lyaw = self._last_pose
            if gap > self.stale_timeout_s:
                # The stream was dead for a while; it already tripped the stale
                # check. Adopt this sample as the new baseline instead of
                # comparing against a pose from before the gap.
                pass
            else:
                dt = max(gap, 1e-3)
                dist = math.hypot(x - lx, y - ly)
                dyaw = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)))
                if dist > self.max_jump_m:
                    reason = (f"odom jumped {dist:.2f} m in one sample "
                              f"(limit {self.max_jump_m:.2f} m)")
                elif dist / dt > self.max_speed_mps:
                    reason = (f"implied odom speed {dist / dt:.1f} m/s "
                              f"(limit {self.max_speed_mps:.1f} m/s)")
                elif dyaw > self.max_yaw_jump_rad:
                    reason = (f"odom yaw jumped {dyaw:.2f} rad in one sample "
                              f"(limit {self.max_yaw_jump_rad:.2f} rad)")

        if reason is not None:
            return self._fault(reason, now)

        self._last_t = now
        self._last_pose = (x, y, yaw)
        if self._fault_reason is not None and now - self._fault_at >= self.recover_s:
            self._fault_reason = None
        return None

    def _fault(self, reason: str, now: float) -> str:
        self._fault_reason = reason
        self._fault_at = now
        return reason

    def stale_reason(self, now: float) -> str | None:
        if self._last_t is None:
            return "no /odom received yet"
        age = now - self._last_t
        if age > self.stale_timeout_s:
            return f"no /odom for {age:.2f}s (limit {self.stale_timeout_s:.2f}s)"
        return None

    def fault_reason(self, now: float) -> str | None:
        """Latched fault, cleared after recover_s of sane samples."""
        if self._fault_reason is None:
            return None
        if now - self._fault_at >= self.recover_s:
            # observe() clears it too, but the stream may have stopped right
            # after the fault; don't keep a fault that is only old, not fresh.
            self._fault_reason = None
            return None
        return self._fault_reason

    def blocking_reason(self, now: float) -> str | None:
        return self.stale_reason(now) or self.fault_reason(now)


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    parser = argparse.ArgumentParser(description="Stop the chassis when odometry is unhealthy.")
    parser.add_argument("--manifest", default="robonix_manifest.yaml",
                        help="stack manifest; its env: ROS_DOMAIN_ID/RMW are applied")
    parser.add_argument("--config", default="config/odom_guard.yaml")
    args = parser.parse_args()

    _apply_ros_env(args.manifest)
    cfg = _load_yaml(args.config)

    # Imported after the ROS env is applied.
    import rclpy  # noqa: E402
    from action_msgs.srv import CancelGoal  # noqa: E402
    from geometry_msgs.msg import Twist  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402
    from rclpy.node import Node  # noqa: E402

    odom_topic = str(cfg.get("odom_topic", "/odom"))
    input_topic = str(cfg.get("input_topic", "/cmd_vel_service"))
    output_topic = str(cfg.get("output_topic", "/cmd_vel"))
    cancel_service = str(cfg.get("cancel_service", "/navigate_to_pose/_action/cancel_goal"))
    tick_hz = float(cfg.get("tick_hz", 20.0))

    class OdomGuard(Node):
        def __init__(self) -> None:
            super().__init__("odom_guard")
            self.monitor = OdomMonitor(
                stale_timeout_s=float(cfg.get("stale_timeout_s", 0.5)),
                max_jump_m=float(cfg.get("max_jump_m", 0.5)),
                max_speed_mps=float(cfg.get("max_speed_mps", 3.0)),
                max_yaw_jump_rad=float(cfg.get("max_yaw_jump_rad", 1.5)),
                recover_s=float(cfg.get("recover_s", 2.0)),
            )
            self._pub = self.create_publisher(Twist, output_topic, 10)
            self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
            self.create_subscription(Twist, input_topic, self._on_cmd, 20)
            self._cancel = self.create_client(CancelGoal, cancel_service)
            self._cancel_sent = False
            self._last_reason = ""
            self._odom_log_at = 0.0
            self.create_timer(1.0 / max(tick_hz, 1.0), self._tick)
            self.get_logger().info(
                f"odom guard active: {odom_topic} -> gate -> {output_topic} "
                f"(input {input_topic}); cancel {cancel_service}")

        def _on_odom(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            now = time.monotonic()
            reason = self.monitor.observe(
                float(pose.position.x), float(pose.position.y),
                _yaw_of(pose.orientation), now)
            if reason and (reason != self._last_reason
                           or now - self._odom_log_at >= 1.0):
                # A diverged FAST-LIO can emit bad samples at the full rate;
                # log at most once a second per distinct reason.
                self._last_reason = reason
                self._odom_log_at = now
                self.get_logger().error(f"ODOM UNHEALTHY: {reason}")

        def _on_cmd(self, msg: Twist) -> None:
            if self.monitor.blocking_reason(time.monotonic()) is None:
                self._pub.publish(msg)
            # While blocked the zero pump owns the topic; dropping the command
            # keeps a stale nav2 command from sneaking through between ticks.

        def _tick(self) -> None:
            reason = self.monitor.blocking_reason(time.monotonic())
            if reason is not None:
                self._pub.publish(Twist())          # keep the base stopped
                if not self._cancel_sent:
                    self._cancel_sent = True
                    self.get_logger().error(
                        f"stopping base: {reason}; cancelling navigation goal")
                    if self._cancel.service_is_ready():
                        # Zeroed goal_info = "cancel all goals" (CancelGoal.srv).
                        self._cancel.call_async(CancelGoal.Request())
                    else:
                        self.get_logger().warning(
                            f"{cancel_service} not available; base still zeroed")
            elif self._cancel_sent:
                self._cancel_sent = False
                self.get_logger().warning(
                    "odom healthy again; goal was cancelled — re-issue navigation")

    rclpy.init()
    node = OdomGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:                          # noqa: BLE001 — log it, don't die silently
        import traceback
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
