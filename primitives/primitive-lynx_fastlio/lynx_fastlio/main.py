#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_fastlio — FAST-LIO odometry primitive for the Lynx S10.

FAST-LIO (RoboSense Airy fork) runs as a ROS 2 node started by ``start.sh``.
It publishes ``nav_msgs/Odometry`` on ``/Odometry`` in a fixed world frame
with the IMU body as child. This primitive converts that stream into the
Robonix chassis-odom contract:

    /Odometry (world -> imu body) --[T_imu_base]--> /fastlio/odom (odom -> base_link)

and broadcasts the matching ``odom -> base_link`` TF. Mapping consumes the TF
as external odometry instead of running RTAB-Map's ``icp_odometry``; Nav2 gets
the same stream through ``lynx_quadruped``'s ``/odom`` forwarding.

Why a correction is needed
--------------------------
FAST-LIO's state is the IMU body pose in its own world frame. That frame is
the initial IMU frame, and the Airy's built-in IMU is tilted with the vertical
lidar mount, so raw FAST-LIO odometry is neither ``base_link`` nor gravity
aligned. With

    T_imu_base = T_imu_lidar * inverse(T_base_link_airy)

the primitive publishes the base pose in the *initial base_link frame*:

    T_odom_base(t) = inverse(T_imu_base) * T_world_imu(t) * T_imu_base

This keeps the odom frame gravity aligned (the robot starts level) and makes
``odom -> base_link`` start at the origin:

  * ``T_imu_lidar`` is ``mapping.extrinsic_R/T`` from the FAST-LIO YAML
    (LiDAR pose in the IMU body frame).
  * ``T_base_link_airy`` is ``base_to_lidar_R/T`` from ``config/s10_mount.yaml``
    — the same transform ``primitive-lynx_lidar`` publishes as the
    ``base_link -> airy_201`` static TF.

Both files are read at init, so the wrapper and the LIO node cannot disagree
about the geometry.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from pathlib import Path

import yaml

from robonix_api import Primitive, Ok, Deferred
from robonix_api.ros import RosBackend

from geometry_msgs.msg import TransformStamped  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from rclpy.clock import Clock  # type: ignore

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_fastlio] %(message)s",
)
log = logging.getLogger("lynx_fastlio")

lynx_fastlio = Primitive(id="lynx_fastlio", namespace="robonix/primitive/chassis")

PKG_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FASTLIO_CONFIG = "robosense_airy_s10.yaml"
DEFAULT_MOUNT_CONFIG = "s10_mount.yaml"

_odom_pub = None
_tf_broadcaster = None
_odom_frame = "odom"
_base_frame = "base_link"
_R_imu_base: tuple[tuple[float, ...], ...] = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
_t_imu_base = (0.0, 0.0, 0.0)
_published = 0
_last_rx_monotonic = 0.0

# Divergence guard. FAST-LIO can lose scan matching ("No Effective Points!" in
# its log), after which the EKF integrates IMU alone and the pose runs away to
# kilometres in seconds. A robot of this size cannot move faster than a few
# m/s or several rad/s, so any sample beyond these limits is corrupt; it must
# never reach Nav2/RTAB-Map as odometry or TF.
_max_speed_mps = 2.0
_max_jump_m = 0.6
_max_yaw_rate_rps = 4.0
_last_good: tuple[float, float, float, float, float] | None = None
_diverged = False


# ── small 3-D math helpers (kept dependency-free) ───────────────────────────


def _vec3(values) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"expected 3 numbers, got {values!r}")
    return (float(values[0]), float(values[1]), float(values[2]))


def _mat3(values) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(values, (list, tuple)) or len(values) != 9:
        raise ValueError(f"expected 9 numbers, got {values!r}")
    return tuple(
        (float(values[r * 3]), float(values[r * 3 + 1]), float(values[r * 3 + 2]))
        for r in range(3)
    )


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _quat_to_mat(q) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _mat_to_quat(m) -> tuple[float, float, float, float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / norm, y / norm, z / norm, w / norm)


def _load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a YAML mapping")
    return data


def _fastlio_params(path: Path) -> dict:
    data = _load_yaml(path)
    # Standard ROS 2 params file: /** -> ros__parameters -> nested keys.
    root = data.get("/**", data)
    params = root.get("ros__parameters", root) if isinstance(root, dict) else {}
    if not isinstance(params, dict) or "mapping" not in params:
        raise ValueError(f"{path} does not look like a FAST-LIO parameters file")
    return params


def _load_geometry(fastlio_config: Path, mount_config: Path) -> None:
    """Compute T_imu_base from the two package config files."""
    global _R_imu_base, _t_imu_base

    params = _fastlio_params(fastlio_config)
    mapping = params["mapping"]
    R_imu_lidar = _mat3(mapping["extrinsic_R"])
    t_imu_lidar = _vec3(mapping["extrinsic_T"])

    mount = _load_yaml(mount_config)
    R_base_lidar = _mat3(mount["base_to_lidar_R"])
    t_base_lidar = _vec3(mount["base_to_lidar_T"])

    # T_lidar_base = inverse(T_base_lidar)
    R_lidar_base = _transpose(R_base_lidar)
    t_lidar_base = tuple(-v for v in _mat_vec(R_lidar_base, t_base_lidar))
    # T_imu_base = T_imu_lidar * T_lidar_base
    _R_imu_base = _mat_mul(R_imu_lidar, R_lidar_base)
    _t_imu_base = tuple(
        a + b for a, b in zip(_mat_vec(R_imu_lidar, t_lidar_base), t_imu_lidar)
    )
    q = _mat_to_quat(_R_imu_base)
    log.info(
        "T_imu_base: t=(%.6f, %.6f, %.6f) q=(%.6f, %.6f, %.6f, %.6f)",
        *_t_imu_base, *q,
    )


# ── lifecycle ───────────────────────────────────────────────────────────────


@lynx_fastlio.on_init
def init(cfg):
    global _odom_pub, _tf_broadcaster, _odom_frame, _base_frame
    global _max_speed_mps, _max_jump_m, _max_yaw_rate_rps, _last_good, _diverged

    odom_topic = str(cfg.get("odom_topic", "/Odometry"))
    out_topic = str(cfg.get("odom_out_topic", "/fastlio/odom"))
    qos = str(cfg.get("qos", "reliable"))
    timeout = float(cfg.get("sentinel_timeout_s", 60.0))
    _odom_frame = str(cfg.get("odom_frame", "odom"))
    _base_frame = str(cfg.get("base_frame", "base_link"))
    _max_speed_mps = float(cfg.get("max_speed_mps", 2.0))
    _max_jump_m = float(cfg.get("max_jump_m", 0.6))
    _max_yaw_rate_rps = float(cfg.get("max_yaw_rate_rps", 4.0))
    _last_good = None
    _diverged = False
    fastlio_config = PKG_ROOT / "config" / str(
        cfg.get("fastlio_config", DEFAULT_FASTLIO_CONFIG)
    )
    mount_config = PKG_ROOT / "config" / str(
        cfg.get("mount_config", DEFAULT_MOUNT_CONFIG)
    )

    try:
        _load_geometry(fastlio_config, mount_config)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        return Deferred(f"cannot load FAST-LIO geometry: {exc}")

    if not lynx_fastlio.wait_for_topic(odom_topic, Odometry, timeout):
        return Deferred(
            f"no nav_msgs/Odometry on {odom_topic} within {timeout}s "
            "(is FAST-LIO running and initialised?)"
        )

    _odom_pub = lynx_fastlio.create_publisher(
        "robonix/primitive/chassis/odom",
        topic=out_topic,
        msg_type=Odometry,
        qos=qos,
        declare=False,
    )
    lynx_fastlio.declare_ros2_topic(
        "robonix/primitive/chassis/odom", out_topic, qos=qos
    )

    from tf2_ros import TransformBroadcaster  # type: ignore

    _tf_broadcaster = TransformBroadcaster(RosBackend.get().node)

    lynx_fastlio.create_subscription(
        "_fastlio_odom_raw",
        topic=odom_topic,
        msg_type=Odometry,
        callback=_on_odom,
        qos=qos,
        declare=False,
    )

    stale_s = float(cfg.get("stale_warn_s", 10.0))
    if stale_s > 0.0:
        threading.Thread(
            target=_watchdog, args=(stale_s,), name="fastlio-watchdog", daemon=True
        ).start()

    log.info(
        "lynx_fastlio ready: %s (%s -> imu) -> %s (%s -> %s, qos=%s)",
        odom_topic, _odom_frame, out_topic, _odom_frame, _base_frame, qos,
    )
    return Ok()


def _watchdog(stale_s: float) -> None:
    """Warn (do not exit) if FAST-LIO stops publishing; a stale TF silently
    breaks Nav2 and RTAB-Map, so make the failure visible in the logs."""
    last_warn = 0.0
    while True:
        time.sleep(max(2.0, stale_s / 2.0))
        if not _published:
            continue
        silent_for = time.monotonic() - _last_rx_monotonic
        if silent_for > stale_s and time.monotonic() - last_warn > 30.0:
            log.warning(
                "no /Odometry for %.1fs; FAST-LIO may have exited "
                "(TF odom -> %s is now stale)", silent_for, _base_frame
            )
            last_warn = time.monotonic()


def _stamp(msg):
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return Clock().now().to_msg()
    return stamp


def _yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _is_impossible(stamp_s: float, pos, yaw: float) -> bool:
    """True when this pose cannot physically be the robot (LIO diverged)."""
    global _last_good, _diverged
    if _last_good is None:
        _last_good = (stamp_s, pos[0], pos[1], pos[2], yaw)
        return False
    last_t, lx, ly, lz, lyaw = _last_good
    dt = stamp_s - last_t
    if dt <= 1e-3:
        # Same or older stamp: no usable rate, and republication would break TF
        # monotonicity. Leave the reference untouched.
        return False
    dist = math.sqrt((pos[0] - lx) ** 2 + (pos[1] - ly) ** 2 + (pos[2] - lz) ** 2)
    yaw_delta = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)))
    speed = dist / dt
    yaw_rate = yaw_delta / dt
    if speed > _max_speed_mps or dist > _max_jump_m or yaw_rate > _max_yaw_rate_rps:
        if not _diverged:
            _diverged = True
            log.error(
                "FAST-LIO odometry diverged: jumped %.2f m / %.2f rad in %.3fs "
                "(%.1f m/s, %.1f rad/s). Suppressing /fastlio/odom and the "
                "odom -> %s TF so Nav2/RTAB-Map do not act on it. FAST-LIO "
                "cannot recover by itself — restart the stack.",
                dist, yaw_delta, dt, speed, yaw_rate, _base_frame,
            )
        return True
    _last_good = (stamp_s, pos[0], pos[1], pos[2], yaw)
    if _diverged:
        _diverged = False
        log.warning("FAST-LIO odometry is sane again; resuming /fastlio/odom and TF")
    return False


def _on_odom(msg):
    global _published, _last_rx_monotonic
    if _odom_pub is None:
        return

    _last_rx_monotonic = time.monotonic()
    stamp = _stamp(msg)

    # FAST-LIO publishes T_world_imu in pos/orientation. Re-express it in the
    # initial base_link frame (gravity aligned, base at the origin at t=0):
    #
    #   T_odom_base = inverse(T_imu_base) * T_world_imu * T_imu_base
    #
    # A plain right-multiplication by T_imu_base would only change the child
    # frame; it would leave the whole odom frame tilted by the vertical lidar
    # mount, which breaks Nav2's planar costmaps and RTAB-Map's 3-DoF grid.
    q_imu = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )
    R_world_imu = _quat_to_mat(q_imu)
    t_world_imu = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )

    R_imu_base_inv = _transpose(_R_imu_base)
    R_odom_base = _mat_mul(
        _mat_mul(R_imu_base_inv, R_world_imu), _R_imu_base
    )
    # t_odom_base = R_imu_base^-1 * (t_world_imu + (R_world_imu - I) * t_imu_base)
    delta = tuple(
        r - t for r, t in zip(_mat_vec(R_world_imu, _t_imu_base), _t_imu_base)
    )
    t_odom_base = _mat_vec(
        R_imu_base_inv, tuple(w + d for w, d in zip(t_world_imu, delta))
    )
    q_base = _mat_to_quat(R_odom_base)

    stamp_s = stamp.sec + stamp.nanosec * 1e-9
    if _is_impossible(stamp_s, t_odom_base, _yaw_of(q_base)):
        return

    out = Odometry()
    out.header.stamp = stamp
    out.header.frame_id = _odom_frame
    out.child_frame_id = _base_frame
    out.pose = msg.pose
    out.pose.pose.position.x = t_odom_base[0]
    out.pose.pose.position.y = t_odom_base[1]
    out.pose.pose.position.z = t_odom_base[2]
    out.pose.pose.orientation.x = q_base[0]
    out.pose.pose.orientation.y = q_base[1]
    out.pose.pose.orientation.z = q_base[2]
    out.pose.pose.orientation.w = q_base[3]
    # FAST-LIO leaves twist at zero; copy whatever it did publish.
    out.twist = msg.twist
    _odom_pub.publish(out)

    if _tf_broadcaster is not None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = _odom_frame
        tf.child_frame_id = _base_frame
        tf.transform.translation.x = t_odom_base[0]
        tf.transform.translation.y = t_odom_base[1]
        tf.transform.translation.z = t_odom_base[2]
        tf.transform.rotation.x = q_base[0]
        tf.transform.rotation.y = q_base[1]
        tf.transform.rotation.z = q_base[2]
        tf.transform.rotation.w = q_base[3]
        _tf_broadcaster.sendTransform(tf)

    _published += 1
    if _published == 1:
        log.info(
            "first odom: %s -> %s t=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
            _odom_frame, _base_frame, *t_odom_base, *q_base,
        )


@lynx_fastlio.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    lynx_fastlio.run()
