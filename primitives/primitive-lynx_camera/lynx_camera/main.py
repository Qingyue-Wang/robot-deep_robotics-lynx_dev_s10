#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_camera — DeepRobotics Lynx S10 wide-angle RGB camera primitive.

The S10's depth topic (``/DEPTH_IMAGE``) is registered but not flowing, while
its four wide-angle RGB cameras are served over RTSP by a MediaMTX instance on
the robot host (``rtsp://10.21.33.103:8554/video{0,2,4,6}``, H.264 1024x576@30).
This primitive pulls those RTSP streams, decodes them to BGR, and publishes each
enabled channel as a standard ``sensor_msgs/Image`` (BGR8). The primary channel
is additionally declared as ``robonix/primitive/camera/rgb`` and served by the
``robonix/primitive/camera/snapshot`` MCP tool (one-shot JPEG for the pilot).

Resource knob: each channel has an ``enabled`` flag. Disable all but one to cut
decode + DDS traffic (see the deploy manifest ``config:``).
"""
from __future__ import annotations

import logging
import os
import threading
import time

import cv2
import numpy as np

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_camera] %(message)s",
)
log = logging.getLogger("lynx_camera")

lynx_camera = Primitive(id="lynx_camera", namespace="robonix/primitive/camera")

# FFMPEG RTSP backend: force TCP transport (reliable over the S10's switch).
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")

from sensor_msgs.msg import Image  # noqa: E402

# Generated codegen artifacts (wired up by robonix_api / rbnx boot).
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from camera_mcp import GetCameraImage_Request, GetCameraImage_Response  # noqa: E402
from sensor_msgs_mcp import Image as MCPImage  # noqa: E402


# ── runtime state ────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_latest: dict[str, np.ndarray] = {}      # channel name -> latest BGR frame
_primary_name: str = ""                  # channel served as camera/rgb + snapshot
_primary_frame_evt = threading.Event()
_stop = threading.Event()
_threads: list[threading.Thread] = []
_qos = "best_effort"
_publish_hz = 10.0
_jpeg_quality = 85


# ── helpers ──────────────────────────────────────────────────────────────────
def _stamp_now() -> tuple[int, int]:
    t = time.time()
    return int(t), int((t % 1.0) * 1e9) % 1_000_000_000


def _mcp_header(frame_id: str) -> std_msgs_mcp.Header:
    sec, nsec = _stamp_now()
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=nsec),
        frame_id=frame_id,
    )


# ── RTSP decoder thread ──────────────────────────────────────────────────────
def _decoder_loop(name: str, url: str, pub) -> None:
    """Pull one RTSP channel and publish BGR8 frames until stopped.

    Reconnects on read failure. Always caches the freshest frame so snapshot is
    never older than one decode; publishing is throttled to ``_publish_hz`` to
    keep DDS traffic bounded across four channels.
    """
    backoff = 1.0
    while not _stop.is_set():
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            log.warning("channel %s: open failed (%s); retry in %.1fs", name, url, backoff)
            _stop.wait(backoff)
            backoff = min(backoff * 2.0, 10.0)
            continue
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low
        backoff = 1.0
        log.info("channel %s: connected (%s)", name, url)
        next_pub = 0.0
        try:
            while not _stop.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    log.warning("channel %s: read failed; reconnecting", name)
                    break
                now = time.monotonic()
                with _state_lock:
                    _latest[name] = frame
                if name == _primary_name:
                    _primary_frame_evt.set()
                if now >= next_pub:
                    img = Image()
                    img.header.stamp.sec, img.header.stamp.nanosec = _stamp_now()
                    img.header.frame_id = f"camera_{name}_optical_frame"
                    img.height = int(frame.shape[0])
                    img.width = int(frame.shape[1])
                    img.encoding = "bgr8"
                    img.is_bigendian = 0
                    img.step = int(frame.shape[1]) * 3
                    img.data = frame.tobytes()
                    pub.publish(img)
                    next_pub = now + (1.0 / _publish_hz)
        finally:
            cap.release()
        if not _stop.is_set():
            _stop.wait(1.0)


# ── MCP snapshot (primary channel) ───────────────────────────────────────────
@lynx_camera.mcp("robonix/primitive/camera/snapshot")
def snapshot(req: GetCameraImage_Request) -> GetCameraImage_Response:
    """Return the primary channel's current RGB frame as a JPEG-encoded
    sensor_msgs/Image (encoding='jpeg', data=JPEG bytes). Use this to see what
    the robot's main camera is looking at."""
    _ = req
    with _state_lock:
        frame = _latest.get(_primary_name)
    if frame is None:
        raise RuntimeError("no RGB frame received yet")
    ok, buf = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), _jpeg_quality]
    )
    if not ok:
        raise RuntimeError("jpeg encode failed")
    h, w = frame.shape[:2]
    return GetCameraImage_Response(image=MCPImage(
        header=_mcp_header(f"camera_{_primary_name}_optical_frame"),
        height=int(h), width=int(w),
        encoding="jpeg",
        is_bigendian=0,
        step=len(buf),
        data=bytes(buf),
    ))


# ── lifecycle ────────────────────────────────────────────────────────────────
@lynx_camera.on_init
def init(cfg: dict):
    """Spawn one decoder thread per enabled channel; declare the primary as
    camera/rgb once its first frame lands."""
    global _primary_name, _qos, _publish_hz, _jpeg_quality

    raw_channels = cfg.get("channels", []) or []
    if not raw_channels:
        return Err("no channels configured (config.channels is empty)")

    _qos = str(cfg.get("qos", "best_effort"))
    _publish_hz = max(1.0, float(cfg.get("publish_hz", 10.0)))
    _jpeg_quality = int(cfg.get("jpeg_quality", 85))
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 20.0))

    channels = []
    for i, ch in enumerate(raw_channels):
        name = str(ch.get("name", f"video{i}"))
        url = str(ch.get("url", "")).strip()
        enabled = bool(ch.get("enabled", True))
        primary = bool(ch.get("primary", False))
        if not url:
            log.warning("channel %s: empty url, skipping", name)
            continue
        channels.append({"name": name, "url": url, "enabled": enabled, "primary": primary})

    enabled = [c for c in channels if c["enabled"]]
    if not enabled:
        return Err("no enabled channels (all channels have enabled=false)")

    # Primary = explicit primary flag, else the first enabled channel.
    primaries = [c for c in enabled if c["primary"]]
    primary = primaries[0] if primaries else enabled[0]
    _primary_name = primary["name"]
    log.info("primary channel: %s", _primary_name)

    pubs: dict[str, object] = {}
    for ch in enabled:
        name = ch["name"]
        topic = f"/camera/{name}/rgb/image_raw"
        # Primary is declared below as the atlas contract; others stay internal.
        cid = "robonix/primitive/camera/rgb" if name == _primary_name else f"_rgb_{name}"
        pubs[name] = lynx_camera.create_publisher(
            cid, topic=topic, msg_type=Image, qos=_qos, declare=False,
        )

    for ch in enabled:
        t = threading.Thread(
            target=_decoder_loop, args=(ch["name"], ch["url"], pubs[ch["name"]]),
            daemon=True,
        )
        t.start()
        _threads.append(t)

    # Gate INIT on the primary channel's first frame.
    if not _primary_frame_evt.wait(timeout=sentinel_timeout):
        return Err(
            f"no frame from primary channel {_primary_name} within {sentinel_timeout:.1f}s"
        )

    lynx_camera.declare_ros2_topic(
        "robonix/primitive/camera/rgb",
        topic=f"/camera/{_primary_name}/rgb/image_raw",
        qos=_qos,
    )
    log.info(
        "lynx_camera ready: %d channel(s), primary=%s rgb=%s qos=%s publish_hz=%.1f",
        len(enabled), _primary_name, f"/camera/{_primary_name}/rgb/image_raw",
        _qos, _publish_hz,
    )
    return Ok()


@lynx_camera.on_shutdown
def shutdown():
    _stop.set()
    for t in _threads:
        t.join(timeout=2.0)
    return Ok()


if __name__ == "__main__":
    lynx_camera.run()
