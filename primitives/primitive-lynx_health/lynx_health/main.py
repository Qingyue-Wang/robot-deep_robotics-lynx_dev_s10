#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_health — DeepRobotics Lynx S10 device-health primitive.

Aggregates the S10's native drdds telemetry into the standard
``robonix/primitive/health`` surface for Vitals:

  /BATTERY_DATA (drdds/BatteryData) -> system voltage, charging flag,
                                        battery % + cell temperature
  /cpu_status   (drdds/CPUStatus)   -> per-SoC package temperature readings

Exposes ``robonix/primitive/health/state`` (unary) and
``robonix/primitive/health/stream`` (server-stream).
"""
from __future__ import annotations

import logging
import os
import threading

from robonix_api import Primitive, Ok, Deferred

# S10 native messages, from the prebuilt drdds package on the Jetson.
from drdds.msg import BatteryData, CPUStatus  # noqa: E402

# gRPC codegen (rbnx-build/codegen/proto_gen, wired by rbnx boot).
import health_pb2  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_health] %(message)s",
)
log = logging.getLogger("lynx_health")

lynx_health = Primitive(id="lynx_health", namespace="robonix/primitive/health")

# ── shared state (guarded by _state_lock) ───────────────────────────────────
_state_lock = threading.Lock()
_battery = None      # latest drdds BatteryDataValue (msg.data[0])
_cpu_temps = {}      # soc_id -> package_temp (°C)
_stop = threading.Event()

_interval_s = 1.0


def _on_battery(msg: BatteryData) -> None:
    global _battery
    with _state_lock:
        _battery = msg.data[0] if msg.data else None


def _on_cpu(msg: CPUStatus) -> None:
    global _cpu_temps
    with _state_lock:
        _cpu_temps[str(msg.soc_id)] = int(msg.package_temp)


def _reading(name, *, temp_c=-1.0, voltage=-1.0, current_a=-1.0,
             battery_percent=-1.0) -> "health_pb2.SensorReading":
    return health_pb2.SensorReading(
        name=name, temp_c=temp_c, voltage=voltage,
        current_a=current_a, battery_percent=battery_percent,
    )


def _build_health_state() -> "health_pb2.HealthState":
    with _state_lock:
        bat = _battery
        cpu = dict(_cpu_temps)

    readings = []
    voltage = -1.0
    charging = False
    remaining_s = -1

    if bat is not None:
        voltage = float(bat.voltage) * 0.01        # uint16, 10 mV -> V
        current_a = float(bat.current) * 0.01      # int16, 10 mA -> A
        battery_pct = float(bat.battery_level)     # uint8, 0..100 %
        temp_c = -1.0
        if bat.battery_temperature:
            temp_c = float(sum(bat.battery_temperature)
                           / len(bat.battery_temperature))
        # Sign heuristic: negative current => charging. No dedicated charge
        # flag is read here (see /CHARGE_STATUS if a hard signal is needed).
        charging = current_a < 0.0
        if current_a > 0.0:
            remaining_mah = float(bat.remaining_capacity) * 10.0  # 10 mAH -> mAH
            current_ma = current_a * 1000.0
            remaining_s = int(remaining_mah / current_ma * 3600.0)
        readings.append(_reading(
            "battery", temp_c=temp_c, voltage=voltage,
            current_a=current_a, battery_percent=battery_pct,
        ))

    for soc_id in sorted(cpu):
        readings.append(_reading(f"cpu_{soc_id}", temp_c=float(cpu[soc_id])))

    return health_pb2.HealthState(
        voltage=voltage, charging=charging, remaining_s=remaining_s,
        readings=readings,
    )


@lynx_health.grpc("robonix/primitive/health/state")
def get_health_state(_request) -> "health_pb2.GetHealthState_Response":
    """Return the latest S10 health snapshot (battery + SoC temperatures).

    Fields: voltage (V), charging (bool), remaining_s (estimated, -1 unknown),
    and one SensorReading per source (battery, cpu_<soc_id>). -1 marks any
    unavailable value. Contract: robonix/primitive/health/state."""
    return health_pb2.GetHealthState_Response(state=_build_health_state())


@lynx_health.grpc("robonix/primitive/health/stream")
def stream_health_state(_request, context):
    """Push one HealthState per interval until the consumer disconnects.

    Contract: robonix/primitive/health/stream (server-stream)."""
    while context.is_active() and not _stop.is_set():
        yield _build_health_state()
        _stop.wait(_interval_s)


@lynx_health.on_init
def init(cfg):
    global _interval_s
    battery_topic = cfg.get("battery_topic", "/BATTERY_DATA")
    cpu_topic = cfg.get("cpu_topic", "/cpu_status")
    _interval_s = float(cfg.get("interval_s", 1.0))
    timeout = float(cfg.get("sentinel_timeout_s", 15.0))

    # drdds topics are published RELIABLE; a best-effort subscriber would not
    # match, so subscribe reliable.
    lynx_health.create_subscription(
        "_battery", topic=battery_topic, msg_type=BatteryData,
        callback=_on_battery, qos="reliable", declare=False)
    lynx_health.create_subscription(
        "_cpu", topic=cpu_topic, msg_type=CPUStatus,
        callback=_on_cpu, qos="reliable", declare=False)

    # The battery stream is the one required source for readiness.
    if not lynx_health.wait_for_topic(battery_topic, BatteryData, timeout):
        return Deferred(f"no BatteryData on {battery_topic} within {timeout}s")

    _stop.clear()
    log.info("lynx_health ready: battery=%s cpu=%s interval_s=%s",
             battery_topic, cpu_topic, _interval_s)
    return Ok()


@lynx_health.on_shutdown
def shutdown():
    _stop.set()
    return Ok()


if __name__ == "__main__":
    lynx_health.run()
