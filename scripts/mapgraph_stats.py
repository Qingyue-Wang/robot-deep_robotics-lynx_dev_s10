#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Print loop-closure statistics from the live RTAB-Map pose graph.

The mapping service latches ``/mapGraph`` (rtabmap_msgs/MapGraph), so this is
read-only and works while mapping. Use it to compare a map built with and
without the proximity/ICP experiment in config/rtabmap_params_mapping.yaml:

  * ``links`` by type: neighbor (0), global loop closure (1), local-space (2).
    Without image input RTAB-Map disables bag-of-words, so type 1 stays 0 and
    the only hope is type 2 (proximity + ICP).
  * ``near pairs``: node pairs that are physically close in the map (< --near
    metres) but far apart in time (>= --gap node indices). A round trip should
    leave very few of these unlinked; many unlinked pairs are the double wall.

Typical good result after the experiment: type-2 links up, unlinked near pairs
down, and the two passes no longer sitting ~0.2-0.4 m apart.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

LINK_NAMES = {
    0: "neighbor",
    1: "global_loop",
    2: "local_space",
    3: "user",
    4: "virtual",
    5: "neighbor_merged",
    6: "prior",
}


def _apply_ros_env(manifest_path: str) -> None:
    """Match the stack's ROS domain/RMW (must run before rclpy.init)."""
    try:
        with open(manifest_path, encoding="utf-8") as handle:
            env = (yaml.safe_load(handle) or {}).get("env") or {}
    except OSError as error:
        print(f"[mapgraph] cannot read manifest {manifest_path}: {error}",
              file=sys.stderr)
        return
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
        value = env.get(key)
        if value is not None:
            os.environ[key] = str(value)


def _analyse(msg, near_m: float, gap: int) -> str:
    from collections import Counter

    ids = np.asarray(msg.poses_id, dtype=np.int64)
    xy = np.array([[p.position.x, p.position.y] for p in msg.poses],
                  dtype=np.float64)
    linked: dict[tuple[int, int], list[int]] = {}
    for link in msg.links:
        key = (min(link.from_id, link.to_id), max(link.from_id, link.to_id))
        linked.setdefault(key, []).append(int(link.type))

    types = Counter(int(link.type) for link in msg.links)
    lines = [
        f"poses={len(ids)} links={len(msg.links)} "
        f"types={{{', '.join(f'{LINK_NAMES.get(k, k)}:{v}' for k, v in sorted(types.items()))}}}"
    ]

    near = 0
    unlinked: list[tuple[int, int, float]] = []
    for i in range(len(ids)):
        # j >= i+gap keeps this from counting ordinary path neighbours.
        for j in range(i + gap, len(ids)):
            distance = float(np.linalg.norm(xy[i] - xy[j]))
            if distance >= near_m:
                continue
            near += 1
            key = (min(int(ids[i]), int(ids[j])), max(int(ids[i]), int(ids[j])))
            if key not in linked:
                unlinked.append((int(ids[i]), int(ids[j]), distance))

    lines.append(
        f"near_pairs(<{near_m:.2f} m, >= {gap} nodes apart)={near} "
        f"linked={near - len(unlinked)} unlinked={len(unlinked)}"
    )
    if unlinked:
        unlinked.sort(key=lambda item: item[2])
        sample = ", ".join(f"{a}->{b} {d:.2f}m" for a, b, d in unlinked[:8])
        lines.append(f"closest unlinked: {sample}")
    if ids.size:
        net = float(np.linalg.norm(xy[-1] - xy[0]))
        lines.append(f"first=({xy[0][0]:.2f},{xy[0][1]:.2f}) "
                     f"last=({xy[-1][0]:.2f},{xy[-1][1]:.2f}) net={net:.2f} m")
    return "\n".join(lines)


class GraphStats(Node):
    def __init__(self, near_m: float, gap: int, timeout_s: float) -> None:
        super().__init__("mapgraph_stats")
        self._near_m = near_m
        self._gap = gap
        self._started = time.monotonic()
        self._timeout_s = timeout_s
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(self._msg_type(), "/mapGraph", self._on_graph, qos)
        self.create_timer(0.5, self._check_timeout)

    @staticmethod
    def _msg_type():
        from rtabmap_msgs.msg import MapGraph  # type: ignore
        return MapGraph

    def _on_graph(self, msg) -> None:
        print(_analyse(msg, self._near_m, self._gap))
        raise SystemExit(0)

    def _check_timeout(self) -> None:
        if time.monotonic() - self._started > self._timeout_s:
            print(f"[mapgraph] no /mapGraph within {self._timeout_s:.0f}s "
                  "(is the mapping service running?)", file=sys.stderr)
            raise SystemExit(1)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Print RTAB-Map loop-closure stats from /mapGraph.")
    parser.add_argument(
        "--manifest", default="robonix_manifest.mapping.yaml",
        help="stack manifest; its env: ROS_DOMAIN_ID/RMW are applied")
    parser.add_argument("--near", type=float, default=0.6,
                        help="near-pair distance threshold in metres (default 0.6)")
    parser.add_argument("--gap", type=int, default=20,
                        help="minimum node-index gap for a near pair (default 20)")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="seconds to wait for a latched /mapGraph")
    args, ros_args = parser.parse_known_args(argv)

    _apply_ros_env(args.manifest)
    rclpy.init(args=ros_args or None)
    node = GraphStats(args.near, args.gap, args.timeout)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
