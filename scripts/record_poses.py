#!/usr/bin/env python3
"""Print live /arm/joint_states_single angles so the arm can be hand-taught.

Usage:  python3 scripts/record_poses.py [seconds]
Then move the arm by hand; the last printed line before timeout is the pose.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Watch(Node):
    def __init__(self):
        super().__init__("pose_recorder")
        self.create_subscription(JointState, "/arm/joint_states_single", self._on, 20)
        self.last = None

    def _on(self, msg: JointState) -> None:
        self.last = msg


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Watch()
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.last is not None:
            msg = node.last
            deg = [round(math.degrees(float(p)), 2) for p in msg.position]
            print(f"[{time.strftime('%H:%M:%S')}] {deg}", flush=True)
            time.sleep(0.8)
    if node.last is not None:
        deg = [round(math.degrees(float(p)), 2) for p in node.last.position]
        print("\nLAST POSE:", deg, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
