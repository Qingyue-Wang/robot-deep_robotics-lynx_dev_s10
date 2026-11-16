#!/usr/bin/env python3
"""记录 cmd_vel 链路的每一跳，用于定位「导航卡住但报 0 速」的真凶。

链路（nav2 这层的 remap 见 nav2_params_lite3_new.yml）：
    controller_server  →  /cmd_vel_nav          (DWB 的原始输出)
    velocity_smoother  →  /cmd_vel_guard_input  (平滑后的输出)
    velocity_guard     →  /cmd_vel              (守卫后的输出，真正给底盘)

只有这三跳都记下来，才能区分是「控制器本来就给 0」还是「中间某一层把非零归零」。

用法（NX 上，栈在跑时）：
    source /opt/ros/humble/setup.bash
    ROS_DOMAIN_ID=43 setsid nohup python3 scripts/cmd_chain_recorder.py \
        > /tmp/cmd_chain.log 2>&1 < /dev/null &

输出：rbnx-boot/data/cmd_chain_<启动时刻>.jsonl，每行
    {"t": 墙钟秒, "topic": "cmd_vel_nav", "lv": 线速度, "av": 角速度, "age_s": 距上一帧}
分析：scripts/cmd_chain_report.py <jsonl>
"""
import json
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TOPICS = ("/cmd_vel_nav", "/cmd_vel_guard_input", "/cmd_vel")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    out_dir = Path(os.getenv("ROBONIX_CMD_TRACE_DIR", root / "rbnx-boot" / "data"))
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"cmd_chain_{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    fh = path.open("a", encoding="utf-8", buffering=1)

    rclpy.init()
    node = Node("cmd_chain_recorder")
    last = {name: 0.0 for name in TOPICS}
    stats = {name: {"n": 0, "nonzero": 0} for name in TOPICS}

    def make_cb(name):
        def _cb(msg: Twist) -> None:
            now = time.time()
            gap = now - last[name]
            last[name] = now
            lv, av = float(msg.linear.x), float(msg.angular.z)
            fh.write(json.dumps({"t": round(now, 3), "topic": name,
                                 "lv": round(lv, 4), "av": round(av, 4),
                                 "gap_s": round(gap, 3)},
                                separators=(",", ":")) + "\n")
            st = stats[name]
            st["n"] += 1
            if abs(lv) > 1e-3 or abs(av) > 1e-3:
                st["nonzero"] += 1
        return _cb

    for name in TOPICS:
        node.create_subscription(Twist, name, make_cb(name), 20)

    def _report() -> None:
        node.get_logger().info("cmd_chain: " + ", ".join(
            f"{name} {stats[name]['n']}帧(非零{stats[name]['nonzero']})" for name in TOPICS))

    node.create_timer(30.0, _report)
    node.get_logger().info(f"cmd_chain_recorder: 记录 {TOPICS} → {path}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _report()
        fh.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
