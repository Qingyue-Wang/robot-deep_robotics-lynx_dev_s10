#!/bin/bash
# 清理所有 robonix 相关残留进程（避免 pkill -f 匹配到自身命令行）
cd /home/dog/robot-deep_robotics-lynx_s10 || exit 1
pats=('rbnx start' 'rbnx boot' 'robonix-atlas' 'robonix-soma' 'robonix-executor' 'robonix-pilot' 'robonix-liaison' 'soma_runtime' 'odom_guard.py' 'piper_single' 'realsense2_camera_node' 'lynx_quadruped.main' 'lynx_lidar' 'lynx_fastlio' 'lynx_imu' 'laser_mapping' 'rtabmap' 'bt_navigator' 'controller_server' 'planner_server' 'velocity_guard' 'map_to_odom_bridge' 'grasp_geometry' 'grasp_skill' 'depth_grasp')
for p in "${pats[@]}"; do
  pkill -f -- "$p" 2>/dev/null
done
sleep 3
for p in "${pats[@]}"; do
  pkill -9 -f -- "$p" 2>/dev/null
done
sleep 2
# 清掉被强杀进程留下的 Fast DDS 共享内存段（不清的话新节点 rclpy 建 Node 会卡死几十秒）。
# 只在确实没有 ROS 相关进程残留时才清，避免误伤正在跑的节点。
if ! pgrep -f -- "robonix-|rbnx start|piper_single|realsense2_camera_node|fastlio_mapping|rtabmap|nav2_" >/dev/null 2>&1; then
  n=$(ls /dev/shm 2>/dev/null | grep -c 'fastrtps_')
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  echo "[shm] 清理了 ${n} 个残留 fastrtps 段"
else
  echo "[shm] 仍有 ROS 进程在跑，跳过 /dev/shm 清理"
fi
echo "=== 剩余 ==="
ps -eo pid,etime,cmd | grep -E "robonix|rbnx|piper_single|realsense2_camera|lynx_|rtabmap|nav2" | grep -v grep | cut -c1-130
echo "=== end ==="
