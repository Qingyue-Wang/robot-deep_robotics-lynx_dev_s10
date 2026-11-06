#!/usr/bin/env python3
"""转发 nos 上的 /LIDAR/POINTS -> /LIDAR/POINTS_fwd。

用途：nos 的原始雷达点云在 nos 上有 publisher，但 NX 因 DDS 发现配置收不到。
在 nos 上跑这个脚本，用默认 DDS 配置重新发布，NX 就能收到。

用法：
    python3 relay_pc2.py

若收不到数据（看不到订阅匹配），在 nos 上执行：
    ros2 topic info /LIDAR/POINTS -v      # 看它的 QoS profile
然后把下面的 qos 改成一致即可：
  - Reliability BEST_EFFORT  -> 用 sensor_data（当前默认）
  - Reliability RELIABLE     -> 改成 rclpy.qos.qos_profile_system_default
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Relay(Node):
    def __init__(self):
        super().__init__('relay_pc2')
        # 雷达点云几乎都是 BEST_EFFORT，先用 sensor_data QoS。
        # 收不到就把这个 profile 换成 qos_profile_system_default。
        self.sub = self.create_subscription(
            PointCloud2, '/LIDAR/POINTS', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, '/LIDAR/POINTS_fwd', qos_profile_sensor_data)
        self.get_logger().info('relay /LIDAR/POINTS -> /LIDAR/POINTS_fwd')

    def cb(self, msg: PointCloud2):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == '__main__':
    main()
