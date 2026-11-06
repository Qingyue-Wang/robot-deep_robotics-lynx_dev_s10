---
description: Lynx S10 3D 激光雷达原语 — 把 rslidar_sdk 的 PointCloud2 声明为 lidar3d
---

# lynx_lidar — Lynx S10 3D lidar primitive

S10 装有两台 RoboSense Airy 雷达，由 NX 上的 `rslidar_sdk`（`~/rslidar_ws`）驱动，
已原生发布标准 `sensor_msgs/PointCloud2`：

| 话题 | frame | 说明 |
|---|---|---|
| `/airy_201/points` | `airy_201` | 前雷达 — 竖装（pitch +90°），扫描面为竖直圆盘 |
| `/airy_202/points` | `airy_202` | 后雷达 — 平装（yaw 180°），扫描面为水平 360° 圆盘 |
| `/LIDAR/POINTS` | — | 机器人 LIO 融合点云，仅感知栈运行时发布（备选） |

本原语确认流存活后，把其中一个话题声明为 `robonix/primitive/lidar/lidar3d`。

## 接口

### `robonix/primitive/lidar/lidar3d` — topic_out (`sensor_msgs/PointCloud2`)

单向输出流，指向配置的源话题（默认前雷达 `/airy_201/points`，竖直扫描）。

## 为什么只暴露一台

两台雷达在不同 frame（`airy_201` / `airy_202`），把两帧点云合并成单条
`lidar3d` 需要 TF 变换到公共坐标系；这里不自行融合，交给机器人 LIO 融合源
（`/LIDAR/POINTS`）或后续的 TF 合并层。需要同时覆盖两台时，把 `points_topic`
切到融合源，或另加一个 provider。

## Config（部署清单 `config:`）

| key | default | 含义 |
|-----|---------|------|
| `points_topic` | `/airy_201/points` | 源点云话题（前雷达，竖直扫描；`/airy_202/points` 为后雷达，水平扫描） |
| `qos` | `reliable` | 声明的 QoS（rslidar_sdk 为 RELIABLE） |
| `sentinel_timeout_s` | `15.0` | 等待首帧点云的超时 |

## 注意

rslidar_sdk 节点起来并注册了发布者，但如果雷达没上电/没有 UDP 数据，话题上
不会真的有点云；本原语会保持 `Deferred` 直到点云真正流动。
