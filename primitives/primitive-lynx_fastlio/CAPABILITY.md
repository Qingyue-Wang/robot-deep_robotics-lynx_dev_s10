---
description: Lynx S10 FAST-LIO 里程计 — 前 Airy 雷达 + 内置 IMU，修正为 odom -> base_link
---

# lynx_fastlio — Lynx S10 FAST-LIO odometry primitive

用前雷达 Airy（`airy_201`，XYZIRT）和它的内置 IMU 跑 FAST-LIO，把算法输出的
IMU body 位姿换算成机器人 `base_link` 位姿，声明为底盘里程计契约，同时广播
`odom -> base_link` TF。mapping 绑它作外部里程计后不再启动 rtabmap 的
`icp_odometry`；nav2 仍通过 `lynx_quadruped` 转发得到的 `/odom` 消费里程计。

## 接口

### `robonix/primitive/chassis/odom` — topic_out (`nav_msgs/Odometry`)

- endpoint：`/fastlio/odom`
- `header.frame_id`：`odom`
- `child_frame_id`：`base_link`
- 时间戳：沿用 FAST-LIO `/Odometry` 的 header stamp

同一条消息同时以相同位姿广播 `odom -> base_link` TF（`/tf`），供 rtabmap 的
外部里程计和 nav2 的 costmap 查找。TF 与消息严格同源。

## 输入（话题，由本包 start.sh 配置的 FAST-LIO 节点消费）

| topic | type | 说明 |
|-------|------|------|
| `/airy_201/points` | `sensor_msgs/PointCloud2` | 前 Airy，XYZIRT，`ts_first_point: true` |
| `/airy_201/imu` | `sensor_msgs/Imu` | Airy 内置 IMU（rslidar_sdk 发布，原始设备时间） |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 输出（world -> IMU body） |

## 坐标换算

FAST-LIO 的世界系是**初始 IMU 系**，而 IMU 随垂直安装的雷达倾斜，所以不能直接
把它的位姿当作 odom→base_link。原语把输出重新表达在**初始 base_link 系**（近似
重力对齐、t=0 时机器人在原点）里：

```
T_odom_base(t) = inverse(T_imu_base) * T_world_imu(t) * T_imu_base
T_imu_base     = T_imu_lidar * inverse(T_base_link_airy)
```

这是换基（conjugation），不是简单的右乘：只有换基才能让 odom 的 z 轴保持竖直，
否则 nav2 的平面 costmap 和 RTAB-Map 的 3-DoF 栅格会被雷达安装倾角带歪。

- `T_imu_lidar`：`config/robosense_airy_s10.yaml` 的
  `mapping.extrinsic_R/T`（LiDAR 在 IMU body 下的位姿，Airy 内部标定）。
- `T_base_link_airy`：`config/s10_mount.yaml` 的 `base_to_lidar_R/T`，与
  `primitive-lynx_lidar` 发布的 `base_link -> airy_201` 静态 TF 完全一致
  （垂直安装：xyz `0.22341 0 -0.0001`，rpy `pi -pi/2 0`）。

## Config（部署清单 `config:`）

| key | default | 含义 |
|-----|---------|------|
| `odom_topic` | `/Odometry` | FAST-LIO 原始里程计话题 |
| `odom_out_topic` | `/fastlio/odom` | 修正后的底盘里程计输出话题 |
| `odom_frame` | `odom` | 输出/ TF 父 frame |
| `base_frame` | `base_link` | 输出/ TF 子 frame |
| `qos` | `reliable` | 订阅/发布 QoS（FAST-LIO 以 reliable 发布） |
| `sentinel_timeout_s` | `60.0` | 等待 `/Odometry` 首帧的超时，超时返回 Deferred 重试 |
| `stale_warn_s` | `10.0` | 运行中超过该时长没有 `/Odometry` 时打印告警（不退出，避免静默失效） |
| `fastlio_config` | `robosense_airy_s10.yaml` | 读取外参用的 FAST-LIO 参数文件名 |
| `mount_config` | `s10_mount.yaml` | 读取安装外参的文件名 |

## 不做什么

- 不建图、不做回环：地图仍由 `service-map-rbnx`（rtabmap）负责，本原语只提供
  里程计与 TF。
- 不修改 FAST-LIO 的算法参数；参数在 `config/robosense_airy_s10.yaml` 中。
- 不发布 `base_link -> airy_201` TF；那是 `primitive-lynx_lidar` 的职责。
