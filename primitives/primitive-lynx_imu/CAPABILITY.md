---
description: Lynx S10 IMU 原语 — 给原生 /IMU 补 frame_id 并声明为 imu 数据面
---

# lynx_imu — Lynx S10 IMU primitive

S10 在 `/IMU` 上原生发布标准 `sensor_msgs/Imu`（已融合），但 **`header.frame_id`
留空**；另有一路雷达内置 IMU 在 `/IMU_103`。rtabmap 的 ICP 里程计需要真实 frame
才能做 `base_link -> imu` 查找，否则每帧 IMU 都被丢弃。

本原语确认原生流存活后，把消息重打上配置的 frame_id（默认 `imu_link`），转发到
重打标话题（默认 `/IMU/framed`），并把该话题声明为
`robonix/primitive/imu/imu` 数据面。`scripts/start.sh` 同时发布
`base_link -> imu_link` 静态 TF（S10 URDF `imu_joint` 外参）。

## 接口

### `robonix/primitive/imu/imu` — topic_out (`sensor_msgs/Imu`)

单向输出流，指向重打标话题（默认 `/IMU/framed`，frame_id `imu_link`）。消费方按
契约发现，得到的 endpoint 是 `/IMU/framed`。

## Config（部署清单 `config:`）

| key | default | 含义 |
|-----|---------|------|
| `imu_topic` | `/IMU` | 源 IMU 话题（`/IMU_103` 为雷达内置 IMU 备选） |
| `out_topic` | `/IMU/framed` | 重打标后的输出话题 |
| `frame_id` | `imu_link` | 打在输出消息 header 上的 frame |
| `qos` | `best_effort` | 订阅/发布 QoS（与 S10 发布端一致） |
| `sentinel_timeout_s` | `15.0` | 等待首帧 IMU 的超时，超时返回 Deferred 重试 |

## 外参

`base_link -> imu_link` 静态 TF 由 `scripts/start.sh` 发布，取值 S10 URDF
`imu_joint`：`xyz = 0.0075 -0.0373 0.0556`，`rpy = 1.57 1.57 0`。

## 不做什么

- 不做坐标变换（只补 frame_id，不改数值）；frame_id 统一为重打标的 `imu_link`。
- 不把 `/IMU_DATA`（`drdds/ImuData`，原始欧拉角/角速度/加速度）转成标准消息；
  需要时另做转换层。
