---
description: Lynx S10 广角 RGB 相机原语 — RTSP 拉流 + 实时 rgb 流 + 按需快照
---

# lynx_camera — Lynx S10 wide-angle RGB camera primitive

S10 的深度话题 `/DEPTH_IMAGE` 虽然注册了发布端，但数据不流动（感知栈未运行）；
而四路广角 RGB 相机通过机身上的 MediaMTX（`rtsp://10.21.33.103:8554`）稳定输出
H.264 1024×576@30。本原语拉取这些 RTSP 流，解码为 BGR，并把每个启用的通道发布成
标准 `sensor_msgs/Image`（bgr8）。主通道额外声明为 `robonix/primitive/camera/rgb`
数据面，并通过 `robonix/primitive/camera/snapshot`（rpc）给大模型智能体按需取一帧 JPEG。

## 接口

| 能力约定 ID | 模式 | 载荷 | 说明 |
|---|---|---|---|
| `robonix/primitive/camera/rgb` | topic_out | `sensor_msgs/Image` | 主通道连续 RGB 流（bgr8），指向 `/camera/<primary>/rgb/image_raw` |
| `robonix/primitive/camera/snapshot` | rpc | `camera/GetCameraImage` | 取主通道最新一帧，JPEG 编码（`encoding='jpeg'`） |

非主通道（`primary: false`）只发布为普通 ROS2 话题 `/camera/<name>/rgb/image_raw`，
不进入 Atlas 数据面，节省路由资源。

## 注意

- 只提供 **RGB**，不提供深度（`/DEPTH_IMAGE` 无数据）。前 video4、后 video0、左 video2、右 video6。
- `snapshot` 返回 JPEG 编码的 `sensor_msgs/Image`（`encoding='jpeg'`），智能体直接当图用。
- 未声明 `intrinsics`/`extrinsics`：S10 的相机几何应由 URDF/TF 提供，需要时再补。
- 使用系统 `ffmpeg` 通过 TCP 取流，`publish_hz` 限制输出帧率；无需 OpenCV。
  断流超时后自动重连，断流或缓存过期时快照明确报错。

## Config（部署清单 `config:`）

| key | default | 含义 |
|-----|---------|------|
| `channels` | — | 通道列表，每项 `{name, url, enabled, primary}` |
| `channels[].name` | `video{i}` | 通道名，决定话题 `/camera/<name>/rgb/image_raw` 与 frame_id |
| `channels[].url` | — | RTSP 地址 |
| `channels[].enabled` | `true` | 是否拉流；`false` 可省资源（解码 + DDS 都不占） |
| `channels[].primary` | 首个 enabled | 作为 `camera/rgb` + `snapshot` 的通道 |
| `qos` | `best_effort` | 声明的 QoS |
| `publish_hz` | `10.0` | 每通道 DDS 发布频率上限 |
| `jpeg_quality` | `85` | snapshot 的 JPEG 质量 |
| `frame_timeout_s` | `5.0` | 无完整帧时重连；也是快照允许的最大帧龄 |
| `sentinel_timeout_s` | `20.0` | 等待主通道首帧的超时；设为 `0` 时后台连接，不阻塞其他服务启动 |

只保留一路省资源时，把其余三项的 `enabled` 置为 `false` 即可，无需改代码。
