# primitive-lynx_camera

Maintained in this robot repository at `primitives/primitive-lynx_camera`,
including `lynx_camera/`, `tests/`, scripts and documentation.
Deployment manifests reference this local package with `path:`.

Robonix wide-angle RGB camera primitive for the DeepRobotics Lynx S10 ("山猫").
The S10's depth topic (`/DEPTH_IMAGE`) is registered but not flowing; its four
wide-angle RGB cameras are served over RTSP by a MediaMTX instance
(`rtsp://10.21.33.103:8554/video{0,2,4,6}`, H.264 1024x576@30). This package
pulls those streams through ffmpeg, decodes to BGR, and exposes the primary channel as
`robonix/primitive/camera/rgb` plus a one-shot `snapshot` tool for LLM agents.

| source | type | direction |
|--------|------|-----------|
| RTSP `video{0,2,4,6}` | H.264 → `sensor_msgs/Image` (bgr8) | in (declared as `camera/rgb` + snapshot) |

Exposed Robonix capabilities: `robonix/primitive/camera/rgb`,
`robonix/primitive/camera/snapshot`.

## Build & run

Requires ROS 2, `ffmpeg`, NumPy and Pillow in the runtime Python environment.
Set `LYNX_CAMERA_PYTHON` (or `LYNX_PYTHON`) to use a virtual environment.

```sh
rbnx codegen -p . --mcp
bash scripts/start.sh
```

See `CAPABILITY.md`. Depth is intentionally not provided (no flowing depth
source on this robot); RGB-only, per the deployment decision.

Enable the front camera with this primitive config:

```yaml
channels:
  - {name: video4, url: "rtsp://10.21.33.103:8554/video4", primary: true}
publish_hz: 5.0
frame_timeout_s: 5.0
sentinel_timeout_s: 15.0
```

Set Scene's `camera_provider_id: lynx_camera`. If Scene has explicit
`observations`, also add `{kind: rgb, contract: robonix/primitive/camera/rgb,
provider_id: lynx_camera, msg_type: Image}`. Camera and Scene must use the same
ROS domain. `/cam` displays the image; `/api/camera` returns its preview.
The primitive reconnects on missing frames and rejects stale snapshots.
