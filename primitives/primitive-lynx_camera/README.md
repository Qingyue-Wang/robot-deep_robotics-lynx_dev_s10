# primitive-lynx_camera

Robonix wide-angle RGB camera primitive for the DeepRobotics Lynx S10 ("山猫").
The S10's depth topic (`/DEPTH_IMAGE`) is registered but not flowing; its four
wide-angle RGB cameras are served over RTSP by a MediaMTX instance
(`rtsp://10.21.33.103:8554/video{0,2,4,6}`, H.264 1024x576@30). This package
pulls those streams, decodes to BGR, and exposes the primary channel as
`robonix/primitive/camera/rgb` plus a one-shot `snapshot` tool for LLM agents.

| source | type | direction |
|--------|------|-----------|
| RTSP `video{0,2,4,6}` | H.264 → `sensor_msgs/Image` (bgr8) | in (declared as `camera/rgb` + snapshot) |

Exposed Robonix capabilities: `robonix/primitive/camera/rgb`,
`robonix/primitive/camera/snapshot`.

## Build & run

```sh
rbnx codegen -p . --mcp
bash scripts/start.sh
```

See `CAPABILITY.md`. Depth is intentionally not provided (no flowing depth
source on this robot); RGB-only, per the deployment decision.
