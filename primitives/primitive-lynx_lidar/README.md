# primitive-lynx_lidar

Robonix 3D lidar primitive for the DeepRobotics Lynx S10 ("山猫"). The S10 has
two RoboSense Airy lidars driven by `rslidar_sdk` (`~/rslidar_ws`) on the NX,
publishing standard `sensor_msgs/PointCloud2`:

| topic | frame | notes |
|-------|-------|-------|
| `/airy_201/points` | `airy_201` | front lidar — VERTICAL scan (pitched +90°) (default source) |
| `/airy_202/points` | `airy_202` | back lidar — HORIZONTAL 360° scan |
| `/LIDAR/POINTS` | — | robot LIO-fused cloud (only when perception stack runs) |

Exposed Robonix capability: `robonix/primitive/lidar/lidar3d` (one lidar,
configurable).

## Build & run

```sh
rbnx codegen -p . --mcp
bash scripts/start.sh
```

`scripts/start.sh` loads ROS and `~/rslidar_ws`, then runs the primitive.
Driver INIT launches `rslidar_sdk_node` and the `base_link -> airy_201` static TF;
Driver SHUTDOWN stops only these owned processes. It never kills processes by name.
Override the workspace with `RSLIDAR_WS=/path/to/ws` if it is not under `~/rslidar_ws`.

Set `managed_driver: false` to consume an externally managed cloud. Otherwise
`lidar_executable`, `lidar_config` and `run_dir` configure the owned driver.
A device lock rejects a second managed instance; a helper crash stops the primitive.

See `CAPABILITY.md`. The rslidar topics have no flowing data until the lidar is
powered and returning UDP; the primitive stays Deferred until then.
