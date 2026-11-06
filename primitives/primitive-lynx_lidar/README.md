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

`scripts/start.sh` sources `~/rslidar_ws`, publishes the `base_link -> airy_201`
(front, pitched) static TF from the S10 URDF, launches `rslidar_sdk_node`, then
runs the primitive; the driver and TF publisher are torn down with the primitive.
Override the workspace with `RSLIDAR_WS=/path/to/ws` if it is not under `~/rslidar_ws`.

See `CAPABILITY.md`. The rslidar topics have no flowing data until the lidar is
powered and returning UDP; the primitive stays Deferred until then.
