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

`scripts/start.sh` publishes the `base_link -> airy_201` (front, pitched) static
TF from the S10 URDF, launches `rslidar_sdk_node`, then runs the primitive; the
driver and TF publisher are torn down with the primitive.

If the driver workspace is missing (fresh NX, wiped `~/rslidar_ws`), `start.sh`
calls `scripts/install.sh` automatically: it clones `rslidar_sdk` +
`rslidar_msg`, applies the deployment config/patches (XYZIRT, BEST_EFFORT point
cloud, Airy ports/host address) and builds with `ENABLE_IMU_DATA_PARSE=ON`.
The install is idempotent — nothing happens once the binary exists — and can be
run by hand:

```sh
bash scripts/install.sh            # install if missing
FORCE=1 bash scripts/install.sh    # re-apply config/patches and rebuild
```

Overrides: `RSLIDAR_WS` (default `~/rslidar_ws`), `RSLIDAR_GIT_MIRROR` (default
`https://gh-proxy.org/`, empty = clone github.com directly), `LIDAR_HOST_IP`
(default `10.21.33.100`), `LYNX_LIDAR_AUTO_INSTALL=0` to disable auto-install.
Source of truth for the driver config: `config/rslidar_config.yaml`.

See `CAPABILITY.md`. The rslidar topics have no flowing data until the lidar is
powered and returning UDP; the primitive stays Deferred until then.
