# primitive-lynx_imu

Robonix IMU primitive for the DeepRobotics Lynx S10 ("山猫"). The S10 publishes
a standard `sensor_msgs/Imu` on `/IMU`, but leaves `header.frame_id` empty, so
this package re-stamps the stream (`frame_id` default `imu_link`) and declares
the re-stamped topic as `robonix/primitive/imu/imu` for Atlas-based discovery.
`scripts/start.sh` also publishes the `base_link -> imu_link` static TF.

| S10 topic  | type                | direction |
|------------|---------------------|-----------|
| `/IMU`     | `sensor_msgs/Imu`   | in (re-stamped -> `/IMU/framed`) |
| `/IMU_103` | `sensor_msgs/Imu`   | alternative source (lidar-internal IMU) |

Exposed Robonix capability: `robonix/primitive/imu/imu`.

## Build & run

```sh
rbnx codegen -p . --mcp   # codegen only
bash scripts/start.sh     # publishes base_link->imu_link TF, runs lynx_imu.main
```

See `CAPABILITY.md` for the contract and config keys.
