# primitive-lynx_fastlio

FAST-LIO odometry for the Lynx S10's front RoboSense Airy lidar. It replaces
RTAB-Map's `icp_odometry` as the source of `odom -> base_link` while leaving
RTAB-Map in charge of the map (occupancy grid, loop closure, save/load).

## Why not directly `RuanJY/robosense_fast_lio`?

[RuanJY/robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio) is a
ROS 1 catkin package. This deployment runs ROS 2 Humble natively, and rbnx can
only start packages that ship a Robonix `package_manifest.yaml`. The primitive
therefore wraps a ROS 2 Humble FAST-LIO port with RoboSense Airy support:

- [`skorbiz/fast_lio_fork_ros2_robosense`](https://github.com/skorbiz/fast_lio_fork_ros2_robosense) (branch `ROS2`)

It supports RoboSense Airy as `lidar_type: 5` and bundles the Livox message
definitions, so no `livox_ros_driver2` is needed. The wrapper only depends on
the node publishing `/Odometry`; `FASTLIO_REPO` in `scripts/setup_fastlio.sh`
swaps in another port if needed.

## Install FAST-LIO

```bash
# on the robot NX
cd primitives/primitive-lynx_fastlio
./scripts/setup_fastlio.sh                      # FASTLIO_WS defaults to ~/fastlio_ws
```

The workspace is pinned to a commit. Requirements: ROS 2 Humble,
`ros-humble-pcl-ros`, `ros-humble-pcl-conversions`, Eigen, PCL, and network
access for the first clone (ikd-Tree is a submodule).

The script re-applies the deployment's patches (`patches/*.patch`) after the
pinned checkout, so re-running it is safe and always leaves a reproducible
tree.

If the workspace lives elsewhere, export `FASTLIO_WS` for the whole deployment
(the manifest `env:` block or soma's environment) — `scripts/start.sh` sources
`$FASTLIO_WS/install/setup.bash`.

## rslidar_sdk requirements

The Airy driver (`~/rslidar_ws` on the NX) must publish **XYZIRT** points with

```yaml
POINT_TYPE: XYZIRT
ts_first_point: true
```

`ts_first_point: true` makes the per-point timestamp relative to the first
point, so FAST-LIO stamps odometry/TF with the message's host-clock header
stamp. With the lidar-clock timestamps the odom TF lands in a different time
domain and Nav2/RTAB-Map will not find it.

## What the primitive does

`start.sh` runs `ros2 launch fork_fast_lio mapping.launch.py` with
`config/robosense_airy_s10.yaml`, then `lynx_fastlio/main.py`:

```
/airy_201/points + /airy_201/imu
            │
        FAST-LIO ──> /Odometry   (world -> IMU body)
            │
      this wrapper ──> /fastlio/odom + TF   (odom -> base_link)
```

The pinned fork does not broadcast its own TF (`tf_br->sendTransform` is
commented out). If you swap in a port that does, either disable that TF or make
sure its child frame is not `imu_link`, which `primitive-lynx_imu` already
attaches to `base_link`.

FAST-LIO's world frame is the initial IMU frame. Because the Airy is mounted
vertically, that frame is tilted; the wrapper therefore re-expresses the pose
in the initial `base_link` frame (gravity aligned, base at the origin at t=0):

```
T_odom_base(t) = inverse(T_imu_base) * T_world_imu(t) * T_imu_base
T_imu_base     = T_imu_lidar * inverse(T_base_link_airy)
```

A plain right-multiplication by `T_imu_base` would only relabel the child frame
and leave the whole odom frame tilted, which breaks Nav2's planar costmaps and
RTAB-Map's 3-DoF grid. The FAST-LIO LiDAR->IMU extrinsic lives in
`config/robosense_airy_s10.yaml`; the Airy mounting in
`config/s10_mount.yaml` is the same transform `primitive-lynx_lidar` publishes
as `base_link -> airy_201` (vertical mount, `roll=pi`, `pitch=-pi/2`).

## Manifest wiring

```yaml
primitive:
  - name: lynx_quadruped
    config:
      odom_source_topic: /fastlio/odom   # republished as /odom for nav2
  - name: lynx_fastlio
    path: ./primitives/primitive-lynx_fastlio

service:
  - name: mapping
    config:
      sensor_providers:
        lidar3d: lynx_lidar
        odom: lynx_fastlio       # rtabmap no longer runs icp_odometry
      # imu binding removed: FAST-LIO consumes /airy_201/imu directly
```

## Tuning

`config/robosense_airy_s10.yaml` carries the FAST-LIO parameters
(`filter_size_surf/map`, `point_filter_num`, `blind`, extrinsics). The front
Airy is mounted vertically, so its scan is a vertical disc; `fov_degree` stays
360. If odometry drifts, recalibrate `mapping.extrinsic_T/R` (LI-Init) and
update `base_to_lidar_*` if the lidar mount changes.

The pinned fork publishes `/Odometry` at the lidar rate (~10 Hz), so the
forwarded chassis `/odom` is 10 Hz as well.

## Latency: why the fork is patched

The pinned fork appends every lidar scan to an unbounded queue and processes
roughly one scan per lidar period. Scans buffered while the IMU initialises
(~5 s on the S10) therefore never drain: `/Odometry` and the
`odom -> base_link` TF stay about 5 s behind the live cloud for the rest of
the session. RTAB-Map looks the pose up at the scan timestamp and its 1.5 s
`wait_for_transform` cannot cover that gap, so every scan is dropped and no
map is built.

`patches/lidar-queue-bound.patch` makes the sync step keep only the newest
scan whenever more than three are queued (and re-select it, so an in-flight
older scan is discarded). The TF then trails the cloud by a few hundred
milliseconds, well within RTAB-Map's wait window, even when the node starts
before the IMU stream is up.

The same patch also subscribes `/airy_201/imu` with `SensorDataQoS()`
(BEST_EFFORT) instead of RELIABLE. A RELIABLE reader on the 200 Hz stream
triggers Fast DDS flow control, and a stalled reader — such as a leftover node
from an earlier boot that was never reaped — throttles the driver's writer for
every reader. FAST-LIO then never initialises and `/Odometry` never appears,
which soma reports as `no nav_msgs/Odometry on /Odometry within 60.0s`. With
BEST_EFFORT, dropped samples are harmless to the EKF and one stalled reader
cannot silence the stream.

`scripts/start.sh` additionally kills stale `fastlio_mapping`, `ros2 launch
fork_fast_lio` and `python3 -m lynx_fastlio.main` processes before starting a
new one, because rbnx can tear a package down without giving the trap in
`start.sh` a chance to run. `primitive-lynx_lidar` and `primitive-lynx_quadruped`
do the same for their driver and domain bridge. Re-run
`scripts/setup_fastlio.sh` after updating the patch.

## Divergence guard

FAST-LIO can lose scan matching (`No Effective Points!` in its log). From then
on the EKF integrates IMU alone and the pose runs away to kilometres within
seconds; Nav2 then drives on a costmap whose walls are nowhere near the robot.
The wrapper checks every sample against the robot's physical limits
(`max_speed_mps` 2.0, `max_jump_m` 0.6, `max_yaw_rate_rps` 4.0, overridable via
the manifest config) and, once a sample is impossible:

* logs `FAST-LIO odometry diverged: jumped ...` once,
* stops publishing `/fastlio/odom` and the `odom -> base_link` TF, so Nav2 and
  RTAB-Map fail safe instead of following garbage.

FAST-LIO does not recover from this state by itself; restart the stack. The
guard only limits the damage — the underlying cause (a vertically mounted lidar
losing geometric constraints) still needs a more robust odometry source if the
divergence repeats.

## Troubleshooting

- `fork_fast_lio not found`: run `scripts/setup_fastlio.sh`, or export
  `FASTLIO_WS` to the colcon workspace that has `install/setup.bash`.
- Mapping previews the cloud but never builds a map:
  `ros2 topic echo /Odometry --once` and compare its stamp with
  `date +%s.%N`. If the odometry timestamp lags by seconds, the queue-bound
  patch is missing — re-run `scripts/setup_fastlio.sh`.
- `Lookup would require extrapolation into the past ... earliest data is at
  <future time>` in the mapping log: another publisher is writing
  `base_link`/`odom` TF on our DDS domain with a skewed clock. The deployment
  runs on a private `ROS_DOMAIN_ID` (manifest env) for exactly this reason;
  check that the manifest env was applied.
- FAST-LIO waits forever for IMU (`no nav_msgs/Odometry on /Odometry within
  60.0s` and the node log stops after `Node init finished.`): the fork
  subscribes `/airy_201/imu` BEST_EFFORT via the patch, so this is no longer a
  QoS mismatch. Check that `rslidar_sdk` publishes `/airy_201/imu` at ~200 Hz,
  and that no leftover `fastlio_mapping`/`rslidar_sdk_node` from an earlier
  boot is still running (`pgrep -af "fastlio_mapping|rslidar_sdk_node"`).
  `scripts/start.sh` kills stale FAST-LIO instances itself; a stale lidar
  driver is killed by `primitive-lynx_lidar`'s start script.
- Nav2 complains about missing `odom -> base_link`: check
  `ros2 topic hz /Odometry` and `ros2 run tf2_ros tf2_echo odom base_link`.
  The watchdog logs a warning when `/Odometry` stops.
