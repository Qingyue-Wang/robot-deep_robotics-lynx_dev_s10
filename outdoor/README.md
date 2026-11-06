# Lynx outdoor deployment

Run the current GNSS/IMU primitives, localization, `service-map-outdoor-rbnx`,
`service-navigation-outdoor-rbnx` and one Scene. No legacy outdoorNav checkout,
campus bundle, places file or old web server is required. Scene imports its
consumer from `outdoor_nav.scene`; global planning belongs to the map service.

## Prepare on the robot

The robot needs ROS 2 Humble, the vendor `drdds` messages, its lidar SDK, `uv`,
Robonix CLI/codegen and Nav2 including `nav2_regulated_pure_pursuit_controller`.
Review `scripts/environment.sh` for the installed ROS workspace locations.

```bash
python3 scripts/prepare.py /home/nvidia/robot-outdoor-rbnx
cd /home/nvidia/robot-outdoor-rbnx
uv venv --system-site-packages --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -e framework/robonix/pylib/robonix-api \
  -e 'packages/service-navigation-outdoor-rbnx[robonix]' \
  -e packages/service-localization-outdoor-rbnx -e packages/service-map-outdoor-rbnx \
  'grpcio-tools==1.76.0' httpx uvicorn 'starlette<1' scipy pytest
```

`prepare.py` copies deployment sources, checks out the exact revisions in
`sources.json` and writes a deployment-local Robonix configuration. It refuses
packages with local changes or missing Git history. It does not start drivers.
For the existing robot deployment, preserve its private settings and data;
update its tracked source files separately rather than overwriting packages.

Configure `config/gnss.base.yaml` with the site's NTRIP host/user/mount and put
`{"NTRIP_PASSWORD":"your-site-password"}` in `config/secrets.json` with mode 600.
Both files are ignored. Confirm serial devices, lidar network and IMU mounting;
a new map origin does not calibrate those settings or the local magnetic north.

## Start and test

```bash
./outdoor check                 # real inputs; no native motion/gait outputs
./outdoor status
./outdoor stop
./outdoor up --enable-motion    # explicit field arming; no goal or posture sent
```

Use check for manual recording and supplementation. Finish filters the data;
review connections and gait conflicts, then save a new map ID. New maps use the
first valid GNSS origin; supplementation retains the loaded map origin.

After saving, stop check and start field, then reload the saved map in Scene.
Loading binds localization to that map's origin, ID and version. Unbound
localization publishes status without odometry/TF. No manufacturer `/ODOM` is used.
The operator must select navigation mode on the remote controller and verify
position, heading and readiness before submitting a goal. Test flat ground
first, then stairs; keep the goal several metres beyond the stairs.

Scene listens only on robot loopback port 50108. Forward one local port:

```bash
ssh -p 6003 -N -L 50112:127.0.0.1:50108 nvidia@ROBOT_HOST
```

Open `http://127.0.0.1:50112/outdoor`. Indoor/outdoor share the same Scene.
Camera uses the explicitly configured local viewer; it starts no camera driver.

Gait changes stop first and require fresh success feedback within 10 seconds.
All stairs and their 1 m buffers pause local avoidance, which returns on flat
ground. Emergency stop and localization/feedback guards remain active. Arrival
requires measured stop within the existing 3 m tolerance. Real stairs remain
subject to operator field acceptance; no simultaneous stair/obstacle scenario
is covered by this deployment.

Maps/raw recordings stay in `data/outdoor_mapping`, logs in `run`, and Scene
annotations in its `rbnx-build/data`. Preserve them during upgrades. The request
journal retains at most 1024 records of 2 MiB; archive completed records when full.
