# primitive-lynx_health

Robonix device-health primitive for the DeepRobotics Lynx S10 ("山猫"). It
aggregates the S10's native drdds telemetry into `robonix/primitive/health/*`
for Vitals.

| S10 topic | type | direction |
|-----------|------|-----------|
| `/BATTERY_DATA` | `drdds/BatteryData` | in (voltage / charging / %) |
| `/cpu_status`   | `drdds/CPUStatus`   | in (per-SoC temperature) |

Exposed Robonix capabilities: `robonix/primitive/health/state`,
`robonix/primitive/health/stream`.

## Build & run

```sh
rbnx codegen -p . --mcp
bash scripts/start.sh   # sources /opt/ros/humble + prebuilt drdds
```

The `drdds` message package is prebuilt at `/home/nvidia/drdds_build_ws`;
override its install prefix with `LYNX_DRDDS_INSTALL`. See `CAPABILITY.md`.
