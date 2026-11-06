#!/usr/bin/env bash
# Source before native rbnx build/boot; scope the framework to this deployment.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case $- in *u*) _outdoor_had_nounset=1;; *) _outdoor_had_nounset=0;; esac
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/drdds_build_ws/install/setup.bash
source /home/nvidia/rslidar_ws/install/setup.bash
export ROBONIX_HOME="$ROOT/.robonix"
export OUTDOOR_DEPLOY_ROOT="$ROOT"
export PATH="$ROOT/.venv/bin:$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
export PYTHONPATH="$ROOT:$ROOT/packages/service-map-outdoor-rbnx:$ROOT/packages/service-navigation-outdoor-rbnx:$ROOT/framework/robonix/system/scene:$ROOT/framework/robonix/pylib/robonix-api:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROBONIX_ATLAS=127.0.0.1:51551
export G70_PYTHON="$ROOT/.venv/bin/python" WIT_PYTHON="$ROOT/.venv/bin/python"
export LOCALIZATION_PYTHON="$ROOT/.venv/bin/python" OUTDOOR_SERVICE_PYTHON="$ROOT/.venv/bin/python"
export OUTDOOR_MAP_PYTHON="$ROOT/.venv/bin/python" RBNX_CODEGEN_PYTHON="$ROOT/.venv/bin/python"
export ROBONIX_DRIVER_INIT_TIMEOUT_S=180
if [[ -f "$ROOT/config/secrets.json" ]]; then
  _outdoor_secrets="$("$ROOT/.venv/bin/python" - "$ROOT/config/secrets.json" <<'PY'
import json,os,shlex,sys
p=sys.argv[1]
if os.stat(p).st_mode&0o077:raise SystemExit('config/secrets.json must have mode 600')
v=json.load(open(p))
for key,value in v.items():
    if key!='NTRIP_PASSWORD' or not isinstance(value,str):raise SystemExit('Unsupported secret key/type')
    print('export '+key+'='+shlex.quote(value))
PY
)" || return 1
  eval "$_outdoor_secrets"
  unset _outdoor_secrets
fi
if [[ "$_outdoor_had_nounset" == 1 ]]; then set -u; fi
unset _outdoor_had_nounset
