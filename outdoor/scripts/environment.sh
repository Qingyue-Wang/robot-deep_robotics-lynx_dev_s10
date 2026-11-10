#!/usr/bin/env bash
# Source before native rbnx build/boot; scope the framework to this deployment.
OUTDOOR_DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case $- in *u*) _outdoor_had_nounset=1;; *) _outdoor_had_nounset=0;; esac
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export DRDDS_SETUP="${DRDDS_SETUP:-$HOME/drdds_build_ws/install/setup.bash}"
export RSLIDAR_WS="${RSLIDAR_WS:-$HOME/rslidar_ws}"
source "$ROS_SETUP" || return 1
source "$DRDDS_SETUP" || return 1
source "$RSLIDAR_WS/install/setup.bash" || return 1
export ROBONIX_HOME="$OUTDOOR_DEPLOY_ROOT/.robonix"
export ROBONIX_SOURCE_PATH="$OUTDOOR_DEPLOY_ROOT/framework/robonix"
export OUTDOOR_DEPLOY_ROOT
export PATH="$OUTDOOR_DEPLOY_ROOT/.venv/bin:$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
export PYTHONPATH="$OUTDOOR_DEPLOY_ROOT/packages/service-map-outdoor-rbnx:$OUTDOOR_DEPLOY_ROOT/packages/service-navigation-outdoor-rbnx:$OUTDOOR_DEPLOY_ROOT/framework/robonix/system/scene:$OUTDOOR_DEPLOY_ROOT/framework/robonix/pylib/robonix-api:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROBONIX_ATLAS=127.0.0.1:51551
export LYNX_PYTHON="$OUTDOOR_DEPLOY_ROOT/.venv/bin/python"
export G70_PYTHON="$LYNX_PYTHON" WIT_PYTHON="$LYNX_PYTHON"
export LOCALIZATION_PYTHON="$LYNX_PYTHON" OUTDOOR_SERVICE_PYTHON="$LYNX_PYTHON"
export OUTDOOR_MAP_PYTHON="$LYNX_PYTHON" RBNX_CODEGEN_PYTHON="$LYNX_PYTHON"
export ROBONIX_DRIVER_INIT_TIMEOUT_S=180 OPENBLAS_NUM_THREADS=1
mkdir -p "$OUTDOOR_DEPLOY_ROOT/run"
if [[ -f "$OUTDOOR_DEPLOY_ROOT/config/secrets.json" ]]; then
  _outdoor_secrets="$("$LYNX_PYTHON" - "$OUTDOOR_DEPLOY_ROOT/config/secrets.json" <<'PY'
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
