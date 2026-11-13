#!/bin/bash
# 干净重启整栈：清残留 → stop → start（分离进程）
cd /home/dog/robot-deep_robotics-lynx_s10 || exit 1
bash scripts/kill_stack.sh > /tmp/restart_kill.log 2>&1
./stop.sh > /tmp/restart_stop.log 2>&1
LOG=/tmp/stack_restart_$(date +%H%M%S).log
echo "$LOG" > /tmp/last_stack_log
setsid nohup ./start.sh > "$LOG" 2>&1 < /dev/null &
echo "started -> $LOG"
