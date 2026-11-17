#!/usr/bin/env bash
# 启动前的残留清理 —— 在 `start.sh` 调完 `stop.sh` 之后执行。
#
# 为什么需要它：`rbnx boot` 在某个 provider 失败后会退出，但它起的 ROS 子进程
# 有时会卡在「活着但不发布」的状态（实测 realsense2_camera_node 就是这样），
# 同时被强杀的进程会在 /dev/shm 留下 `fastrtps_*` 孤儿段。下一次启动时新节点
# 创建 DDS 参与者会卡在这些段的锁上，于是又出现
#   realsense_camera ERROR; no Image ... within 30.0s
#   piper_ctl        ERROR; no JointState ... within 30.0s
# 的死循环。stop.sh 只按 cwd 匹配，卡住的节点 cwd 常常不是 cache 目录，漏网。
#
# 这里的判据：只认「命令行或 cwd 属于本部署」的进程（绝不用通用 ROS 名字，
# 免得误杀同主机上的其它部署），清完再删孤儿共享内存段；只要还有本部署的
# ROS 进程活着就不删段。
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_ROOT="$DEPLOY_DIR/rbnx-boot"
SELF_PID=$$

collect_pids() {
  local pid cwd cmd
  while read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ "$pid" == "$SELF_PID" || "$pid" == "$PPID" ]] && continue
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    cmd="$( { tr '\0' ' ' < "/proc/$pid/cmdline"; } 2>/dev/null || true)"
    if [[ "$cwd" == "$CACHE_ROOT" || "$cwd" == "$CACHE_ROOT/"* ]] ||
       [[ "$cmd" == *"$CACHE_ROOT"* ]]; then
      printf '%s\n' "$pid"
    fi
  done < <(ps -u "$(id -u)" -o pid=)
}

mapfile -t pids < <(collect_pids)
if ((${#pids[@]})); then
  echo "[preboot] 清理本部署残留进程: ${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null
  for _ in {1..25}; do
    alive=()
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive+=("$pid"); done
    ((${#alive[@]} == 0)) && break
    sleep 0.2
  done
  if ((${#alive[@]})); then
    echo "[preboot] 强制结束: ${alive[*]}"
    kill -KILL "${alive[@]}" 2>/dev/null
    sleep 1
  fi
fi

# 还有本部署的进程在跑就别动 /dev/shm（那是活着的参与者要用的）
mapfile -t still < <(collect_pids)
if ((${#still[@]})); then
  echo "[preboot] 仍有本部署进程存活（${still[*]}），跳过 /dev/shm 清理"
  exit 0
fi

n_seg=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true)
n_sem=$(ls /dev/shm 2>/dev/null | grep -c '^sem\.fastrtps_' || true)
if ((${n_seg:-0} + ${n_sem:-0} > 0)); then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  echo "[preboot] 清理了 ${n_seg:-0} 个 fastrtps 共享内存段、${n_sem:-0} 个信号量"
else
  echo "[preboot] /dev/shm 干净"
fi
