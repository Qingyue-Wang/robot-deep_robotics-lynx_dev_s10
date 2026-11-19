#!/usr/bin/env bash
# 启动前清场：
#   1) 杀掉上一轮残留的本部署进程（按 cwd / cmdline 认，绝不按通用 ROS 名误杀）；
#   2) 确认没有存活进程后，清掉死掉的 Fast DDS 参与者留在 /dev/shm 里的段与信号量
#      （不清的话新节点建 DDS 参与者会卡住几十秒，provider 30 s 超时启动失败）；
#   3) 删掉 codegen 目录里的 __pycache__ —— 它们是 provider 运行时 import 生成模块
#      写下的副产物，会让 rbnx 误判「构建输入比 .rbnx-built 标记新」，于是每次
#      ./start.sh 都 inline 重建 depth_grasp / grasp_geometry。
set -uo pipefail

DEPLOY_DIR="${DEPLOY_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

collect_pids() {
  local pid cwd cmd
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [[ "$pid" == "$$" ]] && continue
    cwd=$(readlink "$d/cwd" 2>/dev/null || true)
    [[ "$cwd" == "$DEPLOY_DIR"* ]] && { echo "$pid"; continue; }
    cmd=$({ tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null || true)
    [[ "$cmd" == *"$DEPLOY_DIR/rbnx-boot"* ]] && echo "$pid"
  done | sort -u
}

mapfile -t pids < <(collect_pids)
if ((${#pids[@]})); then
  echo "[preboot] 停止 ${#pids[@]} 个残留进程：${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    alive=()
    for p in "${pids[@]}"; do
      [[ -d "/proc/$p" ]] && alive+=("$p")
    done
    ((${#alive[@]})) || break
    sleep 0.5
  done
  if ((${#alive[@]})); then
    echo "[preboot] 强制结束：${alive[*]}"
    kill -KILL "${alive[@]}" 2>/dev/null || true
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

# codegen 目录里的 __pycache__ 会让 rbnx 把「运行时的 .pyc 写入」当成构建输入变化。
n_cppyc=0
for pkg in "$DEPLOY_DIR"/rbnx-boot/cache/*; do
  d="$pkg/rbnx-build/codegen"
  [[ -d "$d" ]] || continue
  while IFS= read -r -d '' cache_dir; do
    rm -rf "$cache_dir" && n_cppyc=$((n_cppyc + 1))
  done < <(find "$d" -type d -name __pycache__ -print0 2>/dev/null)
done
if ((n_cppyc > 0)); then
  echo "[preboot] 清理了 ${n_cppyc} 个 codegen __pycache__（避免误判需要 inline build）"
fi
# 删 __pycache__ 会刷新父目录 mtime，同样会被 rbnx 当成「输入变了」；这里只把
# 「新输入落在 rbnx-build/ 下」的噪声标记刷掉，capabilities/ 真改了仍然会重建。
if [[ -x "$DEPLOY_DIR/scripts/build_status.sh" ]]; then
  bash "$DEPLOY_DIR/scripts/build_status.sh" --settle | sed -n '2,99p'
fi
