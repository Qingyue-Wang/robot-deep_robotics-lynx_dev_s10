#!/usr/bin/env bash
# 启动前清场：
#   1) 杀掉上一轮残留的本部署「栈进程」（严格按 cmdline / cwd 认，绝不误杀交互 shell）；
#   2) 确认没有存活进程后，清掉死掉的 Fast DDS 参与者留在 /dev/shm 里的段与信号量
#      （不清的话新节点建 DDS 参与者会卡住几十秒，provider 30 s 超时启动失败）；
#   3) 删掉 codegen 目录里的 __pycache__ —— 它们是 provider 运行时 import 生成模块
#      写下的副产物，会让 rbnx 误判「构建输入比 .rbnx-built 标记新」，于是每次
#      ./start.sh 都 inline 重建 depth_grasp / grasp_geometry。
#
# 用法：bash scripts/preboot_clean.sh [--dry-run]
set -uo pipefail

DEPLOY_DIR="${DEPLOY_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DRY_RUN=0
[[ "${1:-}" == "--dry-run" ]] && DRY_RUN=1

# 栈进程的特征：命令行里必须出现这些串之一（都在部署的 rbnx-boot 之下，或 rbnx 自身）。
STACK_PATTERNS=(
  "$DEPLOY_DIR/rbnx-boot"
  "rbnx boot"
  "robonix-atlas"
  "robonix-executor"
  "robonix-soma"
  "robonix-pilot"
  "robonix-liaison"
)
# 绝不杀的进程名（交互 shell / 登录 / 系统服务 / 我们的调用方）。
NEVER_KILL_COMM=(
  bash sh dash zsh fish sshd ssh login agetty sudo su systemd systemd-logind
  dbus-daemon tmux screen init S99*
)

ancestors_of_self() {
  local pid=$$
  while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" ]]; do
    echo "$pid"
    pid=$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null || true)
  done
  echo 1
}

in_list() { local needle=$1; shift; local x; for x in "$@"; do [[ "$x" == "$needle" ]] && return 0; done; return 1; }

collect_pids() {
  local d pid comm cmd cwd
  local -a anc
  mapfile -t anc < <(ancestors_of_self)
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [[ "$pid" == "$$" ]] && continue
    in_list "$pid" "${anc[@]}" && continue          # 自己和祖先（含 start.sh、shell）一律不动
    comm=$({ cat "$d/comm"; } 2>/dev/null || true)
    in_list "$comm" "${NEVER_KILL_COMM[@]}" && continue
    cmd=$({ tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null || true)
    [[ -z "$cmd" ]] && continue                     # 内核线程 / 已退出
    cwd=$(readlink "$d/cwd" 2>/dev/null || true)

    local hit=0 p
    for p in "${STACK_PATTERNS[@]}"; do
      [[ "$cmd" == *"$p"* ]] && { hit=1; break; }
    done
    # 栈的子进程（ros2 launch 起的节点等）cwd 在 rbnx-boot 之下；登录 shell 在部署根，不会命中
    [[ $hit -eq 0 && "$cwd" == "$DEPLOY_DIR/rbnx-boot"* ]] && hit=1
    (( hit )) && echo "$pid|${cmd:0:90}"
  done | sort -u
}

mapfile -t hits < <(collect_pids)
if ((${#hits[@]})); then
  echo "[preboot] 发现 ${#hits[@]} 个本部署残留栈进程："
  for h in "${hits[@]}"; do echo "[preboot]   ${h%%|*}  ${h#*|}"; done
  if (( DRY_RUN )); then
    echo "[preboot] --dry-run：不杀进程、不清 /dev/shm"
    exit 0
  fi
  pids=()
  for h in "${hits[@]}"; do pids+=("${h%%|*}"); done
  kill -TERM "${pids[@]}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    alive=()
    for p in "${pids[@]}"; do [[ -d "/proc/$p" ]] && alive+=("$p"); done
    ((${#alive[@]})) || break
    sleep 0.5
  done
  if ((${#alive[@]})); then
    echo "[preboot] 强制结束：${alive[*]}"
    kill -KILL "${alive[@]}" 2>/dev/null || true
    sleep 1
  fi
else
  echo "[preboot] 没有本部署的残留栈进程"
fi

if (( DRY_RUN )); then
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
# rbnx 会把「构建产物目录」也算成输入、且按秒比较，导致 codegen 包每次启动都判
# 「not built」。契约真变了就显式重建，然后把标记推到严格新于产物（详见该脚本）。
if [[ -x "$DEPLOY_DIR/scripts/prebuild_codegen.sh" ]]; then
  bash "$DEPLOY_DIR/scripts/prebuild_codegen.sh"
fi
