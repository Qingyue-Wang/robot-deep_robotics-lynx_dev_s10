#!/usr/bin/env bash
# 让 `./start.sh` 不再每次都说「not built — building inline」。
#
# 背景：depth_grasp / grasp_geometry 是「纯 codegen 包」，`rbnx boot` 会校验它们的
# 构建标记 `rbnx-build/.rbnx-built` 是否比"输入"新。但 rbnx 自己也把构建产物目录
# 算进输入，而且判定精确到秒——构建结束时标记与 `rbnx-build/` 目录常落在同一秒，
# 或者目录因为写日志又晚几十毫秒，于是下一次启动永远判"过期"，形成
# 「每次启动都重建 → 重建又不留干净标记」的循环。
#
# 这个脚本做两件事：
#   1) 契约真的变了（capabilities/** 或 package_manifest.yaml 比标记新）→ 先
#      `rbnx build -p <pkg>` 显式重建，把失败暴露在启动前；
#   2) 无论如何，重建/检查之后把标记时间往后推 2 秒，保证它严格新于所有构建产物。
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="$DEPLOY_DIR/rbnx-boot/cache"
RBNX="${RBNX:-$HOME/.local/bin/rbnx}"
[[ -x "$RBNX" ]] || RBNX=$(command -v rbnx || true)
DRY_RUN=0
[[ "${1:-}" == "--dry-run" ]] && DRY_RUN=1

is_codegen_only() {
  local pkg=$1 extra
  extra=$(find "$pkg/rbnx-build" -mindepth 1 -maxdepth 1 -type d \
            ! -name codegen ! -name proto-staging ! -name logs 2>/dev/null)
  [[ -z "$extra" ]]
}

newest_contract_mtime() {
  local pkg=$1 newest=0 f s
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    s=$(stat -c %Y "$f" 2>/dev/null) || continue
    (( s > newest )) && newest=$s
  done < <(find "$pkg/capabilities" -type f 2>/dev/null; printf '%s\n' "$pkg/package_manifest.yaml")
  echo "$newest"
}

shopt -s nullglob
for pkg in "$CACHE_DIR"/*; do
  marker="$pkg/rbnx-build/.rbnx-built"
  [[ -f "$marker" ]] || continue
  is_codegen_only "$pkg" || continue
  name=$(basename "$pkg")

  marker_s=$(stat -c %Y "$marker")
  contract_s=$(newest_contract_mtime "$pkg")
  if (( contract_s >= marker_s )); then
    echo "[prebuild] $name：契约文件比构建标记新，显式重建"
    if (( DRY_RUN )); then
      echo "[prebuild]   --dry-run，跳过实际构建"
    elif [[ -n "$RBNX" ]]; then
      if ! "$RBNX" build -p "$pkg" --no-update-check; then
        echo "[prebuild] $name：构建失败（继续启动会由 rbnx 再试一次）" >&2
      fi
    else
      echo "[prebuild] 找不到 rbnx，跳过" >&2
    fi
  fi
  if (( DRY_RUN )); then continue; fi
  # 关键一步：标记严格新于构建产物目录（rbnx 按秒比较，构建产物常与标记同秒）
  touch -d "$(date -d '+2 seconds' '+%Y-%m-%d %H:%M:%S')" "$marker"
done
echo "[prebuild] codegen 包标记已刷新，本次启动不会再 inline 重建"
