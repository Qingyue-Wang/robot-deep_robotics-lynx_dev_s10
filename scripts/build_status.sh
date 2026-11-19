#!/usr/bin/env bash
# 显示每个代码生成包（cache/*）的「构建标记 vs 输入时间」，
# 用来回答「为什么 ./start.sh 又打了 building inline」。
#
#   bash scripts/build_status.sh            # 只报告
#   bash scripts/build_status.sh --touch    # 把标记刷成当前时间（跳过 rebuild）
#
# 判定方式：rbnx 认为 `<pkg>/rbnx-build/.rbnx-built` 比构建输入旧时，
# 会在 rbnx boot 里 inline 重跑 codegen（日志 `⚠ <pkg>: not built — building inline`）。
# 构建输入 = package_manifest.yaml + capabilities/** + rbnx-build/{codegen,proto-staging}。
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="$DEPLOY_DIR/rbnx-boot/cache"
TOUCH=0
[[ "${1:-}" == "--touch" ]] && TOUCH=1

shopt -s nullglob
stale_total=0
printf '%-42s %-20s %-20s %s\n' PACKAGE MARKER NEWEST_INPUT STATE
for pkg in "$CACHE_DIR"/*; do
  marker="$pkg/rbnx-build/.rbnx-built"
  [[ -f "$marker" ]] || continue

  marker_s=$(stat -c %Y "$marker")
  newest=0
  newest_path="(none)"
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    s=$(stat -c %Y "$f" 2>/dev/null) || continue
    if (( s > newest )); then newest=$s; newest_path="${f#$pkg/}"; fi
  done < <(
    find "$pkg/capabilities" -type f 2>/dev/null
    printf '%s\n' "$pkg/package_manifest.yaml"
    find "$pkg/rbnx-build/codegen" -maxdepth 1 2>/dev/null
    find "$pkg/rbnx-build/proto-staging" -maxdepth 1 2>/dev/null
  )

  if (( newest > marker_s )); then
    state="STALE  ($newest_path newer)"
    stale_total=$((stale_total + 1))
    if (( TOUCH )); then
      touch "$marker"
      state="TOUCHED (was stale: $newest_path)"
    fi
  else
    state="ok"
  fi
  printf '%-42s %-20s %-20s %s\n' "$(basename "$pkg")" \
    "$(date -d "@$marker_s" '+%m-%d %H:%M:%S')" \
    "$(date -d "@$newest" '+%m-%d %H:%M:%S')" "$state"
done

echo
if (( stale_total == 0 )); then
  echo "所有包的构建标记都是新鲜的：下一次 ./start.sh 不会 inline 重建。"
else
  echo "有 $stale_total 个包会被 inline 重建。原因见 STATE 列里那个比标记更新的文件。"
  (( TOUCH )) || echo "确定是误报的话：bash scripts/build_status.sh --touch"
fi
