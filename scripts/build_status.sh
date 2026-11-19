#!/usr/bin/env bash
# 显示每个包的「构建标记 vs 代码生成输入」，回答「./start.sh 为什么又 building inline」。
#
#   bash scripts/build_status.sh            # 只报告
#   bash scripts/build_status.sh --settle   # 只把「构建产物噪声」造成的过期标记刷掉
#   bash scripts/build_status.sh --touch    # 无条件刷掉所有过期标记
#
# 背景：`rbnx boot` 只对「纯 codegen 包」做输入新鲜度校验——rbnx-build/ 里只有
# codegen/、proto-staging/、logs/ 的包（当前是 depth_grasp 与 grasp_geometry）。
# 它们的产物是生成的 Python 桩，必须和 capabilities/** 对齐，所以会被 inline 重建。
#
# 两个噪声源会让它误判：
#   * provider 运行时 import 生成模块 → 写 __pycache__/*.pyc 到 codegen/ 里；
#   * 删这些 .pyc 又会刷新父目录 mtime。
# 这两种都只发生在 rbnx-build/ 里，与契约无关。`--settle` 只处理这一类（新输入
# 落在 rbnx-build/ 下），capabilities/** 或 package_manifest.yaml 真变了则不动，
# 保证该重建的时候仍然重建。
set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="$DEPLOY_DIR/rbnx-boot/cache"
MODE=report
case "${1:-}" in
  --touch)  MODE=touch ;;
  --settle) MODE=settle ;;
  "")       MODE=report ;;
  *) echo "用法: $0 [--settle|--touch]" >&2; exit 2 ;;
esac

is_codegen_only() {
  local pkg=$1 extra
  extra=$(find "$pkg/rbnx-build" -mindepth 1 -maxdepth 1 -type d \
            ! -name codegen ! -name proto-staging ! -name logs 2>/dev/null)
  [[ -z "$extra" ]]
}

shopt -s nullglob
stale=0
codegen_count=0

printf '%-44s %-17s %-17s %s\n' PACKAGE MARKER NEWEST_INPUT STATE
for pkg in "$CACHE_DIR"/*; do
  marker="$pkg/rbnx-build/.rbnx-built"
  [[ -f "$marker" ]] || continue
  is_codegen_only "$pkg" || continue
  codegen_count=$((codegen_count + 1))

  marker_s=$(stat -c %Y "$marker")
  newest=0
  newest_path="(none)"
  newest_pyc=0
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    s=$(stat -c %Y "$f" 2>/dev/null) || continue
    case "$f" in
      *__pycache__*|*.pyc) (( s > newest_pyc )) && newest_pyc=$s; continue ;;
    esac
    if (( s > newest )); then newest=$s; newest_path="${f#$pkg/}"; fi
  done < <(
    find "$pkg/capabilities" -type f 2>/dev/null
    printf '%s\n' "$pkg/package_manifest.yaml"
    # rbnx 的判定精确到「秒」，并且把构建产物目录本身也算作输入：构建结束时
    # 标记与这些目录常落在同一秒里，于是每次重启都判过期。这里一并纳入统计。
    printf '%s\n' "$pkg/rbnx-build" "$pkg/rbnx-build/codegen" \
      "$pkg/rbnx-build/proto-staging" "$pkg/rbnx-build/logs"
    find "$pkg/rbnx-build/codegen" -maxdepth 2 -not -path '*__pycache__*' 2>/dev/null
    find "$pkg/rbnx-build/proto-staging" -maxdepth 1 2>/dev/null
  )

  # rbnx 把「同一秒」也算过期（构建产物目录与标记常同秒），这里同样按 >= 判
  if (( newest >= marker_s )); then
    case "${newest_path%%/*}" in
      rbnx-build) noise=1 ;;   # 构建产物（含 rbnx-build 目录本身）造成的噪声
      *)          noise=0 ;;
    esac
    stale=$((stale + 1))
    state="STALE ($newest_path newer)"
    if [[ "$MODE" == touch ]] || { [[ "$MODE" == settle ]] && (( noise )); }; then
      # 往后推 2 秒，确保严格新于构建产物目录（rbnx 按秒比较）
      touch -d "$(date -d '+2 seconds' '+%Y-%m-%d %H:%M:%S')" "$marker"
      state="FIXED (was stale: $newest_path)"
    fi
  else
    state="ok"
    (( newest_pyc > marker_s )) && state="ok（另有 .pyc 写入，已忽略）"
  fi
  printf '%-44s %-17s %-17s %s\n' "$(basename "$pkg")" \
    "$(date -d "@$marker_s" '+%m-%d %H:%M:%S')" \
    "$(date -d "@$newest" '+%m-%d %H:%M:%S')" "$state"
done

echo
echo "纯 codegen 包 ${codegen_count} 个（其余包不会 inline 重建，不参与此判定）。"
if (( stale == 0 )); then
  echo "全部新鲜：下一次 ./start.sh 不会出现 'not built — building inline'。"
elif [[ "$MODE" == settle || "$MODE" == touch ]]; then
  echo "已刷新上面的标记；capabilities/** 与 package_manifest.yaml 真正变化时仍会重建。"
else
  echo "有 $stale 个包会被 inline 重建。噪声（rbnx-build/ 下）可用 --settle，"
  echo "确定是误报也可以用 --touch 无条件刷新。"
fi
