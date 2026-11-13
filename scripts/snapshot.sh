#!/usr/bin/env bash
# 给这套部署做一次“可回退快照”：所有相关 git 仓库提交当前改动 + 打一个同名 tag。
#
#   bash scripts/snapshot.sh                 # tag 形如 snap-20261113-1605
#   bash scripts/snapshot.sh before-lookfix  # tag 形如 snap-20261113-1605-before-lookfix
#
# 回退用 scripts/rollback.sh <tag>（会 git reset --hard，未提交的改动会丢）。
set -u

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DEPLOY_DIR" || exit 1

STAMP="$(date +%Y%m%d-%H%M)"
LABEL="${1:-}"
TAG="snap-$STAMP"
[ -n "$LABEL" ] && TAG="$TAG-$LABEL"

SNAP_DIR="$HOME/snapshots/$TAG"
mkdir -p "$SNAP_DIR"

# 相关仓库：部署根目录 + 每个 primitive/skill/service 缓存 + primitives/ 下的嵌套仓库
REPOS=("$DEPLOY_DIR")
while IFS= read -r gitdir; do
  REPOS+=("$(dirname "$gitdir")")
done < <(find "$DEPLOY_DIR/rbnx-boot/cache" "$DEPLOY_DIR/primitives" -maxdepth 3 -name .git \
           -not -path "*/rbnx-build/*" 2>/dev/null | sort)

# 加一次不带用户身份的容错提交
GIT="git -c user.name=robodeploy -c user.email=robodeploy@localhost"

SUMMARY="$SNAP_DIR/repos.txt"
: > "$SUMMARY"
echo "snapshot tag: $TAG"
printf '%-62s %-28s %-10s %s\n' "REPO" "BRANCH" "STATE" "COMMIT"

for repo in "${REPOS[@]}"; do
  [ -d "$repo/.git" ] || continue
  rel="${repo#$DEPLOY_DIR/}"; [ "$rel" = "$repo" ] && rel="."
  branch="$($GIT -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')"
  dirty="$($GIT -C "$repo" status --porcelain 2>/dev/null | wc -l)"
  state="clean"
  if [ "$dirty" -gt 0 ]; then
    if $GIT -C "$repo" add -A >/dev/null 2>&1 \
       && $GIT -C "$repo" commit -q -m "snapshot $TAG: $rel ($dirty files)" >/dev/null 2>&1; then
      state="committed:$dirty"
    else
      state="COMMIT-FAILED:$dirty"
    fi
  fi
  $GIT -C "$repo" tag -f "$TAG" >/dev/null 2>&1 || state="$state/TAG-FAILED"
  commit="$($GIT -C "$repo" rev-parse --short HEAD 2>/dev/null || echo '?')"
  printf '%-62s %-28s %-10s %s\n' "$rel" "$branch" "$state" "$commit"
  printf '%s\t%s\t%s\t%s\n' "$rel" "$branch" "$commit" "$TAG" >> "$SUMMARY"
done

# 另外把“不在 git 里但回退时要知道”的东西抄一份：清单与关键配置
cp -a "$DEPLOY_DIR/robonix_manifest.nx.yaml" "$SNAP_DIR/" 2>/dev/null
mkdir -p "$SNAP_DIR/config"
cp -a "$DEPLOY_DIR"/config/*.yaml "$DEPLOY_DIR"/config/*.yml "$SNAP_DIR/config/" 2>/dev/null
cp -a "$DEPLOY_DIR/pilot_soul.md" "$SNAP_DIR/" 2>/dev/null
cp -a "$DEPLOY_DIR"/start.sh "$DEPLOY_DIR"/stop.sh "$SNAP_DIR/" 2>/dev/null

echo
echo "summary : $SUMMARY"
echo "files   : $SNAP_DIR"
echo "rollback: bash scripts/rollback.sh $TAG"
