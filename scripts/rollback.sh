#!/usr/bin/env bash
# 把整套部署回退到 scripts/snapshot.sh 打的某个 tag。
#
#   bash scripts/rollback.sh snap-20261113-1605
#   bash scripts/rollback.sh            # 列出可用 tag
#
# 注意：git reset --hard 会丢弃这些仓库里“tag 之后”的改动（包括未提交的）。
set -u

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DEPLOY_DIR" || exit 1

TAG="${1:-}"
if [ -z "$TAG" ]; then
  echo "可用的快照 tag："
  git -C "$DEPLOY_DIR" tag --list 'snap-*' --sort=-creatordate | head -20
  echo
  echo "用法: bash scripts/rollback.sh <tag>"
  exit 0
fi

REPOS=("$DEPLOY_DIR")
while IFS= read -r gitdir; do
  REPOS+=("$(dirname "$gitdir")")
done < <(find "$DEPLOY_DIR/rbnx-boot/cache" "$DEPLOY_DIR/primitives" -maxdepth 3 -name .git \
           -not -path "*/rbnx-build/*" 2>/dev/null | sort)

GIT="git -c user.name=robodeploy -c user.email=robodeploy@localhost"
missing=0
for repo in "${REPOS[@]}"; do
  [ -d "$repo/.git" ] || continue
  rel="${repo#$DEPLOY_DIR/}"; [ "$rel" = "$repo" ] && rel="."
  if ! $GIT -C "$repo" rev-parse -q --verify "refs/tags/$TAG" >/dev/null 2>&1; then
    printf '%-62s %s\n' "$rel" "no tag $TAG (skipped)"
    missing=$((missing + 1))
    continue
  fi
  if $GIT -C "$repo" reset --hard "$TAG" >/dev/null 2>&1; then
    printf '%-62s -> %s %s\n' "$rel" "$TAG" "$($GIT -C "$repo" rev-parse --short HEAD)"
  else
    printf '%-62s RESET FAILED\n' "$rel"
  fi
done

echo
echo "回退完成（$missing 个仓库没有这个 tag）。建议接下来："
echo "  bash scripts/kill_stack.sh && ./start.sh"
