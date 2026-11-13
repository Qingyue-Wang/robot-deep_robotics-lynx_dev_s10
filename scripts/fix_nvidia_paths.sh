#!/usr/bin/env bash
# Rebase absolute symlinks/egg-links inside the rbnx build cache that were
# created while the deployment lived under /home/nvidia.
# Reversible: every touched link's old target is written to a .tsv backup.
set -uo pipefail

ROOT=/home/dog/robot-deep_robotics-lynx_s10
CACHE="$ROOT/rbnx-boot/cache"
OLD=/home/nvidia
NEW=/home/dog
TS=$(date +%Y%m%d_%H%M%S)
BK="$ROOT/rbnx-boot/link-rebase-backup-$TS.tsv"

: > "$BK"
n=0
while IFS= read -r l; do
    t=$(readlink "$l") || continue
    printf '%s\t%s\n' "$l" "$t" >> "$BK"
    ln -sfn "${t/$OLD/$NEW}" "$l"
    n=$((n+1))
done < <(find "$CACHE/primitive-agilex-piper-arm-rbnx" \
              "$CACHE/primitive-intel-realsense_d435i-camera-rbnx" \
              -type l -lname "$OLD/*" 2>/dev/null)
echo "rebased symlinks: $n   backup: $BK"

# Python editable install pointer used by the piper console scripts.
EGG="$CACHE/primitive-agilex-piper-arm-rbnx/rbnx-build/ws/install/piper/lib/python3.10/site-packages/piper.egg-link"
if [ -f "$EGG" ] && grep -q "$OLD" "$EGG"; then
    cp -a "$EGG" "$EGG.bak-$TS"
    sed -i "s#$OLD#$NEW#g" "$EGG"
    echo "rewrote $EGG"
fi

echo "--- remaining dangling symlinks in both caches ---"
dangling=0
while IFS= read -r l; do
    [ -e "$l" ] || { echo "  DANGLING: $l -> $(readlink "$l")"; dangling=$((dangling+1)); }
done < <(find "$CACHE/primitive-agilex-piper-arm-rbnx" \
              "$CACHE/primitive-intel-realsense_d435i-camera-rbnx" \
              -type l 2>/dev/null)
echo "dangling total: $dangling"
