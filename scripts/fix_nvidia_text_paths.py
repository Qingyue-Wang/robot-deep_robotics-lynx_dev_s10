#!/usr/bin/env python3
"""Rewrite stale /home/nvidia absolute paths inside the rbnx build caches.

The deployment was copied from a machine whose home was /home/nvidia; the
symlink targets were already rebased by scripts/fix_nvidia_paths.sh, but the
*generated text files* (colcon env hooks, venv shebangs, colcon build logs,
sdk env files) still contain the old absolute paths.  Those stale paths end up
on PYTHONPATH/LD_LIBRARY_PATH at boot, which breaks `import piper` and breaks
the realsense plugin ABI.

Usage:
  python3 scripts/fix_nvidia_text_paths.py           # dry run, shows counts
  python3 scripts/fix_nvidia_text_paths.py --apply   # rewrite in place
"""
import argparse
import os
import sys
from pathlib import Path

ROOT = Path("/home/dog/robot-deep_robotics-lynx_s10")
CACHES = ROOT / "rbnx-boot" / "cache"
SKIP_DIR_NAMES = {".git", "logs", "__pycache__"}

# (old, new) replacements applied in order.  Only rules whose new target
# exists on this machine are enabled.
RULES = [
    ("/home/nvidia/robot-deep_robotics-lynx_s10", "/home/dog/robot-deep_robotics-lynx_s10"),
    ("/home/nvidia/.robonix", "/home/dog/.robonix"),
    ("/home/nvidia/.ros/", "/home/dog/.ros/"),
    ("/home/nvidia/ros2_ws", "/home/dog/ros2_ws"),
    ("/home/nvidia/.local", "/home/dog/.local"),
    ("/home/nvidia/.cargo", "/home/dog/.cargo"),
    ("/home/nvidia/gsusb-build", "/home/dog/gsusb-build"),
]


def enabled_rules():
    out = []
    for old, new in RULES:
        if Path(new).exists() or new.endswith("/robot-deep_robotics-lynx_s10"):
            out.append((old.encode(), new.encode()))
    return out


def iter_files(base: Path):
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIR_NAMES]
        for fn in filenames:
            if fn.startswith("link-rebase-backup") or ".bak-" in fn:
                continue
            yield Path(dirpath) / fn


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--caches", nargs="*", default=None,
                    help="cache dir names (default: the two with stale builds)")
    args = ap.parse_args()

    rules = enabled_rules()
    print("rules:")
    for old, new in rules:
        print(f"  {old.decode()} -> {new.decode()}")

    targets = []
    if args.caches:
        for c in args.caches:
            targets.append(CACHES / c)
    else:
        targets = [
            CACHES / "primitive-agilex-piper-arm-rbnx",
            CACHES / "primitive-intel-realsense_d435i-camera-rbnx",
        ]

    changed, pyc_stale, scanned = [], [], 0
    for base in targets:
        if not base.is_dir():
            print(f"! missing cache: {base}")
            continue
        for path in iter_files(base):
            try:
                data = path.read_bytes()
            except OSError:
                continue
            scanned += 1
            if b"/home/nvidia" not in data:
                continue
            if path.suffix == ".pyc":
                pyc_stale.append(path)
                continue
            if b"\x00" in data:          # binary, leave alone
                continue
            new = data
            for old, repl in rules:
                if old in new:
                    new = new.replace(old, repl)
            if new == data:
                continue
            changed.append(path)
            if args.apply:
                tmp = path.with_name(path.name + ".tmpfix")
                tmp.write_bytes(new)
                os.chmod(tmp, path.stat().st_mode & 0o7777)
                os.replace(tmp, path)

    print(f"\nscanned files: {scanned}")
    print(f"text files to rewrite: {len(changed)}")
    print(f"stale .pyc to delete: {len(pyc_stale)}")
    for p in changed[:15]:
        print("  txt", p.relative_to(ROOT))
    if len(changed) > 15:
        print(f"  ... and {len(changed) - 15} more")

    if args.apply:
        for p in pyc_stale:
            try:
                p.unlink()
            except OSError:
                pass
        print("\nAPPLIED.")
    else:
        print("\nDRY RUN - re-run with --apply to write.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
