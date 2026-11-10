"""Fetch pinned dependencies in place; never copy deployments or start hardware."""

import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def prepare(root=ROOT):
    items = json.loads((root / "sources.json").read_text())
    # Reject local edits before updating any dependency.
    for item in items:
        folder = root / item["path"]
        if folder.exists():
            if not (folder / ".git").exists():
                raise ValueError("Existing package has no Git history: " + str(folder))
            dirty = subprocess.check_output(
                ["git", "-C", str(folder), "status", "--porcelain"], text=True
            )
            if dirty:
                raise ValueError(
                    "Preserve local package changes before updating: " + str(folder)
                )
    for item in items:
        folder = root / item["path"]
        if not folder.exists():
            folder.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                ["git", "clone", "--filter=blob:none", item["url"], str(folder)],
                check=True,
            )
        actual = subprocess.check_output(
            ["git", "-C", str(folder), "rev-parse", "HEAD"], text=True
        ).strip()
        if actual != item["revision"]:
            subprocess.run(
                ["git", "-C", str(folder), "fetch", "origin", item["revision"]],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(folder), "checkout", "--detach", item["revision"]],
                check=True,
            )
    config = root / ".robonix/config.yaml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(
        json.dumps(
            {
                "package_storage_path": str(root / ".robonix/packages"),
                "robonix_source_path": str(root / "framework/robonix"),
            },
            indent=2,
        )
    )
    base = root / "config/gnss.base.yaml"
    if not base.exists():
        shutil.copy2(root / "config/gnss.base.example.yaml", base)
        base.chmod(0o600)
    (root / "run").mkdir(exist_ok=True)
    print(
        "Dependencies prepared. Configure GNSS and source scripts/environment.sh before rbnx build."
    )


if __name__ == "__main__":
    prepare()
