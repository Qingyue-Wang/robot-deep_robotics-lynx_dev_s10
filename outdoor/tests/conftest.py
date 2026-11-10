"""Use installed dependencies on the robot or sibling source packages in the workspace."""

import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
packages = ROOT / "packages"
framework = ROOT / "framework/robonix"
os.environ.setdefault(
    "OUTDOOR_PACKAGES_ROOT", str(packages if packages.exists() else WORKSPACE)
)
os.environ.setdefault(
    "OUTDOOR_FRAMEWORK_ROOT",
    str(framework if framework.exists() else WORKSPACE / "robonix"),
)
packages = Path(os.environ["OUTDOOR_PACKAGES_ROOT"])
navigation = packages / "service-navigation-outdoor-rbnx"
sys.path[:0] = [
    str(ROOT.parent / "primitives/primitive-lynx_chassis"),
    str(packages / "service-localization-outdoor-rbnx"),
    str(packages / "primitive-wit-imu-rbnx"),
    str(navigation),
    str(navigation / "rbnx-build/codegen/proto_gen"),
    str(navigation / "rbnx-build/codegen/robonix_mcp_types"),
]
