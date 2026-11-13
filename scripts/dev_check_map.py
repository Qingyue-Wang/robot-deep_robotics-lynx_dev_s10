import sys, os, json, time
import numpy as np
REPO = "/home/nvidia/robot-deep_robotics-lynx_s10"
CACHE = os.path.join(REPO, "rbnx-boot/cache/skill-pick-depth-grasp-rbnx/grasp_skill")
sys.path[:0] = [os.path.join(REPO, "scripts"), CACHE, os.path.join(CACHE, "calib")]
import handeye3d_web as T
import handeye_web as hw
from common import charuco

import rclpy
rclpy.init()
cfg = T.load_cfg(os.path.join(REPO, "robonix_manifest.nx.yaml"))
board = charuco.Board(12, 9, 0.015, 0.01125, "DICT_5X5_100")
b = hw.Bridge(cfg, dry_run=True, node_name="check_map")
t0 = time.time()
while time.time() - t0 < 20:
    if b.ready()["color"] and b.ready()["arm"]:
        break
    time.sleep(0.2)
print("ready", b.ready())

cal = T.Calib.__new__(T.Calib)
cal.board = board
cal.K = np.asarray(cfg["K"], dtype=float)
cal.dist = np.asarray(cfg["dist"], dtype=float) if cfg.get("dist") is not None else np.zeros(5)
cal.bridge = b
cal.log = lambda *a, **k: None
cal.image_size = (1280, 720)

det = None
for k in range(5):
    color, depth, cs, ds = b.snap_frame()
    det = cal.detect_board(color)
    print("frame", k, "ok", det["ok"], det.get("why"), det.get("n_corners"), det.get("rms_px"), det.get("view_deg"))
    if det["ok"]:
        break
    time.sleep(0.5)

d = json.load(open("/home/nvidia/.robonix/calib3d/20261112_144802/handeye.json"))
X = np.asarray(d["X"], float)
corr = d.get("correction") or {}
print("X t(mm)", np.round(X[:3, 3] * 1000, 2), "zeros", corr.get("zeros_deg"))
fl = b.flange()
print("flange t", np.round(fl[:3, 3], 4))
Tbc = fl @ X
corners = np.asarray(det["corners"]).reshape(-1, 2)
ids = np.asarray(det["ids"]).reshape(-1)
pts_b = []
good_ids = []
for (u, v), i in zip(corners, ids):
    p = b.pixel_to_cam(u, v)[0]
    if p is None:
        continue
    pts_b.append((Tbc @ np.array([p[0], p[1], p[2], 1.0]))[:3])
    good_ids.append(int(i))
pts_b = np.asarray(pts_b)
good_ids = np.asarray(good_ids)
print("mapped", len(pts_b), "z mean", np.round(pts_b[:, 2].mean(), 4), "z std", np.round(pts_b[:, 2].std(), 4))
res = []
for i in range(len(pts_b)):
    for j in range(i + 1, len(pts_b)):
        di = good_ids[i]
        dj = good_ids[j]
        if abs(di - dj) not in (1, 11):
            continue
        gi = (di // 11, di % 11)
        gj = (dj // 11, dj % 11)
        exp = 0.015 * np.hypot(gi[0] - gj[0], gi[1] - gj[1])
        res.append(np.linalg.norm(pts_b[i] - pts_b[j]) - exp)
res = np.asarray(res) * 1000
print("adjacent-pair err mm: mean %.2f std %.2f max %.2f n %d" % (res.mean(), res.std(), np.abs(res).max(), len(res)))
