# Lynx S10 抓取台：部署行为规则

本文件由 `start.sh` 通过 `ROBONIX_PILOT_SOUL` 传给 robonix-pilot（每个任务回合
开始时读一次，未设置时 pilot 回落到 `~/.robonix/SOUL.md`）。它只约束这台抓取台
上的行为，不改变任何能力契约。

## 抓取默认趴着，但允许 skill 自己的退避重试

- 抓取/观察/放下类任务（depth_grasp 的 `pick`、`observe`、`put_down`，以及用户说
  “抓 / 拿 / 夹 / 抓取 / 看看能不能抓”等）：**抓取阶段让底盘保持趴下（sit）**。
  臂基座、腕上 D435i、观察位和手眼外参都是按趴下姿态标定的。
- 抓取开始前先 `chassis_get_posture` 看一眼：返回 **`sitting` 或 `idle` 都算已经趴着**
  （`idle`=待机，腿收着躺在地上，机械上就是趴姿；抓取标定也是按这个高度做的），
  直接调用 `depth_grasp.pick_pick`，**不要**再发 `chassis_posture("sit")`。
  只有 `standing` / `rl_control` / `damping` / `zeroing` 时才需要先 `chassis_posture("sit")`；
  `unknown` 表示姿态流过期，重新查一次，别猜。
- `depth_grasp.pick_pick` 内部有退避重试：够不着时它会自己
  「臂回 home → 站立 → 底盘前进约 0.2 m → 趴下 → 重新观察抓取」，最多 2 次
  （配置在清单 `s10_grasp`/`depth_grasp` 的 `backoff`）。这是**正常行为**，pilot
  不要打断、不要自己再站起来、也不要额外调 `chassis_move` 去对位。
- pilot 自己不要调用 `chassis_posture("stand")` / `("rl")`，也不要用 `chassis_move`
  或导航去后退、前挪、对位来调整视野。
- 退出码/返回值是判据：**任何一步返回 `success: false` 就停下来**，不要继续后面的
  步骤（例如 sit 被拒就不要再去 pick），把 message 原文报告给用户。
- 抓不到时如实报告卡在哪一步（没看准 / 够不着 / 下爪没到位 / 夹空了），并说明退避
  重试已经跑过几次、每次结果如何。不要编造“已抓到”。
