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

## 导航：目标朝向用 `position.z` 传，别自己算四元数

- 导航目标的四元数**永远**写 `w=1.0, z=0`；**目标朝向（弧度）填进 `goal.pose.position.z`**。
  2D 导航本来不用 z，服务端把它当目标 yaw（detail 会写
  `goal_yaw=…deg (from pose.position.z)`），nav2 于是在**行驶中**就把朝向带过去，
  到点只剩几度微调——不会再出现「到点才发现朝向差几十度」。
- 例子：`前往 pose=(1.86, 3.99, 1.81)` → 调用
  `navigate({"goal": {"header": {"frame_id": "map"}, "pose": {"position": {"x": 1.86,
  "y": 3.99, "z": 1.81}, "orientation": {"w": 1.0, "z": 0.0}}}})`。
- **不要自己把 yaw 换算成 `z=sin(yaw/2), w=cos(yaw/2)`**（2026-11-16 16:03 就算错成
  2.70 rad，车转到 55° 以外的地方）。
- 到点后 `mapping.map_get_pose` 复核**一次**：位置差 ≤0.2 m、朝向 |差| ≤11.46°
  （0.20 rad，与 nav2 的 `yaw_goal_tolerance` 一致）就算「到达」，**不要为了几度
  反复微调**。只有超出这个范围才用 `chassis_move {"command": {"rotate_deg": Δ}}`
  转正，其中 **`Δ = 目标 yaw − 当前 yaw`**（度；正=左转，负=右转），别把误差原值
  直接当旋转量发出去（2026-11-19 15:47：误差 -8.1° 却发了 `rotate_deg=-8.1`，
  方向反了，越转越偏）。转完再读一次 `map_get_pose`，落在容差内就停——map 系 yaw
  本身有 ±3~5° 抖动，继续追几度只会来回摆。
- **朝向没核对通过之前，禁止 `chassis_posture("sit")` / `depth_grasp.observe` / `pick_pick`。**
- `chassis_move` 现在会**自己立起进导航模式**（以前在趴姿下调会报
  `robot not in RL nav mode`）：趴着时直接调 `forward_m` / `rotate_deg` 就行，
  不需要先手动 `posture("stand")`。
- **后退/前进用 `chassis_move` 的 `forward_m`（odom 闭环）**：按实测位移停在 ±2 cm 内，
  被挡住（1 s 内位移 < 1 cm）会返回 `{"status":"error","error":"blocked: …"}` 而不是照走
  满时长。看到这个 error 就先停下来报告，不要连续盲退。
- 导航失败（`Failed to make progress` / `ROTATION GUARD TRIPPED`）时，允许「先后退拉开
  距离、再朝目标方向转向、然后重试导航」，但每次后退都要用闭环 `forward_m` 并且**只退
  必要的距离**（0.2–0.4 m 量级），退完先读 `map_get_pose` 确认没退进墙里。
