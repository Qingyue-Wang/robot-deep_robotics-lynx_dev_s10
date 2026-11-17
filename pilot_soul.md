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

## 导航：朝向一律「先转正、再走」

- **不要自己把 yaw 换算成四元数**（LLM 手算 `z=sin(yaw/2), w=cos(yaw/2)` 极易出错：
  2026-11-16 16:03 发成 yaw 2.70 rad 而正文写 1.79 rad）。导航目标的四元数**永远**写
  `w=1.0, z=0`（单位四元数）——服务端把它解释为「保持当前朝向」，只负责把车开到点，
  不改变朝向。
- **目标带 yaw 时，先转正再出发**（例如「前往 pose=(1.86, 3.99, 1.81)」里的 1.81 是
  **弧度** → 103.7°）：
  1. `mapping.map_get_pose` 读当前 `theta`（弧度）；
  2. Δ = 目标 yaw − theta，折到 ±180°；
  3. `|Δ| > 5°` 就 `chassis_move {"command": {"rotate_deg": Δ}}`（正=逆时针/左转），
     看返回的 `actual_rotate_deg`；
  4. 再 `map_get_pose` 复核，`|差| ≤ 5°` 才继续；
  5. 然后 `navigate`（单位四元数）开到点。出发前已经转正，到达时朝向就被保持住了。
- 顺序为什么是「先转后走」：到点再转，一旦忘了转就会带着错朝向去抓（2026-11-16 16:42
  那趟：yaw 差 88° 直接趴下，腕上相机根本看不到萝卜）。先转后走即使漏掉复核，朝向也是对的。
- 到点后仍要读一次 `map_get_pose` 复核位置与朝向（导航 detail 里也有 `yaw_error`）；
  若中途被挡住、退让过，发现朝向偏了就**先转正再**继续。
- **朝向没核对通过之前，禁止 `chassis_posture("sit")` / `depth_grasp.observe` / `pick_pick`**。
- **后退/前进用 `chassis_move` 的 `forward_m`（odom 闭环）**：按实测位移停在 ±2 cm 内，
  被挡住（1 s 内位移 < 1 cm）会返回 `{"status":"error","error":"blocked: …"}` 而不是照走
  满时长。看到这个 error 就先停下来报告，不要连续盲退。
- 导航失败（`Failed to make progress` / `ROTATION GUARD TRIPPED`）时，允许「先后退拉开
  距离、再朝目标方向转向、然后重试导航」，但每次后退都要用闭环 `forward_m` 并且**只退
  必要的距离**（0.2–0.4 m 量级），退完先读 `map_get_pose` 确认没退进墙里。
