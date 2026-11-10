# 山猫底盘原语

由 Robonix 启动，接收通用速度和步态请求，转换成山猫 ROS 2 消息。
不做路线规划，不自动站起或切换控制姿态，不使用本体 `/ODOM`。

| 接口 | 用途 |
|---|---|
| `robonix/primitive/chassis/twist_in` | `/cmd_vel`，通用速度输入 |
| `robonix/primitive/chassis/gait_in` | `/outdoor/gait_request`，允许步态输出时声明 |
| `/outdoor/chassis_status` | 就绪状态、运动开关和反馈年龄 |
| `/MOTION_INFO` | 从本体通信域转发的实际状态，供建图与导航使用 |

`config` 的 `allow_motion`、`allow_gait` 默认均为 false；仅检查时不创建真实
速度或步态发布者。`robot_domain_id` 默认 0，必须与服务的 `ROS_DOMAIN_ID` 不同。
`robot_ip`/`robot_port` 仅用于读取使用模式，默认 `10.21.33.103:30004`。
`run_dir` 保存两个桥接进程的日志。

速度转换和本体转发保留为两个进程：前者失效时后者仍会在 0.35 秒超时后发零速度。
实际反馈必须处于 state=17、导航步态和 usage_mode=1，指令才被放行。
前进速度限制为 0～0.4 m/s，转弯速度限制为 ±0.4 rad/s。
步态仅允许 0x3002、0x3003；切换时停车并等待反馈由导航服务负责。

INIT 只验证配置和声明接口，ACTIVATE 才启动桥接；DEACTIVATE 和 SHUTDOWN 都关闭桥接。
子进程通过 Robonix SDK 管理，任一桥接退出则停止整个原语；退出发送零速度。
设备锁阻止同一用户在相同本体通信域启动第二份本原语。

离线测试：在原语目录运行 `PYTHONPATH=. python -m pytest tests/`。
`test_gait_dds.py` 需要 ROS 和 drdds，只使用本机隔离通信域 229/230。
