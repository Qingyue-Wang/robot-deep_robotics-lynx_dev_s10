# 山猫 Robonix 运行结构与 RTK／前端接入计划

核查日期：2026-10-19。开发与集成根目录：`/home/nvidia/robot-deep_robotics-lynx_s10`。

这是一份基于官方手册、山猫当前部署清单和只读检查的工作说明，不是官方手册的替代。当前整机源码基点为 `0f419d81d2b19a65d3ec919e26c9479e0d06035a`；运动驱动存在需保留的未提交小功能，本轮未修改它们。

## 1. 结论和范围

- 以山猫现有的传感器、本地定位、建图／代价地图、导航和受控运动输出为运行基础，不重新部署仿真栈，也不并行增加第二个规划器／控制器／速度发布者。
- 新增功能主要可以归为两块：**RTK 地理坐标接入**与**前端／任务应用接入**。但两块之间需要明确的坐标适配，以及任务反馈／取消／途经点语义适配，并非把两个消息口接上即可。
- RTK 驱动仍由现有开发者调试；本轮只读取 USB、串口元数据与 ROS 图，没有打开串口、改变 RTK 配置或下发机器人动作。
- 本次进程检查没有发现 Robonix 主部署的 Atlas／Executor／Soma／Pilot／Liaison、Primitive、mapping、nav2 进程。旧 state、lock、PID 与日志文件仍存在，不能用它们证明当前在线。底层厂商 DDS 数据仍可见，与上述部署是否启动是两回事。

## 2. Robonix 中的概念不能混为一条堆栈

按手册，Primitive、Service、Skill 是三类 **Provider**，不是必须逐层经过的三道关：

| 概念 | 含义 | 山猫对应 |
|---|---|---|
| Primitive | 面向硬件／设备的接入提供者 | 底盘、IMU、雷达、相机 |
| Service | 可复用算法或持续功能提供者 | mapping、nav2 |
| Skill | 面向任务、可被规划选用的复用行为 | 当前 manifest 的 skill 列表为空 |
| Contract | 接口规范／IDL、版本和交互模式 | 导航目标、导航状态、取消等接口约定 |
| Capability | Provider 对某个 Contract 的运行实现及 transport 绑定 | 某个已注册的导航提供者通过 gRPC 或 MCP 实现 navigate |
| Package | 构建、启动和交付单元 | package_manifest 描述的原语／服务／技能软件包 |

因此 Capability 不是夹在 Primitive 与 Service 之间的第四层。存在 Contract 文件、manifest 声明或源码，不等于当前已注册、激活并可调用。

框架自身还有核心组件与系统级服务的职责划分。架构图中的多项职责不等于都需要独立进程；例如 Nexus 是通信库，Scribe 是进程内日志库。不能因架构中列出 Sentinel 就认为部署已有完整统一安全监管。

## 3. 山猫当前配置了哪些模块

| 模块 | 类别／职责 | 当前部署声明 |
|---|---|---|
| rbnx | 构建、包解析、部署启动与生命周期工具 | 从整机 robonix_manifest.yaml 组织部署 |
| Atlas | 能力／Provider 注册、查询、连接信息登记 | 启用；配置 gRPC 50051 |
| Executor | 执行 RTDL 任务结构、反馈事件、轮询异步能力状态与取消 | 启用；配置 50061 |
| Soma | 读取本体描述、协调 Primitive／Skill 生命周期，汇集有限本体状态 | 启用；soma.yaml 引用 URDF／footprint 等 |
| Pilot | 根据任务进行推理／规划，产生交给 Executor 的执行方案 | 启用；确定性地图点击不必绕经模型规划 |
| Liaison | 用户任务入口，规范化任务并转交 Pilot，转发事件 | 启用；不是所有网页业务都必须走的通用 REST 网关 |
| lynx_quadruped | Primitive；底盘运动、姿态、速度输入、里程计 | 启用，接标准 Twist 与厂商 DDS |
| lynx_imu | Primitive；原始 IMU 的坐标帧／时间处理与能力暴露 | 启用 |
| lynx_lidar | Primitive；前部 Airy 点云与雷达驱动／静态变换 | 启用；实际来源依配置绑定，不按旧注释推断硬件型号 |
| lynx_camera | Primitive；当前接 RTSP RGB／snapshot | 启用 video4；这不证明 RGB-D 已经完成能力接入 |
| mapping | Service；RTAB-Map、地图／位姿、保存／加载／模式等能力 | 启用；使用外部 chassis odom、IMU、前雷达 |
| nav2 | Service；本地坐标导航、状态、取消、速度设置 | 启用；复用现有规划、代价地图与控制链 |
| Skill | 可复用任务行为 | 无已启用 Skill |

Scene、Vitals 和健康 Primitive 当前为关闭／注释状态。环境变量存在，不代表相应组件已被启用。

上表端口是配置值，不是本轮证明正在监听的端口。Provider 的业务 gRPC／MCP 地址一般动态分配，应经 Atlas 发现，不能写死旧日志端口。

## 4. 如何由 Robonix 启动

三个配置对象各司其职：

- `robonix_manifest.yaml`：整机系统组件、包实例、仓库／本地路径、实例配置。
- `soma.yaml` 与 `urdf/`：本体描述与几何／部件／能力关联；并非完整运行接口清单。
- 每个包的 `package_manifest*.yaml`：该包的构建、启动、停止与接口声明。

声明的启动流程为：

1. 准备／构建所需包。正常应先 build 再 boot；boot 仍可能补拉取或构建，因而不是只读检查命令。
2. 按顺序启动已声明系统组件：Atlas → Executor → Soma → Vitals（若启用）→ Pilot → Liaison。
3. Soma 第一阶段拉起 Primitive，等待注册，执行 INIT、ACTIVATE 并确认状态。
4. rbnx 拉起 mapping、nav2 等 Service，传入 manifest 配置，完成注册、初始化与激活。
5. Service 就绪后通知 Soma 第二阶段初始化 Skill；当前山猫没有 Skill。

注意：主工程的 `start.sh` 会先调用停止脚本，Primitive 初始化也可能切换底盘使用模式。不能为了查看环境随意执行它。本轮没有执行 start／boot。

## 5. 实际数据与命令流

### 传感器／定位数据面

- 厂商 `/ODOM` → 底盘 Primitive → `/odom`。
- 厂商 `/IMU` → IMU Primitive → `/IMU/framed`。
- 前雷达点云 → 雷达能力；mapping／nav2 经 Atlas 解析具体 ROS topic 后直接消费。
- mapping 内部运行 RTAB-Map，提供地图、地图元数据与位姿等输出。
- nav2 消费地图、里程计和雷达等观测；点云可被转换成 `/scanner/scan`。

**图像、点云、里程计等高频载荷并不经过 Atlas 转发。** Atlas 属于发现／连接登记的控制面；ROS 2 DDS 承担这里的数据面。

### 导航命令

前端任务后端 → Atlas 发现导航提供者 → 导航 Capability → ROS NavigateToPose → 现有 Nav2 规划／控制／平滑与 guard → `/cmd_vel` → 底盘 Primitive → 厂商 `/NAV_CMD`。

网页不得直接给最终速度话题或原生底盘运动口发指令，从而绕过现有受控链。现有 guard 的存在也不等于已经涵盖所有物理急停和防碰撞场景；实机验证必须确认取消、断流和遥控接管时真实停车。

### 自然语言与确定性任务

Liaison → Pilot → Executor 是自然语言任务链的一条入口。网页已有明确地图目标时，可以经薄后端直接调用导航能力，或构造确定性 Executor Plan，不必强制调用 Pilot。

Executor 可复用 RTDL 的 sequence／parallel／do 与事件流；其当前外部业务能力调用采用 MCP，生命周期使用 gRPC。不要描述为 Executor 直接向 ROS topic 发控制命令。

## 6. 已有接口能做什么，尚缺什么

已有导航能力：

- navigate：map 坐标系下的 PoseStamped → accepted、run_id、detail。
- navigate/status：指定 run_id → known、state、detail。
- navigate/cancel：指定 run_id → accepted、detail。

accepted 首先表示排队，不等于 ROS action 已经接受，更不等于机器人已移动。取消请求被接受也不等于真实停车。前端必须按 run_id 跟踪完整状态。

现有接口没有提供结构化的实时百分比／剩余距离字段；Nav2 feedback 在内部有保存，但需要有意扩展只读遥测接口才能给网页使用。状态和任务记录主要在进程内，重启会丢失；是否持久化应按任务需要决定，不能默认已有恢复执行能力。

当前封装主要是单点 NavigateToPose，没有已确认的连续途经点数组接口：

- Executor 顺序调用多个单点，可以复用现有机制，但通常意味着每点完成再进入下一点。
- 这与“中间点不停车、连续沿参考道路通过”不是同一能力。若保留该要求，应在导航 Service／任务适配中补连续经过语义，不能把全部目标快速并发提交。

mapping 已有地图管理网页及只读地图／状态接口，例如 `/api/map.png`、`/api/state`、`/api/maps`。其现有点击交互用于定位播种，不是导航选目标。该管理入口还包含加载／删除／重置等操作，不宜原样公开成无鉴权任务接口。

## 7. 本次 RTK 被动检测

观察时间：2026-10-19，本轮仅检查元数据和被动 ROS 图。

- 检测到 USB u-blox GNSS 设备 `1546:01a9`。
- `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` 指向 `/dev/ttyACM0`。
- 一个 Python 调试进程正持有该串口，本轮未打开、抢占、重置或改串口配置。
- ROS domain 0 中 `/GPS` 存在 bare-DDS 发布者，类型 `drdds/msg/NavSat`；该消息定义包含 latitude、longitude、speed、course、fixquality 等字段。
- 但尚未证明 `/GPS` 是本次新接 USB RTK 的输出，不能把它直接当作已调通的 RTK。
- 在该域的本轮检查中，标准 `/gps/fix` 及所查 FIBOCOM RTK／fusion 话题没有发布者。这个结论仅针对所查域和时刻，不等于设备没连上、驱动未安装或其他开发者调试失败。
- 尚未验证新 RTK 的固定解、差分改正时效、数据时钟或航向输出；不能由“USB 可见”直接推出“定位可用于导航”。

下一步应让 RTK 调试方给出确定的数据入口、字段／单位、WGS84／高度基准、时间戳与定位质量定义。如果已有唯一串口采集进程，接入 Primitive 优先消费其输出；不要再开第二个串口读者。

## 8. 后续新增两块功能的落点

以下目录是建议新增位置，不是已经实现的组件。

| 工作 | Robonix 位置 | 在山猫主工程中的建议落点 |
|---|---|---|
| RTK 硬件／上游数据接入 | Primitive | primitives/ 下的新 RTK 包，加入整机 primitive 声明；优先复用调试方已稳定的采集输出 |
| 地理坐标与现有地图对齐 | Service | services/ 下的坐标适配包，加入 service 声明；输出已有导航接口能消费的 map 米制目标 |
| 网页与确定性任务后端 | Application，消费已有 Capability／Executor | apps/ 下，复用既有网页代码与地图只读接口，不拉起仿真或第二套 Nav2 |
| 可复用任务意图（按需要） | Skill，可选 | 只有需要供 Pilot／Executor 复用某类任务时再增加，不必为了一个网页按钮先创建 Skill |

坐标适配必须统一处理：

1. RTK 经纬度到米制 ENU／投影坐标的 datum 与原点。
2. ENU ↔ 山猫现有 map 的旋转与平移；现有本地坐标不能直接当经纬度。
3. 天线到 base_link 的安装偏移、航向约定、时间同步及定位质量门槛。
4. 地图 ID／重建／重定位变化时，标定是否仍有效；失效就禁用地理目标下发并明确提示。
5. 前端显示与目标下发使用同一个变换；不能各算一套。
6. 保留既有 TF 发布权，不能随便再发一条冲突的 map→odom。

## 9. 建议实施顺序和边界

1. **先接只读前端**：现有地图、机器人位姿、能力发现与状态展示，不发运动目标。
2. **接本地坐标任务闭环**：明确导航提交／反馈／取消与实机停车确认；实际运动需得到相应授权与现场安全条件。
3. **接 RTK 数据和坐标适配**：先做离线对应点／航向标定，验证同一目标的地理坐标与 map 坐标一致。
4. **接地理目标与道路任务**：最后做授权下的短距到达、连续途经、断流／取消停车和地图变化失效提示。

本轮没有改动部署参数、驱动小功能或运行服务，也没有调用导航、取消、速度或姿态操作。Node 已在前一轮按授权用户级安装；本轮未重新安装依赖。

## 10. 主要依据和后续核对入口

官方手册：

- https://book.robonix.ai/architecture/namespace-and-contracts
- https://book.robonix.ai/architecture/components
- https://book.robonix.ai/architecture/runtime-communication
- https://book.robonix.ai/architecture/deployment-and-startup
- https://book.robonix.ai/components/rbnx
- https://book.robonix.ai/interface-catalog/system/soma
- https://book.robonix.ai/interface-catalog/system/executor
- https://book.robonix.ai/integration-guide/packaging-spec

机器人部署／源码：

- 主工程 robonix_manifest.yaml、soma.yaml、config/、primitives/。
- service-navigation-rbnx/nav2_wrapper/atlas_bridge.py、configuration.py、velocity_guard.py。
- service-map-rbnx/src/mapping_rbnx/atlas_bridge.py、webui.py 与 webui_static/。
- /home/nvidia/robonix/system/atlas、executor、soma、pilot、liaison。
- /home/nvidia/robonix/capabilities/lib/navigation/srv/。

手册页面有其固定源码快照，不能代替当前机器人源码／二进制验证。本轮没有通过重建和运行去证明已安装系统二进制与 checkout 完全同版；已明确区分文档定义、部署声明、源码实现和本轮在线观察。
