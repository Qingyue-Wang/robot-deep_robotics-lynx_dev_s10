# 云深处 Lite3 · Robonix 使用说明

面向对象：拿到一台**已配置好的** Jetson Orin 设备的人，快速上手使用 Robonix 进行**建图**和**导航**。

**Orin 设备信息：**

- 用户名：`dog`
- 密码：`123456`
- 系统：Ubuntu 22.04 / JetPack 6.2
- 已安装：ROS 2 Humble、Robonix 及全部依赖、robonix和lite3均位于/home/dog目录下

**重要提示：** Orin 已配置完成，但**你自己的机器狗还需要配置网络和运动主机参数**（见第 3 节）。

相关仓库：

- Robonix 本体：https://github.com/syswonder/robonix
- Lite3 部署包：https://github.com/syswonder/robot-deep_robotics-lite3
- 用户客户端：https://github.com/syswonder/robonix-client

---

## 0. 总览

Lite3 部署包是一个「最小 Robonix 部署」，只包含底盘控制、传感、建图、导航所需的包。三条脚本是全部入口：

| 脚本         | 作用                                 | 底层命令                                 |
| ------------ | ------------------------------------ | ---------------------------------------- |
| `./build.sh` | 拉取并构建 manifest 里的所有包       | `rbnx build -f robonix_manifest.yaml`    |
| `./start.sh` | 启动整套部署（会先跑一次 stop 清理） | `rbnx boot -f robonix_manifest.yaml`     |
| `./stop.sh`  | 干净停止，回收本部署残留进程         | `rbnx shutdown -f robonix_manifest.yaml` |

---

## 1. 登录 Orin 设备

使用以下凭据通过 SSH 登录到 Orin 设备：

```bash
ssh dog@<orin-ip-address>
# 密码：123456
```

或者直接在 Orin 设备上登录（用户名 `dog`，密码 `123456`）。

---

## 2. 部署包位置

部署包已安装在：

```bash
cd /home/dog/robot-deep_robotics-lite3
```

Robonix 及所有依赖已安装完成，可直接使用。

---

## 3. 配置你的机器狗

**重要：** 虽然 Orin 已配置完成，但你需要配置**你自己的机器狗**的网络和运动主机，才能让系统正常工作。

### 3.1 网络拓扑

确保以下连接正确：

- 激光雷达（MID-360）→ 机器狗**网口 1**
- 机器狗**网口 2** → Orin 网口
- Orin 网口 IP 已设为 `192.168.1.50`（已配置）

### 3.2 配置机器狗运动主机

你需要 SSH 进入**你的机器狗**运动主机，修改配置使其将状态数据发送到 Orin。

**步骤：**

1. 连接机器狗的 WiFi

2. SSH 连接到运动主机：

	```bash
	ssh ysc@192.168.1.120
	# 密码：<运动主机密码，询问设备方>
	```

3. 编辑配置文件：

	```bash
	cd ~/jy_exe/conf
	vim network.toml
	```

4. 修改配置内容（`ip` 改为 Orin 的 IP）：

	```toml
	ip = '192.168.1.50'
	target_port = 43897
	local_port = 43893
	```

5. **重启运动主机使配置生效**（必须）：

	```bash
	cd ~/jy_exe/scripts
	./restart.sh
	```

### 3.3 检查硬件连接

确认激光雷达和相机连接正常：

```bash
# 检查雷达连通性
ping -c1 192.168.1.3

# 检查路由
ip route get 192.168.1.3

# 检查机器狗控制口
ping -c1 192.168.1.120

# 检查 UDP 端口是否空闲（应无输出）
ss -H -lunp | awk '$5 ~ /:43897$/'
```

相机（Orbbec Gemini 330）通过 USB 连接，插上即可。

### 3.4 核对 manifest 参数（可选）

如果你的机器狗或雷达 IP 与默认值不同，需要修改：

```bash
cd /home/dog/robot-deep_robotics-lite3
vim robonix_manifest.yaml
```

在 `primitive:` 段中检查：

- **底盘**（`lite3_quadruped`）：

	```yaml
	target_ip: 192.168.1.120     # 机器狗运动主机 IP
	target_port: 43893
	local_port: 43897
	```

- **雷达**（`mid360_lidar`）：

	```yaml
	lidar_ip: 192.168.1.3      # 雷达 IP
	host_ip:  192.168.1.50       # Orin IP
	```

---

## 4. 启动系统

配置完机器狗后，即可启动 Robonix：

```bash
cd /home/dog/robot-deep_robotics-lite3
./start.sh
```

`start.sh` 会自动：

- 清理残留进程
- 检查 UDP 43897 端口
- 启动 Zenoh RMW 路由
- 启动所有服务

### 4.1 验证系统健康

在另一个终端中检查：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0

# 检查能力注册
rbnx caps

# 检查雷达数据
ros2 topic hz /scanner/cloud      # 应约 10 Hz

# 检查 IMU
ros2 topic hz /livox/imu          # 应约 200 Hz

# 检查底盘里程计
ros2 topic hz /odom               # 应约 20 Hz

# 检查 URDF 发布
ros2 topic echo /robot_description --once
```

### 4.2 停止系统

```bash
cd /home/dog/robot-deep_robotics-lite3
./stop.sh
```

---

## 5. 建图

如果你需要为自己的环境创建新地图，按以下步骤操作。

### 5.1 修改 manifest

```bash
cd /home/dog/robot-deep_robotics-lite3
vim robonix_manifest.yaml
```

找到 `service: mapping` 段，修改：

```yaml
service:
  - name: mapping
    config:
      algo: rtabmap
      map_id: my_map              # 你的地图名称
      map_mode: mapping           # 从 localization 改为 mapping
```

重启系统：

```bash
./stop.sh && ./start.sh
```

### 5.2 打开建图 Web UI

建图 Web UI 默认只监听本机 `127.0.0.1`，所以**从远程电脑访问推荐用 SSH 隧道**：

```bash
ssh -L 8091:127.0.0.1:8091 dog@<orin-ip>
# 然后浏览器打开 http://127.0.0.1:8091
```

如果要在同网段直接用 IP 访问，先在 `.env` 里加一行：

```dotenv
MAPPING_WEBUI_HOST=0.0.0.0
```

然后 `./stop.sh && ./start.sh` 重启，再打开 `http://192.168.1.50:8091`（替换为 Orin 实际 IP）。

> ⚠️ 该页面**无鉴权**，绑到 `0.0.0.0` 意味着同网段任何人都能打开、切模式、删地图；只在可信网络这样做，个人使用请优先走隧道。

### 5.3 遥控走图

使用云深处官方遥控手柄或 App 慢速遥控机器狗：

**要点：**

- 速度慢（建议 0.10–0.15 m/s）
- 转弯平缓
- 覆盖所有要导航的区域
- 尽量走出回环（回到走过的地方）
- 观察 Web UI 实时地图，栅格应连续无错层

### 5.4 保存地图

覆盖完成后：

1. 在 Web UI 点击 **Save**
2. 输入 map_id（如 `my_map`）
3. 地图保存在 `~/.robonix/maps/my_map/`

确认保存成功：

```bash
ls -la ~/.robonix/maps/my_map/
cat ~/.robonix/maps/my_map/meta.yaml
```

### 5.5 切回定位模式

编辑 manifest，改回：

```yaml
      map_id: my_map
      map_mode: localization      # 改回 localization
```

重启：

```bash
./stop.sh && ./start.sh
```

### 5.6 场景 Web UI（50107，可选）

除建图页外还有一个 Scene 场景页，默认监听所有网卡，可直接用 `http://<orin-ip>:50107` 访问，显示俯视地图、目标物体和机器人位姿。它同样无鉴权，只在可信网络使用。

 `http://<orin-ip>:50107/user` 下能看到地图选择和房间标注，保存刷新后便能通过rbnx chat要求机器狗导航到目标区域

---

## 6. 导航

导航功能在定位模式（`localization`）下使用已保存的地图。

### 6.1 验证导航就绪

```bash
source /opt/ros/humble/setup.bash

# 检查 map->odom 变换
ros2 run tf2_ros tf2_echo map odom

# 检查定位位姿
ros2 topic echo /robonix/map/pose --once

# 检查地图发布
ros2 topic echo /map --once

# 检查 2D scan
ros2 topic hz /scanner/scan

# 检查导航动作服务器
ros2 action list | grep navigate_to_pose
```

### 6.2 初始位姿校准（如需要）

如果机器人初始位姿偏差较大，在建图 Web UI（见 5.2）中：

- 点击地图上机器人的真实位置
- 系统会播种位姿估计，用于重定位
- 等待几秒让系统重定位

### 6.3 发送导航目标

**方法1：命令行直接发 goal**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, \
    orientation: {z: 0.0, w: 1.0}}}}"
```

**方法2：一句话交给 Robonix agent**

```bash
rbnx ask "导航到 x=3.0, y=2.0 的位置"
```

**方法3：开启会话Robonix agent**

```bash
#通过robonix
rbnx chat
#或是通过用户客户端：https://github.com/syswonder/robonix-client
#本地下载用户客户端后按readme配置后即在http://127.0.0.1:7860打开agent界面
```

**方法4：查看可调用的工具**

```bash
rbnx tools
```

取消导航、查询状态等能力也都在 `rbnx tools` 列出的工具里，按工具名调用即可。

---

## 7. 快速参考

### 7.1 常用命令

```bash
# 启动系统
cd /home/dog/robot-deep_robotics-lite3 && ./start.sh

# 停止系统
cd /home/dog/robot-deep_robotics-lite3 && ./stop.sh

# 查看能力列表
rbnx caps
```

### 7.2 重要文件路径

| 路径                                   | 说明         |
| -------------------------------------- | ------------ |
| `/home/dog/robot-deep_robotics-lite3/` | 部署包根目录 |
| `~/.robonix/maps/`                     | 保存的地图   |
| `~/.robonix/scene/annotations/`        | 语义标注     |
| `/opt/ros/humble/setup.bash`           | ROS 2 环境   |

### 7.3 关键网络地址

| 项目           | 地址                       |
| -------------- | -------------------------- |
| Orin IP        | `192.168.1.50`（已配置）   |
| 机器狗运动主机 | `192.168.1.120:43893`      |
| MID-360 雷达   | `192.168.1.3`              |
| 建图 Web UI    | `http://127.0.0.1:8091`（SSH 隧道，见 5.2） |
| Scene Web UI   | `http://<orin-ip>:50107`   |
| 底盘反馈端口   | UDP `43897`（必须独占）    |

---

## 8. 故障排查

### 8.1 底盘里程计无数据

检查：

- 机器狗运动主机的 `network.toml` 是否配置为 `ip = '192.168.1.50'`
- 配置修改后是否执行了 `./restart.sh`
- UDP 43897 端口是否被占用：`ss -H -lunp | awk '$5 ~ /:43897$/'`

### 8.2 雷达无数据

检查：

- 雷达连通性：`ping -c1 192.168.1.3`
- Orin IP 是否为 `192.168.1.50`
- manifest 中 `mid360_lidar` 的 IP 配置是否正确

### 8.3 导航无反应

检查：

- 是否在 `localization` 模式
- 地图是否已加载：`ros2 topic echo /map --once`
- map->odom 变换是否存在：`ros2 run tf2_ros tf2_echo map odom`
- 初始位姿是否正确（在 Web UI 中校准）

---

## 9. 更多帮助

- Robonix 官方文档：https://github.com/syswonder/robonix
- Lite3 部署包：https://github.com/syswonder/robot-deep_robotics-lite3
- 用户客户端：https://github.com/syswonder/robonix-client

如遇到问题，请查看部署包中的 `docs/` 目录下的专项文档，或联系技术支持。
