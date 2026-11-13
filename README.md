# DeepRobotics Lynx S10 for Robonix

山猫整机部署仓库：选择原语、服务和现场参数，由 Robonix 负责启动与退出。
定位、建图、导航算法保留在独立 service，设备协议保留在 primitive。

执行现场导航前先读 [AGENTS.md](AGENTS.md)，尤其是底盘导航模式的准备方法。

| 入口 | 用途 |
|---|---|
| `robonix_manifest.yaml`、`robonix_manifest.nx.yaml` | 原有雷达地图导航及设备组合，具体配置按机器选择 |
| [outdoor/](outdoor/README.md) | RTK＋WIT IMU、道路录制和室外导航，默认检查模式 |
| `primitives/primitive-lynx_chassis` | 受控速度、步态与实际状态接口，不自动改变姿态 |
| `primitives/primitive-lynx_lidar` | 受管雷达驱动和点云接口 |
| [primitives/primitive-lynx_camera](primitives/primitive-lynx_camera/README.md) | 山猫相机原语，代码、测试和说明统一在本仓库维护 |
| `primitives/primitive-lynx_fastlio` | 原有雷达惯性定位接口 |
| `soma.yaml`、`urdf/` | 本体描述 |
| [tools/camera_view](tools/camera_view/README.md) | 四路相机网页与电量显示，独立于导航启动 |

## 室外部署

进入 `outdoor`，按其 README 准备锁定版本依赖和现场配置，随后使用：

```bash
source scripts/environment.sh
rbnx build -f robonix_manifest.yaml
flock -n run/boot.lock rbnx boot -f robonix_manifest.yaml --no-update-check -v
# 在另一个相同环境的终端：
rbnx caps -v
rbnx shutdown -f robonix_manifest.yaml
```

真实运动需停止检查模式，再显式选择 `robonix_manifest.field.yaml`。
不再提供 `./outdoor` Python CLI 或复制部署源码的安装方式。
地图、采集数据、日志和私密配置不纳入源码，不随重构删除。

## 原有整机配置

根目录 `build.sh`、`start.sh`、`stop.sh` 对应根目录的整机清单。
这些清单包含机器相关路径和外部设备包，使用前必须核对实际安装；当前源码中没有
`primitives/primitive-lynx_quadruped`，不能据清单存在就认定这套部署可以直接启动。
室外部署使用上述新的底盘原语，不依赖这个缺失包。

室内外先停止再切换，不同时占用底盘和雷达。室内采用 FAST-LIO 输出，室外采用 RTK＋IMU，
均不使用本体 `/ODOM`。本轮重构没有验收真实运动、动态定位精度或实际楼梯。
