# 山猫室外部署

本目录只保留 Robonix 部署清单、现场参数与依赖准备。硬件代码在 `../primitives`，
定位、建图和导航由独立服务提供。没有额外的 Python 启停 CLI。

```bash
python3 scripts/prepare.py
uv venv --system-site-packages --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -e framework/robonix/pylib/robonix-api -r requirements.txt
# 填写 config/gnss.base.yaml 和 mode 600 的 config/secrets.json。
source scripts/environment.sh
rbnx build -f robonix_manifest.yaml
flock -n run/boot.lock rbnx boot -f robonix_manifest.yaml --no-update-check -v
```

另一个已加载环境的终端执行 `rbnx caps -v` 或 `rbnx shutdown -f robonix_manifest.yaml`。
运动模式使用 `robonix_manifest.field.yaml`，先停止检查模式，两者共用同一个启动锁。
网页在机器人回环地址 `http://127.0.0.1:50108/outdoor`。

两份清单均启动山猫前摄像头（video4），Scene 的 `/cam` 直接显示原语画面，
不依赖 `tools/camera_view` 网页。需安装 `ffmpeg`，Python 环境需有 NumPy 和 Pillow。
相机断流时后台继续重连，不阻止定位、地图和 Scene 启动。
独立工具网页仍可用于切换前后左右相机。
Scene 电量显示复用独立工具的电量服务（`camera_battery_port: 8080`），
需启动 `lynx-camera.service`；每 5 秒刷新，断开或过期时会提示。

本体通信在 domain 0，室外程序在 domain 80。不使用本体 `/ODOM`。
启动不自动站起或发目标；航向质量尚未确认，动态精度与真实楼梯待现场验收。

统一文档位于 outdoorNav 工作区的 `docs/ARCHITECTURE.md`、`docs/DEPLOYMENT.md` 和
`docs/VALIDATION.md`。地图及采集记录保留在 `data/outdoor_mapping`，日志在 `run`。
