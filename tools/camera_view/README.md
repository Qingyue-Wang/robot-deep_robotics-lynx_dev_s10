# 山猫相机网页

通过 ffmpeg 读取山猫四路 RTSP 相机，在浏览器中显示最新画面和电量。
默认每秒更新一帧，可切换前后左右视角，调整画质与 0.1～5 秒刷新间隔。
需要 Python 3、ffmpeg 和 ffprobe，不需要 ROS 或导航服务。

在山猫上的 `robot-deep_robotics-lynx_s10` 仓库根目录启动：

```bash
bash tools/camera_view/start.sh
```

在电脑转发相机网页：

```bash
ssh -p 6003 -N -L 8080:127.0.0.1:8080 nvidia@svr.gddg08.top
```

浏览器打开 `http://127.0.0.1:8080`。服务默认只监听机器人回环地址，通过隧道访问。
导航与相机同时查看时，可在同一个 SSH 命令增加 `-L 50110:127.0.0.1:50108`。

相机地址在 `config.json`：前 video4、后 video0、左 video2、右 video6，
默认主机为 `10.21.33.103:8554`。用 `CAMERA_CONFIG=/实际路径/config.json`
可指定其他配置。按钮灰色表示该路暂时没有可用视频流；画质设置只在当前进程生效。

需要常驻时使用 `lynx-camera.service`；先按实际工作区位置修改其中的
`WorkingDirectory` 和 `ExecStart`，再安装：

```bash
mkdir -p ~/.config/systemd/user
cp tools/camera_view/lynx-camera.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now lynx-camera.service
```
