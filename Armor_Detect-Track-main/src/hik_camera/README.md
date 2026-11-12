# HIK Camera ROS 2 Driver

**简体中文** | [English](README_EN.md)

面向 HIKROBOT USB3/GigE 工业相机的低延迟 ROS 2 Jazzy 驱动，主要用于
RoboMaster `rm_vision` 自瞄系统。节点直接发布 `rgb8` 图像和相机标定信息，
可与 `armor_detector` 默认话题连接。

> 本仓库不包含海康机器人 MVS SDK、头文件或动态库。使用前请从厂商渠道安装
> 与系统架构匹配的 MVS SDK，并遵守其许可协议。

## 功能

- 支持 USB3 和 GigE 相机，可按序列号或设备序号选择设备
- 支持 Bayer 8 位采集及 SDK 内 RGB 转换
- 发布 `/image_raw`（`sensor_msgs/msg/Image`，`rgb8`）
- 发布 `/camera_info`（`sensor_msgs/msg/CameraInfo`）
- 运行时调节曝光、增益和帧率
- 使用 Sensor Data QoS 和短缓冲队列降低延迟
- 定期输出实际发布帧率和 SDK 丢帧计数

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- HIKROBOT MVS SDK（默认安装路径 `/opt/MVS`）
- CMake、colcon 和支持 C++17 的编译器

## 编译

将仓库放入 ROS 2 工作空间的 `src` 目录：

```bash
cd ~/ros2_ws/src
git clone git@github.com:UESTC-LIMITI-RM-CV/HIK_Camera_ROS2_Driver.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hik_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DMVS_ROOT=/opt/MVS
source install/setup.bash
```

若 SDK 不在 `/opt/MVS`，请将 `MVS_ROOT` 改为实际安装路径。

## 配置与启动

启动前编辑 [`config/camera.yaml`](config/camera.yaml)：

- 多相机环境建议填写 `serial_number`，否则使用 `device_index`
- `pixel_format` 应与相机的 Bayer 排列一致
- `width`、`height` 和 ROI 改变后必须使用对应分辨率重新标定
- 示例标定参数仅适用于产生这些参数的特定相机、镜头和分辨率，不能直接复用

```bash
ros2 launch hik_camera hik_camera.launch.py
```

使用其他配置文件：

```bash
ros2 launch hik_camera hik_camera.launch.py \
  config_file:=/absolute/path/to/camera.yaml
```

## 运行时调节

```bash
ros2 param set /hik_camera exposure_time 1500.0  # 微秒
ros2 param set /hik_camera gain 6.0              # dB
ros2 param set /hik_camera frame_rate 0.0        # 0 表示请求最高帧率
```

`exposure_time`、`gain` 和 `frame_rate` 可动态修改。设备选择、像素格式、ROI、
缓冲区和标定参数属于静态配置，修改后需要重启节点。实际帧率还受曝光时间、
传感器、ROI、接口带宽、颜色转换以及订阅者负载限制。

自瞄场景建议使用手动曝光和增益、8 位 Bayer 格式、`buffer_count: 2`，并在需要时
减小 ROI。可使用以下命令检查数据：

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once --field k
```

## 接入 rm_vision

默认话题与 `armor_detector` 一致，无需重映射：

```bash
ros2 run armor_detector armor_detector_node --ros-args -p debug:=true
```

在 RViz2 中查看 `/image_raw` 时，将 Image 显示项的 Reliability Policy 设置为
`Best Effort`。查看识别叠加结果可订阅 `/detector/result_img`。

## 标定

安装 ROS 2 `camera_calibration` 后，可使用棋盘格标定。例如 8×6 个内角点、
25 mm 方格：

```bash
ros2 run camera_calibration cameracalibrator \
  --no-service-check --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw
```

完成后将生成的 `D`、`K`、`R`、`P` 和图像尺寸填写到配置文件。镜头、焦距、
分辨率或 ROI 发生变化时应重新标定。

## 许可

本项目源码采用 [MIT License](LICENSE)。HIKROBOT MVS SDK 仍受厂商许可协议约束，
本仓库不对其进行再分发。
