# LIMIT-RM-Vision

UESTC-LIMITI-RM-CV 面向 RoboMaster 自瞄场景维护的 ROS 2 视觉工程。目前仓库聚焦于
海康机器人工业相机采集、装甲板检测、PnP 位姿解算与目标跟踪，并以
ROS 2 Jazzy 作为主要运行环境。

本项目在开源项目 `rm_auto_aim` 和 `rm_auto_record` 的基础上继续开发，并通过
Git subtree 保留上游提交历史、版权声明和许可证。当前版本尚不包含串口通信、
弹道解算、能量机关识别或完整机器人 bringup。

## 当前功能

- HIKROBOT USB3/GigE 工业相机低延迟采集
- 发布 `rgb8` 图像和相机标定信息
- 灯条提取、装甲板匹配和数字分类
- 基于相机内参的 PnP 三维位姿解算
- 基于扩展卡尔曼滤波的目标跟踪
- RViz2/rqt 调试图像与三维 Marker
- 已导入自动录包组件源码（ROS 2 Jazzy 适配待完成）

## 数据流

```text
HIK Camera
  ├─ /image_raw ───────┐
  └─ /camera_info ─────┴─> armor_detector
                              ├─ /detector/result_img
                              ├─ /detector/marker
                              └─ /detector/armors ──> armor_tracker
                                                        └─ /tracker/target

TF (camera frame -> target frame) ─────────────────────> armor_tracker
/record_controller + configured topics ───────────────> rm_auto_record
```

## 仓库结构

```text
.
└── src
    ├── hik_camera
    │   ├── config
    │   ├── launch
    │   └── src
    ├── rm_auto_aim
    │   ├── auto_aim_interfaces
    │   ├── armor_detector
    │   ├── armor_tracker
    │   └── rm_auto_aim
    └── rm_auto_record
```

主要 ROS 2 包：

| 包 | 作用 |
| --- | --- |
| `hik_camera` | 连接 HIKROBOT 相机，发布 `/image_raw` 和 `/camera_info` |
| `auto_aim_interfaces` | 定义检测器、跟踪器和调试消息 |
| `armor_detector` | 装甲板检测、数字分类和 PnP 解算 |
| `armor_tracker` | 坐标变换、目标匹配和 EKF 跟踪 |
| `rm_auto_record` | 自动 rosbag2 录制；当前仍需完成 ROS 2 Jazzy API 适配 |

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- HIKROBOT MVS SDK，默认安装在 `/opt/MVS`
- 系统 OpenCV 4.6（与 ROS 2 Jazzy 的 `cv_bridge` 保持一致）
- CMake、colcon 和支持 C++17 的编译器

> 本仓库不分发 HIKROBOT MVS SDK、厂商头文件或动态库。请通过厂商渠道获取 SDK，
> 并遵守其许可协议。

## 获取源码

```bash
git clone git@github.com:UESTC-LIMITI-RM-CV/Armor_Detect-Track.git LIMIT-RM-Vision
cd LIMIT-RM-Vision
```

## 安装依赖

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

MVS SDK 不由 rosdep 安装，需要提前确认以下文件存在：

```text
/opt/MVS/include/MvCameraControl.h
/opt/MVS/lib/64/libMvCameraControl.so
```

## 编译

建议分别编译相机和算法包，使每组 CMake 参数只作用于相关包：

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select hik_camera --cmake-args -DCMAKE_BUILD_TYPE=Release -DMVS_ROOT=/opt/MVS

colcon build --symlink-install --packages-up-to rm_auto_aim --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4

source install/setup.bash
```

显式指定 `OpenCV_DIR` 是为了避免错误链接到 `/usr/local` 中自行安装的其他
OpenCV 版本。不要让同一个节点同时加载 ROS Jazzy 的 OpenCV 4.6 和另一个 ABI
版本。

## 相机配置

启动前检查：

```text
src/hik_camera/config/camera.yaml
```

重点参数：

- `serial_number`：多相机环境建议使用序列号选择设备
- `pixel_format`：必须与传感器 Bayer 排列一致
- `exposure_time`：曝光时间，单位为微秒
- `gain`：模拟/数字增益，单位为 dB
- `frame_rate`：小于等于 0 时请求最高帧率
- `camera_info.d/k/r/p`：当前分辨率和 ROI 对应的标定参数

镜头、焦距、图像尺寸或 ROI 改变后必须重新标定。仓库内的标定值不能直接用于
另一台相机或另一支镜头。

## 快速验证装甲板检测

以下命令分别在不同终端执行。每个新终端都需要：

```bash
cd /path/to/LIMIT-RM-Vision
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. 启动相机

```bash
ros2 launch hik_camera hik_camera.launch.py
```

检查图像、帧率和标定：

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once --field k
```

### 2. 启动检测器

识别红色目标：

```bash
ros2 run armor_detector armor_detector_node --ros-args -p debug:=true -p detect_color:=0 -p binary_thres:=160 -p classifier_threshold:=0.7
```

识别蓝色目标时，将 `detect_color` 改为 `1`。

### 3. 查看结果

```bash
ros2 run rqt_image_view rqt_image_view
```

在界面中选择：

- `/image_raw`：原始画面
- `/detector/binary_img`：二值化结果
- `/detector/number_img`：送入分类器的数字图像
- `/detector/result_img`：最终识别叠加结果

查看结构化检测结果：

```bash
ros2 topic echo /detector/armors
```

检测器会在没有目标时发布空数组，因此：

```bash
ros2 topic hz /detector/armors
```

测量的是处理帧率，不等同于识别成功率。

## 启动跟踪器

```bash
ros2 run armor_tracker armor_tracker_node --ros-args -p target_frame:=odom
```

跟踪器需要 TF 树中存在从相机图像帧（默认 `camera_optical_frame`）到
`target_frame` 的有效变换。没有真实的相机—云台外参和云台姿态 TF 时，消息会
被 TF MessageFilter 丢弃，`/tracker/target` 不会正常输出。

生产环境不得使用单位变换代替真实外参；应由 URDF、静态 TF 和云台姿态节点共同
提供正确坐标关系。

## 运行时调节相机

```bash
ros2 param set /hik_camera exposure_time 1500.0
ros2 param set /hik_camera gain 6.0
ros2 param set /hik_camera frame_rate 0.0
```

只有曝光、增益和帧率可以动态修改。设备选择、像素格式、ROI、缓冲区和标定参数
修改后需要重启相机节点。

## 自动录包组件

`rm_auto_record` 的上游源码已通过 subtree 保留在仓库中，但当前版本尚不能在
ROS 2 Jazzy 下编译。其 rosbag2 写入代码使用了旧版 `TopicMetadata` 初始化方式和
`SerializedBagMessage::time_stamp` 字段，需要完成 Jazzy API 迁移后才能启用。

因此默认编译和快速启动流程不会构建或启动该包。

完成移植后，该节点的预期行为如下。

`rm_auto_record` 订阅 `/record_controller`：

- 消息内容为 `start` 时开始录制
- 消息内容为 `stop` 时停止录制

当前上游实现固定创建两个订阅，依次对应：

1. `sensor_msgs/msg/CompressedImage`
2. `sensor_msgs/msg/CameraInfo`

除完成 Jazzy API 适配外，使用前还必须同时配置 `topic_name` 和
`topic_type`，并确保压缩图像话题确实存在；仅设置保存路径会导致启动录制时
访问空话题列表。

配置文件位于：

```text
src/rm_auto_record/config/node_params.yaml
```

建议在后续开发中将其改造成真正的动态通用录包节点，或直接调用 rosbag2 的通用
录制接口。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select armor_detector
colcon test-result --verbose
```

`armor_detector` 的测试覆盖节点启动、数字分类器以及垂直、双向倾斜和水平灯条
几何行为。

## 常见问题

### RViz2 中没有图像

`/image_raw` 使用 Sensor Data QoS。在 RViz2 的 Image 显示项中，将
Reliability Policy 设置为 `Best Effort`。

### 检测器没有三维输出

确认 `/camera_info` 中的 `K` 和 `D` 不是全零，并确认图像分辨率与标定时
一致。

### 跟踪器没有输出

检查：

```bash
ros2 run tf2_ros tf2_echo odom camera_optical_frame
```

如果无法获得变换，应先完成相机—云台外参和云台姿态 TF。

### 编译时出现 OpenCV 4.6/4.14 冲突

清理旧 CMake 缓存，并使用系统 OpenCV：

```bash
-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

## 上游项目与版权

本仓库包含从以下开源项目导入并继续修改的代码：

### rm_auto_aim

- 上游仓库：<https://github.com/FaterYU/rm_auto_aim>
- 原始作者：Chen Jun
- 后续开发：Zheng Yu（FaterYU）
- 许可证：[MIT License](src/rm_auto_aim/LICENSE)
- 原版权声明：`Copyright (c) 2022 ChenJun`

本仓库保留了上游提交历史、LICENSE 文件以及源码中的原作者版权头。对
`rm_auto_aim` 的修改仍须遵守其 MIT License。

### rm_auto_record

- 上游仓库：<https://github.com/FaterYU/rm_auto_record>
- 作者/维护者：Zheng Yu（FaterYU）
- 许可证：[Apache License 2.0](src/rm_auto_record/LICENSE)

本仓库保留了 Apache-2.0 许可证和原源码版权声明。分发修改版本时应继续满足
Apache-2.0 的署名、许可证及修改说明要求。

### HIK Camera ROS 2 Driver

- 项目仓库：<https://github.com/UESTC-LIMITI-RM-CV/HIK_Camera_ROS2_Driver>
- 许可证：[MIT License](src/hik_camera/LICENSE)

HIKROBOT MVS SDK 为独立的厂商软件，不属于上述 MIT License 授权范围，本仓库
不对其进行再分发。

## 许可证说明

当前 monorepo 内包含多种许可证，并非所有目录自动适用同一个统一许可证：

- `src/rm_auto_aim`：MIT License
- `src/hik_camera`：MIT License
- `src/rm_auto_record`：Apache License 2.0

各子目录中的 LICENSE 文件分别约束对应代码。新增代码的统一版权主体和根许可证
将在项目正式发布前由维护团队确定。在此之前，请勿删除或替换任何上游版权声明、
LICENSE 或源码文件头。

## 致谢

感谢 Chen Jun、Zheng Yu（FaterYU）以及原 `rm_vision` 社区贡献者公开
RoboMaster 视觉框架及相关实现。本项目的后续开发建立在这些工作之上。
