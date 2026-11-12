# HIK Camera ROS 2 Driver

[简体中文](README.md) | **English**

A low-latency ROS 2 Jazzy driver for HIKROBOT USB3/GigE industrial cameras,
designed primarily for the RoboMaster `rm_vision` auto-aim pipeline. The node
publishes `rgb8` images and calibrated camera information on the topic names
expected by `armor_detector`.

> This repository does not include or redistribute the HIKROBOT MVS SDK,
> headers, or libraries. Install the SDK for your system architecture from an
> official vendor source and comply with its license agreement.

## Features

- USB3 and GigE camera support with serial-number or device-index selection
- 8-bit Bayer acquisition with SDK-based RGB conversion
- `/image_raw` output as `sensor_msgs/msg/Image` in `rgb8` encoding
- `/camera_info` output as `sensor_msgs/msg/CameraInfo`
- Runtime exposure, gain, and frame-rate control
- Sensor Data QoS and a short capture queue for low latency
- Periodic reporting of publish FPS and SDK frame gaps

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- HIKROBOT MVS SDK (default location: `/opt/MVS`)
- CMake, colcon, and a C++17 compiler

## Build

Clone the repository into the `src` directory of a ROS 2 workspace:

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

Set `MVS_ROOT` to the actual SDK location if it is not installed under
`/opt/MVS`.

## Configure and run

Edit [`config/camera.yaml`](config/camera.yaml) before starting the node:

- Set `serial_number` on multi-camera systems, or select a `device_index`
- Match `pixel_format` to the sensor's Bayer pattern
- Recalibrate after changing the resolution or ROI
- Do not reuse the example calibration with a different camera, lens, or image size

```bash
ros2 launch hik_camera hik_camera.launch.py
```

To load another configuration file:

```bash
ros2 launch hik_camera hik_camera.launch.py \
  config_file:=/absolute/path/to/camera.yaml
```

## Runtime tuning

```bash
ros2 param set /hik_camera exposure_time 1500.0  # microseconds
ros2 param set /hik_camera gain 6.0              # dB
ros2 param set /hik_camera frame_rate 0.0        # 0 requests maximum FPS
```

Only `exposure_time`, `gain`, and `frame_rate` are dynamically adjustable.
Device selection, pixel format, ROI, buffering, and calibration changes require
a node restart. Actual FPS is also limited by exposure, the sensor, ROI,
interface bandwidth, color conversion, and subscriber workload.

For low-latency auto aim, prefer manual exposure and gain, an 8-bit Bayer
format, `buffer_count: 2`, and a smaller ROI where practical. Verify the stream
with:

```bash
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once --field k
```

## rm_vision integration

The default topics match `armor_detector`, so remapping is not required:

```bash
ros2 run armor_detector armor_detector_node --ros-args -p debug:=true
```

When viewing `/image_raw` in RViz2, set the Image display's Reliability Policy
to `Best Effort`. Use `/detector/result_img` for the annotated detection image.

## Calibration

With the ROS 2 `camera_calibration` package installed, a board with 8×6 inner
corners and 25 mm squares can be calibrated as follows:

```bash
ros2 run camera_calibration cameracalibrator \
  --no-service-check --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw
```

Copy the resulting `D`, `K`, `R`, `P`, and image dimensions into the YAML
configuration. Recalibrate whenever the lens, focus, resolution, or ROI changes.

## License

The source code is released under the [MIT License](LICENSE). The HIKROBOT MVS
SDK remains subject to the vendor's license and is not redistributed here.
