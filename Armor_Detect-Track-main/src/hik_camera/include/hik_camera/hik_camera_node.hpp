#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HikCameraNode() override;

private:
  void declareParameters();
  void initializeCameraInfo();
  void openCamera();
  void configureCamera();
  void startCapture();
  void stopAndCloseCamera();
  void captureLoop();

  bool setFloatFeatureLocked(const char * name, double value, std::string & reason);
  bool setFrameRateLocked(double value, std::string & reason);
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void * camera_handle_{nullptr};
  bool sdk_initialized_{false};
  bool grabbing_{false};
  std::atomic_bool running_{false};
  std::thread capture_thread_;
  std::mutex camera_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  sensor_msgs::msg::CameraInfo camera_info_template_;

  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  double exposure_time_{2000.0};
  double gain_{8.0};
  double frame_rate_{0.0};
  int grab_timeout_ms_{100};
};

}  // namespace hik_camera
