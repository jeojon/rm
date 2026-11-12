// Copyright 2026
// ROS 2 image prefilter node: run YOLO and keep only detected ROIs.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "yolo_roi_mask.hpp"

namespace yolo_node
{

class YoloRoiNode : public rclcpp::Node
{
public:
  YoloRoiNode()
  : Node("yolo_roi_node")
  {
    const std::string default_model_path = resolveDefaultModelPath();

    model_path_ = declare_parameter<std::string>("model_path", default_model_path);
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/detector/image_roi");

    yolo_roi::Config config;
    config.input_width = declare_parameter<int>("input_width", 640);
    config.input_height = declare_parameter<int>("input_height", 640);
    config.score_threshold =
      static_cast<float>(declare_parameter<double>("score_threshold", 0.25));
    config.nms_threshold =
      static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    config.has_objectness = declare_parameter<bool>("has_objectness", false);
    config.class_names =
      declare_parameter<std::vector<std::string>>("class_names", std::vector<std::string>{});

    algorithm_ = std::make_unique<yolo_roi::YoloRoiMask>(model_path_, config);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&YoloRoiNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "YOLO ROI node ready: %s -> %s, model: %s", image_topic_.c_str(),
      output_topic_.c_str(), model_path_.c_str());
  }

private:
  static std::string resolveDefaultModelPath()
  {
    try {
      return ament_index_cpp::get_package_share_directory("yolo_node") +
             "/model/yolov8n.onnx";
    } catch (const std::exception &) {
      return "model/yolov8n.onnx";
    }
  }

  cv::Mat imageMsgToBgr(const sensor_msgs::msg::Image & message) const
  {
    if (message.height == 0 || message.width == 0 || message.data.empty()) {
      throw std::runtime_error("Received an empty image message");
    }

    if (message.encoding == "rgb8") {
      cv::Mat rgb(
        static_cast<int>(message.height), static_cast<int>(message.width), CV_8UC3,
        const_cast<std::uint8_t *>(message.data.data()), static_cast<std::size_t>(message.step));
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return bgr;
    }

    if (message.encoding == "bgr8") {
      cv::Mat bgr(
        static_cast<int>(message.height), static_cast<int>(message.width), CV_8UC3,
        const_cast<std::uint8_t *>(message.data.data()), static_cast<std::size_t>(message.step));
      return bgr.clone();
    }

    if (message.encoding == "mono8") {
      cv::Mat gray(
        static_cast<int>(message.height), static_cast<int>(message.width), CV_8UC1,
        const_cast<std::uint8_t *>(message.data.data()), static_cast<std::size_t>(message.step));
      cv::Mat bgr;
      cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }

    throw std::runtime_error("Unsupported image encoding: " + message.encoding);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      const cv::Mat bgr = imageMsgToBgr(*message);
      const yolo_roi::Result result = algorithm_->process(bgr);

      auto output = std::make_unique<sensor_msgs::msg::Image>();
      output->header = message->header;
      output->height = message->height;
      output->width = message->width;
      output->encoding = "rgb8";
      output->is_bigendian = false;
      output->step = message->width * 3U;
      output->data.resize(static_cast<std::size_t>(output->step) * output->height);

      cv::Mat output_rgb(
        static_cast<int>(output->height), static_cast<int>(output->width), CV_8UC3,
        output->data.data(), static_cast<std::size_t>(output->step));
      cv::cvtColor(result.masked, output_rgb, cv::COLOR_BGR2RGB);

      publisher_->publish(std::move(output));
      RCLCPP_DEBUG(
        get_logger(), "Published %zu ROI detections for frame %s", result.detections.size(),
        message->header.frame_id.c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to process image: %s", exception.what());
    }
  }

  std::string model_path_;
  std::string image_topic_;
  std::string output_topic_;
  std::unique_ptr<yolo_roi::YoloRoiMask> algorithm_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace yolo_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yolo_node::YoloRoiNode>());
  rclcpp::shutdown();
  return 0;
}
