#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hik_camera/hik_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    rclcpp::spin(std::make_shared<hik_camera::HikCameraNode>(options));
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("hik_camera"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
