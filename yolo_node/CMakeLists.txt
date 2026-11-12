cmake_minimum_required(VERSION 3.16)
project(yolo_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

add_executable(yolo_roi_node src/yolo_roi_node.cpp)
target_include_directories(yolo_roi_node PRIVATE src)
target_link_libraries(yolo_roi_node PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  ${OpenCV_LIBS}
)

install(TARGETS yolo_roi_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
