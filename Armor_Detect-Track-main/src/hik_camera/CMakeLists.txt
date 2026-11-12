cmake_minimum_required(VERSION 3.10)
project(hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "HIKROBOT MVS SDK installation directory")
find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  HINTS "${MVS_ROOT}/include" "/opt/MVS/include"
)
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  HINTS
    "${MVS_ROOT}/lib/64"
    "${MVS_ROOT}/lib/aarch64"
    "${MVS_ROOT}/lib/arm64"
    "/opt/MVS/lib/64"
    "/opt/MVS/lib/aarch64"
    "/opt/MVS/lib/arm64"
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "HIKROBOT MVS SDK was not found. Install it under /opt/MVS or pass -DMVS_ROOT=<path>.")
endif()

get_filename_component(MVS_LIBRARY_DIR "${MVS_LIBRARY}" DIRECTORY)

add_library(hik_camera_component SHARED
  src/hik_camera_node.cpp
)
target_include_directories(hik_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${MVS_INCLUDE_DIR}"
)
target_compile_options(hik_camera_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hik_camera_component
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs
)
target_link_libraries(hik_camera_component "${MVS_LIBRARY}")
set_target_properties(hik_camera_component PROPERTIES
  BUILD_RPATH "${MVS_LIBRARY_DIR}"
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
)

rclcpp_components_register_nodes(hik_camera_component
  "hik_camera::HikCameraNode"
)

add_executable(hik_camera_node src/main.cpp)
target_link_libraries(hik_camera_node hik_camera_component)
ament_target_dependencies(hik_camera_node rclcpp)
set_target_properties(hik_camera_node PROPERTIES
  BUILD_RPATH "${MVS_LIBRARY_DIR}"
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
)

install(TARGETS hik_camera_component hik_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components rcl_interfaces sensor_msgs)
ament_package()
