cmake_minimum_required(VERSION 3.8)
project(hanatra_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hanatra_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(esp32_bridge_node
  src/esp32_bridge.cpp
)

ament_target_dependencies(esp32_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  hanatra_msgs
  tf2_ros
)

install(TARGETS esp32_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ament_cmake)
ament_package()
