cmake_minimum_required(VERSION 3.8)
project(px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Include directories
include_directories(include)

# PX4 Bridge Node - sends data from DroneSim to PX4
add_executable(px4_bridge_node
  src/px4_bridge_node.cpp
)

ament_target_dependencies(px4_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  px4_msgs
)

# Install targets
install(TARGETS
  px4_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
