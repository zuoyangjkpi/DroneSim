cmake_minimum_required(VERSION 3.8)
project(drone_low_level_controllers)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# Build C++ Controller Node
# ==============================================================================
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node Eigen3::Eigen)
ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  actuator_msgs
)

# ==============================================================================
# Install Targets
# ==============================================================================
install(TARGETS controller_node
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

ament_package()
