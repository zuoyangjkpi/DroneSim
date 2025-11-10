cmake_minimum_required(VERSION 3.8)
project(drone_low_level_controllers)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  drone_low_level_controllers/multicopter_velocity_control_adapter.py
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
