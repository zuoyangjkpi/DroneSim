cmake_minimum_required(VERSION 3.8)
project(neural_network_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(neural_network_msgs REQUIRED)
find_package(ros2_utils REQUIRED)

# -----------------------------
# third_party roots / ONNX Runtime (GPU 1.22.0 via stable symlink)
# -----------------------------
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP")

# Use the stable symlink you created:
#   third_party/YOLOs-CPP/onnxruntime -> onnxruntime-linux-x64-gpu-1.22.0
set(ONNXRUNTIME_ROOT "${THIRD_PARTY_DIR}/onnxruntime")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime libonnxruntime
  PATHS "${ONNXRUNTIME_ROOT}/lib"
  NO_DEFAULT_PATH
  REQUIRED)

if(NOT EXISTS "${ONNXRUNTIME_LIB}")
  message(FATAL_ERROR "ONNX Runtime library not found at: ${ONNXRUNTIME_LIB}")
endif()

# -----------------------------
# Sources / Target
# -----------------------------
add_executable(yolo12_detector_node
  src/yolo12_detector_node.cpp
)

add_executable(detection_visualizer_node
  src/detection_visualizer_node.cpp
)

target_include_directories(yolo12_detector_node PRIVATE
  include
  ${THIRD_PARTY_DIR}/include           # YOLO-provided headers (if any)
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(yolo12_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  neural_network_msgs
  OpenCV
  rclcpp_components
  ament_index_cpp
  ros2_utils
)

target_link_libraries(yolo12_detector_node
  ${ONNXRUNTIME_LIB}
  curl
  ${OpenCV_LIBS}
)

target_include_directories(detection_visualizer_node PRIVATE
  include
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(detection_visualizer_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
  neural_network_msgs
  ament_index_cpp
  OpenCV
)

target_link_libraries(detection_visualizer_node
  curl
  ${OpenCV_LIBS}
)

# Runtime RPATH:
#   - During builds: pick provider .so files from the third_party ORT dir
#   - After install: look next to the executable in lib/ ($ORIGIN = install/lib/${PROJECT_NAME})
set_target_properties(yolo12_detector_node PROPERTIES
  BUILD_RPATH   "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "\$ORIGIN/lib"
)

# -----------------------------
# Install
# -----------------------------
# Executables
install(TARGETS
  yolo12_detector_node
  detection_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the ORT core + provider shared libs next to the executable in lib/
install(DIRECTORY "${ONNXRUNTIME_ROOT}/lib/"
  DESTINATION "lib/${PROJECT_NAME}/lib"
  FILES_MATCHING
    PATTERN "libonnxruntime*.so*"
    PATTERN "libonnxruntime_providers_*.so*"
)

# Models / labels / metadata
install(DIRECTORY
  third_party/
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING
    PATTERN "*.onnx"
    PATTERN "*.names"
    PATTERN "*.txt"
)

# Headers (if needed)
install(DIRECTORY include/
  DESTINATION include
)

# -----------------------------
# Testing
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
