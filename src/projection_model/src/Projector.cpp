//
// Migrated to ROS2 from ROS1 version created by glawless on 10.05.17.
// Migration done by Mohamed Abdelmawgoud on 12.06.25 .
//

#include <projection_model/Projector.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ros2_utils/clock_sync.hpp>

namespace model_distance_from_height {

Projector::Projector() : Node("model_distance_from_height_node"), last_time_(rclcpp::Time(0)) {
  using namespace pose_cov_ops::interface;

  // Declare use_sim_time parameter FIRST (default: wall-clock/system time)
  if (!this->has_parameter("use_sim_time")) {
    this->declare_parameter("use_sim_time", false);
  }

  // Declare and get parameters
  this->declare_parameter("projected_object_topic", "/machine_1/object_detections/projected_to_world");
  this->declare_parameter("camera_debug_topic", "/machine_1/object_detections/camera_debug");
  this->declare_parameter("detections_topic", "/person_detections");
  this->declare_parameter("tracker_topic", "/machine_1/target_tracker/pose");
  this->declare_parameter("offset_topic", "/machine_1/target_tracker/offset");
  this->declare_parameter("feedback_topic", "/neural_network_feedback");
  
  // Frame topics
  this->declare_parameter("topics.robot", "/X3/pose_with_covariance");
  this->declare_parameter("topics.camera", "/machine_1/camera/pose");
  this->declare_parameter("topics.optical", "/machine_1/camera/pose_optical");
  
  // Model parameters
  this->declare_parameter("height_model_mean", 1.7);
  this->declare_parameter("height_model_var", 1.0);
  this->declare_parameter("uncertainty_scale_head", 1.0);
  this->declare_parameter("uncertainty_scale_feet", 1.0);
  
  // Camera info topic
  this->declare_parameter("camera.info_topic", "/machine_1/video/camera_info");

  // Get use_sim_time status for logging
  bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    RCLCPP_INFO(this->get_logger(), "Using simulation time");
  } else {
    RCLCPP_INFO(this->get_logger(), "Using system time");
  }

  // Get parameter values
  projected_object_topic_ = this->get_parameter("projected_object_topic").as_string();
  camera_debug_topic_ = this->get_parameter("camera_debug_topic").as_string();
  detections_topic_ = this->get_parameter("detections_topic").as_string();
  tracker_topic_ = this->get_parameter("tracker_topic").as_string();
  offset_topic_ = this->get_parameter("offset_topic").as_string();
  feedback_topic_ = this->get_parameter("feedback_topic").as_string();
  
  robot_topic_ = this->get_parameter("topics.robot").as_string();
  camera_topic_ = this->get_parameter("topics.camera").as_string();
  optical_topic_ = this->get_parameter("topics.optical").as_string();
  
  double height_model_mean = this->get_parameter("height_model_mean").as_double();
  double height_model_var = this->get_parameter("height_model_var").as_double();
  head_uncertainty_scale_ = this->get_parameter("uncertainty_scale_head").as_double();
  feet_uncertainty_scale_ = this->get_parameter("uncertainty_scale_feet").as_double();
  
  camera_info_topic_ = this->get_parameter("camera.info_topic").as_string();

  // Model configuration
  projectionModel_ = std::make_unique<Model3D>(height_model_mean, height_model_var);

  // Set up parameter callback for dynamic reconfigure
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Projector::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Projector node constructed - call init() next");
}

void Projector::init() {
  // Camera information subscriber - keep active until camera model is initialized
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 1, 
    std::bind(&Projector::cameraInfoCallback, this, std::placeholders::_1));

   // Wait for camera info with proper sim_time handling
  RCLCPP_INFO(this->get_logger(), "Waiting for camera info on topic: %s", camera_info_topic_.c_str());
  while (!cameraModel_.initialized() && rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for camera info");
  }
  camera_info_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Camera model initialized successfully");

  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.keep_last(1);
  qos_profile.reliable();
  qos_profile.transient_local();

  // Setup interface configuration
  topics_ = {
    pose_cov_ops::interface::topicSubInfo<int>(robot_topic_, static_cast<int>(Poses::robot), 2000, 50, 
                     rclcpp::QoS(50).reliability(rclcpp::ReliabilityPolicy::Reliable)),
    pose_cov_ops::interface::topicSubInfo<int>(offset_topic_, static_cast<int>(Poses::gpsoffset), 2000, 50, 
                     rclcpp::QoS(50).reliability(rclcpp::ReliabilityPolicy::Reliable)),
    pose_cov_ops::interface::topicSubInfo<int>(camera_topic_, static_cast<int>(Poses::camera), 1, 1, qos_profile),
    pose_cov_ops::interface::topicSubInfo<int>(optical_topic_, static_cast<int>(Poses::optical), 1, 1, qos_profile)
  };

  // NOW it's safe to call shared_from_this() - Initialize interface
  interface_ = std::make_unique<pose_cov_ops::interface::Interface<int>>(topics_, shared_from_this());
  RCLCPP_INFO(this->get_logger(), "Waiting for camera poses to be available...");
rclcpp::Time start_wait = this->get_clock()->now();
while (rclcpp::ok()) {
    try {
        // Try to get camera pose - this will throw if not available
        geometry_msgs::msg::PoseWithCovariance test_pose;
        if (interface_->compose_up(geometry_msgs::msg::Point(), 
                                  static_cast<int>(Poses::camera), 
                                  this->get_clock()->now(), test_pose)) {
            RCLCPP_INFO(this->get_logger(), "Camera poses available, proceeding with initialization");
            break;
        }
    } catch (const std::exception& e) {
        // Expected during startup
    }
    
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Timeout after 10 seconds
    if ((this->get_clock()->now() - start_wait).seconds() > 10.0) {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for camera poses - proceeding anyway");
        break;
    }
}
  // ROS2 subscribers
  detection_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
    detections_topic_, 5, 
    std::bind(&Projector::detectionCallback3D, this, std::placeholders::_1));

  tracker_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    tracker_topic_, 5,
    std::bind(&Projector::trackerCallback, this, std::placeholders::_1));

  // ROS2 publishers
  object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    projected_object_topic_, 10);
    
  camera_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    camera_debug_topic_, 10);
    
  feedback_pub_ = this->create_publisher<neural_network_msgs::msg::NeuralNetworkFeedback>(
    feedback_topic_, 5);

#ifdef DEBUG_PUBLISHERS
  debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "debug/model_distance_from_height/debugPose", 10);
#endif

  // Update timestamps after clock sync
  updateTimestampsAfterClockSync();

  RCLCPP_INFO(this->get_logger(), "Projector node fully initialized");
}

void Projector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  cameraModel_.fromCameraInfo(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), "Camera model updated");
}

rcl_interfaces::msg::SetParametersResult Projector::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters) {
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "height_model_mean") {
      projectionModel_->height_model_.mean = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Height model mean updated to: %f", param.as_double());
    } else if (param.get_name() == "height_model_var") {
      projectionModel_->height_model_.var = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Height model variance updated to: %f", param.as_double());
    } else if (param.get_name() == "uncertainty_scale_head") {
      head_uncertainty_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Head uncertainty scale updated to: %f", param.as_double());
    } else if (param.get_name() == "uncertainty_scale_feet") {
      feet_uncertainty_scale_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Feet uncertainty scale updated to: %f", param.as_double());
    } else if (param.get_name() == "use_sim_time") {
      // Handle use_sim_time parameter changes
      bool use_sim_time = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "use_sim_time updated to: %s", use_sim_time ? "true" : "false");
    }
  }
  
  return result;
}

void Projector::detectionCallback3D(const neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr msg) {
  if (!msg) {
    RCLCPP_ERROR(this->get_logger(), "Received null detection message");
    return;
  }

  if (detectBackwardsTimeJump()) {
    return;
  }

  geometry_msgs::msg::Point pup, pzero;
  pup.z = -1.0;
  geometry_msgs::msg::PoseWithCovariance vec_zero, vec_up, out_pose;

  const static std::vector<int> keys{
    (int)(Poses::gpsoffset), 
    (int)(Poses::robot), 
    (int)(Poses::camera), 
    (int)(Poses::optical)
  };

  // Covariance matrix indices
  const static int colx = 0;
  const static int coly = 1;
  const static int colz = 2;
  const static int rowx = 0;
  const static int rowy = 1 * 6;
  const static int rowz = 2 * 6;

  // For every detection
  for (const auto &detection : msg->detections) {
    // Filter low confidence detections - reduced threshold for better tracking
    if (detection.detection_score < 0.5) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping low confidence detection: %.3f", detection.detection_score);
      continue;
    }
    // Transform vector [0 0 -1] to optical frame
    if (!interface_->compose_down(pup, keys, detection.header.stamp, vec_up)) {
      interface_warning();
      continue;
    }

    // We need a second vector because we want a resulting vector relative to camera
    interface_->compose_down(pzero, keys, detection.header.stamp, vec_zero);
    vec_up.pose.position.x -= vec_zero.pose.position.x;
    vec_up.pose.position.y -= vec_zero.pose.position.y;
    vec_up.pose.position.z -= vec_zero.pose.position.z;

    // Rectify points if there's distortion
    cv::Point2d unrect_min(detection.xmin, detection.ymin);
    cv::Point2d unrect_max(detection.xmax, detection.ymax);
    cv::Point2d rect_min(cameraModel_.rectifyPoint(unrect_min));
    cv::Point2d rect_max(cameraModel_.rectifyPoint(unrect_max));

    // Apply camera model to pixel variables
    cv::Point3d ray_min(cameraModel_.projectPixelTo3dRay(rect_min));
    cv::Point3d ray_max(cameraModel_.projectPixelTo3dRay(rect_max));

    // Propagate uncertainty from pixel variables to meters
    double var_xmin = detection.variance_xmin / (cameraModel_.fx() * cameraModel_.fx());
    double var_xmax = detection.variance_xmax / (cameraModel_.fx() * cameraModel_.fx());
    double var_ymin = detection.variance_ymin / (cameraModel_.fy() * cameraModel_.fy());
    double var_ymax = detection.variance_ymax / (cameraModel_.fy() * cameraModel_.fy());

    // Center in normalized optical frame (2d projection)
    geometry_msgs::msg::PoseWithCovariance center_pose;
    center_pose.pose.position.z = 1.0;
    center_pose.pose.position.x = 0.5 * (ray_max.x + ray_min.x);
    center_pose.pose.position.y = 0.5 * (ray_max.y + ray_min.y);
    center_pose.pose.orientation.w = 1.0;
    center_pose.covariance[rowx + colx] = 0.25 * (var_xmax + var_xmin);
    center_pose.covariance[rowy + coly] = 0.25 * (var_ymin + var_ymax);

    // Height information
    double uncorr_height = fabs(ray_max.y - ray_min.y);
    double uncorr_height_cov = var_ymin + var_ymax;

    // Estimate adaptive height based on detection
    double adaptive_height = projectionModel_->estimate_person_height_from_detection(detection);

    // Compute expected distance to object using adaptive height
    double scalefactor = projectionModel_->compute_distance_adaptive(vec_up.pose.position, ray_min.y, uncorr_height, adaptive_height);

    // Apply additional stability checks
    if (scalefactor < 0.5 || scalefactor > 50.0) {
      RCLCPP_WARN(this->get_logger(), "Computed distance %.2fm is out of reasonable range, using fallback", scalefactor);
      // Use fallback with default height model
      scalefactor = projectionModel_->compute_distance(vec_up.pose.position, ray_min.y, uncorr_height);
      scalefactor = std::max(0.5, std::min(50.0, scalefactor));
    }

    RCLCPP_DEBUG(this->get_logger(), "Adaptive height: %.2fm, distance: %.2fm (confidence: %.3f, aspect: %.2f)",
                adaptive_height, scalefactor, detection.detection_score,
                std::abs(detection.ymax - detection.ymin) / std::abs(detection.xmax - detection.xmin));

    // Scale all variables accordingly - z is later after rotating to object
    center_pose.pose.position.x *= scalefactor;
    center_pose.pose.position.y *= scalefactor;
    center_pose.pose.position.z *= scalefactor;
    center_pose.covariance[rowx + colx] *= scalefactor * scalefactor;
    center_pose.covariance[rowy + coly] *= scalefactor * scalefactor;

    // Obtain new frame rotated to object
    cv::Point3d rotation_axis(cv::Point3d{0, 0, 1}.cross(cv::Point3d{
      center_pose.pose.position.x, center_pose.pose.position.y, center_pose.pose.position.z}));
    rotation_axis /= cv::norm(rotation_axis);
    
    geometry_msgs::msg::PoseWithCovariance rotation_pose;
    double angle = asin(sqrt(center_pose.pose.position.x * center_pose.pose.position.x + 
                            center_pose.pose.position.y * center_pose.pose.position.y) / scalefactor);
    rotation_pose.pose.orientation.x = rotation_axis.x * sin(angle / 2);
    rotation_pose.pose.orientation.y = rotation_axis.y * sin(angle / 2);
    rotation_pose.pose.orientation.z = rotation_axis.z * sin(angle / 2);
    rotation_pose.pose.orientation.w = cos(angle / 2);

    // Compose rotation
    geometry_msgs::msg::PoseWithCovariance rotated_center_pose;
    pose_cov_ops::inverseCompose(center_pose, rotation_pose, rotated_center_pose);

    // Propagation of uncertainty for z
    double prop_A = rotated_center_pose.pose.position.z / scalefactor;
    double prop_B = scalefactor;
    double prop_f = rotated_center_pose.pose.position.z;
    double prop_var_A = rotated_center_pose.covariance[rowz + colz];
    double prop_var_B = projectionModel_->compute_dist_var(vec_up, uncorr_height, uncorr_height_cov, ray_min.y, var_ymin, scalefactor);
    rotated_center_pose.covariance[rowz + colz] = (prop_f * prop_f) * (prop_var_A / (prop_A * prop_A)) + (prop_var_B / (prop_B * prop_B));

    // Compose the rotation back
    pose_cov_ops::compose(rotation_pose, rotated_center_pose, center_pose);

    // Rotate using the interface
    interface_->compose_up(center_pose, keys, detection.header.stamp, out_pose);

    // Publish with consistent timestamp (preserve original detection timestamp)
    geometry_msgs::msg::PoseWithCovarianceStamped out_stamped;
    out_stamped.pose = out_pose;
    out_stamped.header.stamp = detection.header.stamp;  // Keep original timestamp for consistency
    out_stamped.header.frame_id = "world";  // world frame is hardcoded
    object_pose_pub_->publish(out_stamped);

#ifdef DEBUG_PUBLISHERS
    geometry_msgs::msg::PoseWithCovarianceStamped debug_pose_stamped;
    debug_pose_stamped.pose = vec_up;
    debug_pose_stamped.header.stamp = detection.header.stamp;
    debug_pose_stamped.header.frame_id = "machine_1_camera_rgb_optical_link";
    debug_pub_->publish(debug_pose_stamped);
#endif
  }
}

void Projector::interface_warning() const {
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "Some frame wasn't found at this time. Enable DEBUG logging for this node to view the interface state. Possible issues:\n"
    "1) This message may appear once or twice in normal conditions, while gathering all necessary messages\n"
    "2) There is a large time offset between different cache keys while composing; this can be caused by a node crash OR "
    "not setting the parameter use_sim_time true while using offline data - static TFs might be using rclcpp::Clock::now() "
    "and normal topics will use the stamps from when they were produced");
}

void Projector::trackerCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (detectBackwardsTimeJump()) {
    return;
  }

  neural_network_msgs::msg::NeuralNetworkFeedback feedback_msg;
  feedback_msg.header.stamp = msg->header.stamp;
  feedback_msg.debug_included = static_cast<uint8_t>(true);

  // 3D pose is received already predicted into its timestamp
  const static std::vector<int> keys{
    static_cast<int>(Poses::gpsoffset), 
    static_cast<int>(Poses::robot), 
    static_cast<int>(Poses::camera), 
    static_cast<int>(Poses::optical)
  };

  // Auxiliary
  const static int rowy = 1 * 6;
  const static int coly = 1;

  // Get head and feet positions assuming standing up
  geometry_msgs::msg::PoseWithCovariance head(msg->pose), feet(msg->pose);

  // Head and feet given the center and half the height model mean
  // This is "reverted" because in our world frame z points down
  head.pose.position.z -= (0.5 * projectionModel_->height_model_.mean);
  feet.pose.position.z += (0.5 * projectionModel_->height_model_.mean);

  // Before adding uncertainty, compose to add debug information
  geometry_msgs::msg::PoseWithCovariance head_debug, feet_debug;
  interface_->compose_down(head.pose, keys, msg->header.stamp, head_debug);
  interface_->compose_down(feet.pose, keys, msg->header.stamp, feet_debug);

  // Now add the uncertainty
  double person_model_uncertainty = sqrt(projectionModel_->height_model_.var);
  head.pose.position.z -= head_uncertainty_scale_ * 0.5 * person_model_uncertainty;
  feet.pose.position.z += feet_uncertainty_scale_ * 0.5 * person_model_uncertainty;

  // Compose both these points into optical frame
  geometry_msgs::msg::PoseWithCovariance head_optical, feet_optical;
  interface_->compose_down(head, keys, msg->header.stamp, head_optical);
  interface_->compose_down(feet, keys, msg->header.stamp, feet_optical);

  // Add scalar * half (for each side) sigma of projection uncertainty
  double head_pose_uncertainty = sqrt(head_optical.covariance[rowy + coly]);
  double feet_pose_uncertainty = sqrt(feet_optical.covariance[rowy + coly]);
  head_optical.pose.position.y -= head_uncertainty_scale_ * 0.5 * head_pose_uncertainty;
  feet_optical.pose.position.y += feet_uncertainty_scale_ * 0.5 * feet_pose_uncertainty;

  // Finally we need the center for x (and y for debug)
  geometry_msgs::msg::PoseWithCovariance center_optical;
  interface_->compose_down(msg->pose, keys, msg->header.stamp, center_optical);

  // Special case where target is behind camera
  if (head_optical.pose.position.z <= 0.0 || feet_optical.pose.position.z <= 0.0 || center_optical.pose.position.z <= 0.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "The target is behind me!");
    feedback_msg.ymin = -1;
    feedback_msg.ymax = -1;
    feedback_msg.xcenter = -1;
    feedback_pub_->publish(feedback_msg);
    return;
  }

  // Project points to image, distorting them afterwards
  cv::Point2d detection_head = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(
    cv::Point3d(head_optical.pose.position.x, head_optical.pose.position.y, head_optical.pose.position.z)));
  cv::Point2d detection_feet = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(
    cv::Point3d(feet_optical.pose.position.x, feet_optical.pose.position.y, feet_optical.pose.position.z)));
  cv::Point2d detection_center = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(
    cv::Point3d(center_optical.pose.position.x, center_optical.pose.position.y, center_optical.pose.position.z)));

  // These are for debugging
  cv::Point2d detection_head_raw = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(
    cv::Point3d(head_debug.pose.position.x, head_debug.pose.position.y, head_debug.pose.position.z)));
  cv::Point2d detection_feet_raw = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(
    cv::Point3d(feet_debug.pose.position.x, feet_debug.pose.position.y, feet_debug.pose.position.z)));

  // Send these points back to neural network
  feedback_msg.ymin = static_cast<int16_t>(detection_head.y);
  feedback_msg.ymax = static_cast<int16_t>(detection_feet.y);
  feedback_msg.xcenter = static_cast<int16_t>(detection_center.x);
  feedback_msg.ycenter = static_cast<int16_t>(detection_center.y);
  feedback_msg.head_raw = static_cast<int16_t>(detection_head_raw.y);
  feedback_msg.feet_raw = static_cast<int16_t>(detection_feet_raw.y);

  feedback_pub_->publish(feedback_msg);

  // Additionally, publish camera debug pose
  geometry_msgs::msg::Point pzero;
  geometry_msgs::msg::PoseWithCovariance camera_in_world;

  if (!interface_->compose_up(pzero, keys, msg->header.stamp, camera_in_world)) {
    interface_warning();
  } else {
    geometry_msgs::msg::PoseWithCovarianceStamped camera_pose_stamped;
    camera_pose_stamped.pose = camera_in_world;
    camera_pose_stamped.header.stamp = msg->header.stamp;
    camera_pose_stamped.header.frame_id = "world";
    camera_debug_pub_->publish(camera_pose_stamped);
  }
}

bool Projector::detectBackwardsTimeJump() {
  bool using_sim_time = false;
  try {
    using_sim_time = this->get_parameter("use_sim_time").as_bool();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
    // Parameter not set, default to false
    RCLCPP_WARN_ONCE(this->get_logger(), "use_sim_time parameter not declared - defaulting to false");
    using_sim_time = false;
  }

  if (!using_sim_time)
    return false;

  rclcpp::Time current_time = this->get_clock()->now();

  if (current_time < last_time_) {
    // Jump backwards detected, reset interface
    RCLCPP_WARN(this->get_logger(), "Backwards jump in time detected, resetting interface");
    interface_ = std::make_unique<pose_cov_ops::interface::Interface<int>>(topics_, shared_from_this());
    last_time_ = current_time;
    return true;
  }
  last_time_ = current_time;
  return false;
}

void Projector::updateTimestampsAfterClockSync() {
  auto current_time = this->get_clock()->now();
  
  if (current_time.nanoseconds() > 0) {
    RCLCPP_INFO(this->get_logger(), "Updating internal timestamps after clock sync: %.3f", 
               current_time.seconds());
    
    // Update last_time_ to prevent false backwards time jump detection
    last_time_ = current_time;
    
    RCLCPP_INFO(this->get_logger(), "Internal timestamps updated successfully");
  } else {
    RCLCPP_WARN(this->get_logger(), "Cannot update timestamps - invalid clock time");
  }
}

} // namespace model_distance_from_height

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<model_distance_from_height::Projector>();

  WAIT_FOR_CLOCK_DELAYED(node);

  node->init();  // Now safe to call shared_from_this() and update timestamps

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
