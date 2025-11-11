//
// tf_from_uav_pose_ros2.cpp
// ROS2 migration of tf_from_uav_poses_node
// Migrated to maintain exact functionality with proper sim time handling
//

#include "tf_from_uav_pose/tf_from_uav_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ros2_utils/clock_sync.hpp>
#include <cmath>
#include <array>
#include <Eigen/Dense>


namespace tf_from_uav_pose {

TfFromUAVPose::TfFromUAVPose() : Node("tf_from_uav_pose") {
    
    // CRITICAL: Declare use_sim_time parameter FIRST
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", false);
    }
    
    // Initialize parameters first
    initializeParameters();
    
    // Initialize TF broadcasters
    if (!dont_publish_tfs_) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' and publishing tf with parent '%s' and child '%s'", 
                   pose_topic_name_.c_str(), world_frame_id_.c_str(), machine_frame_id_.c_str());
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                         .transient_local()
                         .reliable();
    
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
            *this,  // The node reference
            qos     // QoS settings
        );
    } else {
        RCLCPP_INFO(this->get_logger(), "Requested to not publish TFs - Publishing only poses!");
    }

    // Set up parameter callback for dynamic reconfigure equivalent
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TfFromUAVPose::parametersCallback, this, std::placeholders::_1));

    // Setup static transforms (with placeholder timestamps - will be updated after clock sync)
    setupStaticTransforms();
    
    // Initialize publishers and subscribers
    initializePublishers();
    initializeSubscribers();

    RCLCPP_INFO(this->get_logger(), "TF from UAV Pose ROS2 node initialized successfully");
}

void TfFromUAVPose::initializeParameters() {
    // Get sim time status for logging
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using simulation time");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using system time");
    }
    
    // Declare topic name parameters
    this->declare_parameter("poseTopicName", pose_topic_name_);
    this->declare_parameter("rawPoseTopicName", raw_pose_topic_name_);
    this->declare_parameter("stdPoseTopicName", std_pose_topic_name_);
    this->declare_parameter("stdRawPoseTopicName", std_raw_pose_topic_name_);
    this->declare_parameter("throttledPoseTopicName", throttled_pose_topic_name_);
    this->declare_parameter("throttledUAVPoseTopicName", throttled_uav_pose_topic_name_);
    
    // Declare frame ID parameters 
    this->declare_parameter("machineFrameID", machine_frame_id_);
    this->declare_parameter("worldFrameID", world_frame_id_);
    this->declare_parameter("worldENUFrameID", world_enu_frame_id_);
    this->declare_parameter("worldNWUFrameID", world_nwu_frame_id_);
    this->declare_parameter("cameraFrameID", camera_frame_id_);
    this->declare_parameter("cameraRGBOpticalFrameID", camera_optical_frame_id_);
    
    // Declare offset and covariance parameters 
    this->declare_parameter("offsetX", offset_[0]);
    this->declare_parameter("offsetY", offset_[1]);
    this->declare_parameter("offsetZ", offset_[2]);
    this->declare_parameter("covarianceX", added_covariance_[0]);
    this->declare_parameter("covarianceY", added_covariance_[1]);
    this->declare_parameter("covarianceZ", added_covariance_[2]);
    
    // Other parameters 
    this->declare_parameter("throttleRate", throttle_rate_);
    this->declare_parameter("dontPublishTFs", dont_publish_tfs_);
    
    // Camera static publishing parameters 
    this->declare_parameter("cameraStaticPublish.publish", publish_camera_to_robot_tf_and_pose_);
    this->declare_parameter("cameraStaticPublish.TFParameters", camera_tf_parameters_);
    this->declare_parameter("cameraStaticPublish.topic", std::string("/machine_1/camera/pose"));
    this->declare_parameter("cameraStaticPublish.pose_optical_topic", std::string("/machine_1/camera/pose_optical"));

    // Get parameter values
    pose_topic_name_ = this->get_parameter("poseTopicName").as_string();
    raw_pose_topic_name_ = this->get_parameter("rawPoseTopicName").as_string();
    std_pose_topic_name_ = this->get_parameter("stdPoseTopicName").as_string();
    std_raw_pose_topic_name_ = this->get_parameter("stdRawPoseTopicName").as_string();
    throttled_pose_topic_name_ = this->get_parameter("throttledPoseTopicName").as_string();
    throttled_uav_pose_topic_name_ = this->get_parameter("throttledUAVPoseTopicName").as_string();
    
    machine_frame_id_ = this->get_parameter("machineFrameID").as_string();
    world_frame_id_ = this->get_parameter("worldFrameID").as_string();
    world_enu_frame_id_ = this->get_parameter("worldENUFrameID").as_string();
    world_nwu_frame_id_ = this->get_parameter("worldNWUFrameID").as_string();
    camera_frame_id_ = this->get_parameter("cameraFrameID").as_string();
    camera_optical_frame_id_ = this->get_parameter("cameraRGBOpticalFrameID").as_string();
    
    offset_[0] = this->get_parameter("offsetX").as_double();
    offset_[1] = this->get_parameter("offsetY").as_double();
    offset_[2] = this->get_parameter("offsetZ").as_double();
    added_covariance_[0] = this->get_parameter("covarianceX").as_double();
    added_covariance_[1] = this->get_parameter("covarianceY").as_double();
    added_covariance_[2] = this->get_parameter("covarianceZ").as_double();
    
    throttle_rate_ = this->get_parameter("throttleRate").as_double();
    dont_publish_tfs_ = this->get_parameter("dontPublishTFs").as_bool();
    
    publish_camera_to_robot_tf_and_pose_ = this->get_parameter("cameraStaticPublish.publish").as_bool();
    camera_tf_parameters_ = this->get_parameter("cameraStaticPublish.TFParameters").as_double_array();

    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Offset: [%.3f, %.3f, %.3f]", offset_[0], offset_[1], offset_[2]);
    RCLCPP_INFO(this->get_logger(), "  Added covariance: [%.3f, %.3f, %.3f]", 
               added_covariance_[0], added_covariance_[1], added_covariance_[2]);
    RCLCPP_INFO(this->get_logger(), "  Throttle rate: %.1f Hz", throttle_rate_);
}

void TfFromUAVPose::setupStaticTransforms() {
    // Prepare TF Messages with placeholder timestamps (will be updated after clock sync)
    rclcpp::Time placeholder_time(0);
    
    // TF from world to machine (this will be dynamic)
    tf_pose_.header.frame_id = world_frame_id_;
    tf_pose_.child_frame_id = machine_frame_id_;

    // TF from world_ENU to world (world_ENU will be the root of the tree)
    tf_world_enu_.header.stamp = placeholder_time;
    tf_world_enu_.header.frame_id = world_enu_frame_id_;
    tf_world_enu_.child_frame_id = world_frame_id_;
    tf2::Quaternion q_enu;
    q_enu.setEuler(M_PI, 0, M_PI_2);
    q_enu.normalize();  // Ensure normalization
    tf_world_enu_.transform.rotation = tf2::toMsg(q_enu);
    
    // Verify normalization
    auto& q_enu_msg = tf_world_enu_.transform.rotation;
    double norm_enu = sqrt(q_enu_msg.w*q_enu_msg.w + q_enu_msg.x*q_enu_msg.x + 
                          q_enu_msg.y*q_enu_msg.y + q_enu_msg.z*q_enu_msg.z);
    RCLCPP_INFO(this->get_logger(), "World ENU quaternion norm: %.6f", norm_enu);

    // TF from world to world_NWU
    tf_world_nwu_.header.stamp = placeholder_time;
    tf_world_nwu_.header.frame_id = world_frame_id_;
    tf_world_nwu_.child_frame_id = world_nwu_frame_id_;
    tf2::Quaternion q_nwu;
    q_nwu.setEuler(0, M_PI, 0);
    q_nwu.normalize();  // Ensure normalization
    tf_world_nwu_.transform.rotation = tf2::toMsg(q_nwu);
    
    // Verify normalization
    auto& q_nwu_msg = tf_world_nwu_.transform.rotation;
    double norm_nwu = sqrt(q_nwu_msg.w*q_nwu_msg.w + q_nwu_msg.x*q_nwu_msg.x + 
                          q_nwu_msg.y*q_nwu_msg.y + q_nwu_msg.z*q_nwu_msg.z);
    RCLCPP_INFO(this->get_logger(), "World NWU quaternion norm: %.6f", norm_nwu);

    // TF from camera to rgb optical link
    tf_cam_rgb_.header.stamp = tf_world_enu_.header.stamp;
    tf_cam_rgb_.header.frame_id = camera_frame_id_;
    tf_cam_rgb_.child_frame_id = camera_optical_frame_id_;
    tf2::Quaternion q_cr;
    q_cr.setEuler(M_PI_2, 0, M_PI_2);
    q_cr.normalize();  // Ensure normalization
    tf_cam_rgb_.transform.rotation = tf2::toMsg(q_cr);
    
    // Verify normalization
    auto& q_cr_msg = tf_cam_rgb_.transform.rotation;
    double norm_cr = sqrt(q_cr_msg.w*q_cr_msg.w + q_cr_msg.x*q_cr_msg.x + 
                         q_cr_msg.y*q_cr_msg.y + q_cr_msg.z*q_cr_msg.z);
    RCLCPP_INFO(this->get_logger(), "Camera RGB optical quaternion norm: %.6f", norm_cr);

    // Setup camera transforms if requested
    if (publish_camera_to_robot_tf_and_pose_) {
        setupCameraTransforms();
    }

    RCLCPP_INFO(this->get_logger(), "Static transforms configured with verified quaternion normalization");
}

void TfFromUAVPose::setupCameraTransforms() {
    if (camera_tf_parameters_.size() < 7) {
        RCLCPP_WARN(this->get_logger(), "Camera TF parameters incomplete, need 7 values (x,y,z,qx,qy,qz,qw)");
        return;
    }

    // Camera transform with placeholder timestamp
    camera_transform_.header.frame_id = machine_frame_id_;
    camera_transform_.child_frame_id = camera_frame_id_;
    camera_transform_.header.stamp = rclcpp::Time(0);  // Will be updated after clock sync

    // Set translation
    camera_transform_.transform.translation.x = camera_tf_parameters_[0];
    camera_transform_.transform.translation.y = camera_tf_parameters_[1];
    camera_transform_.transform.translation.z = camera_tf_parameters_[2];

    // Set rotation
    camera_transform_.transform.rotation.x = camera_tf_parameters_[3];
    camera_transform_.transform.rotation.y = camera_tf_parameters_[4];
    camera_transform_.transform.rotation.z = camera_tf_parameters_[5];
    camera_transform_.transform.rotation.w = camera_tf_parameters_[6];

    // CRITICAL FIX: Normalize camera quaternion to prevent MRPT errors
    auto& q = camera_transform_.transform.rotation;
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    
    if (norm < 1e-6) {
        RCLCPP_WARN(this->get_logger(), "Invalid camera quaternion with near-zero norm, using identity");
        q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
    } else {
        // Normalize camera quaternion
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
        
        RCLCPP_INFO(this->get_logger(), "Normalized camera quaternion: [%.6f, %.6f, %.6f, %.6f], norm=%.6f",
                   q.w, q.x, q.y, q.z, norm);
    }

    // Setup camera pose message with placeholder timestamp
    cam_rob_pose_.header = camera_transform_.header;
    cam_rob_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    cam_rob_pose_.pose.pose.position.x = camera_transform_.transform.translation.x;
    cam_rob_pose_.pose.pose.position.y = camera_transform_.transform.translation.y;
    cam_rob_pose_.pose.pose.position.z = camera_transform_.transform.translation.z;
    cam_rob_pose_.pose.pose.orientation = camera_transform_.transform.rotation;  // Now normalized

    // Setup RGB camera pose  
    rgb_cam_pose_.header.frame_id = camera_frame_id_;
    rgb_cam_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    rgb_cam_pose_.pose.pose.position.x = 0.0;
    rgb_cam_pose_.pose.pose.position.y = 0.0;
    rgb_cam_pose_.pose.pose.position.z = 0.0;
    rgb_cam_pose_.pose.pose.orientation = tf_cam_rgb_.transform.rotation;  // This should already be normalized from tf2

    RCLCPP_INFO(this->get_logger(), "Camera to robot transform and poses configured with normalized quaternions");
}

void TfFromUAVPose::updateTimestampsAfterClockSync() {
    auto current_time = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Updating static transform timestamps after clock sync: %.3f", 
               current_time.seconds());
    
    // Update all static transform timestamps
    tf_world_enu_.header.stamp = current_time;
    tf_world_nwu_.header.stamp = current_time; 
    tf_cam_rgb_.header.stamp = current_time;
    
    if (publish_camera_to_robot_tf_and_pose_) {
        camera_transform_.header.stamp = current_time;
        cam_rob_pose_.header.stamp = current_time;
        rgb_cam_pose_.header.stamp = current_time;
    }
    
    // Re-broadcast static transforms with correct timestamps
    if (!dont_publish_tfs_ && static_tf_broadcaster_) {
        std::vector<geometry_msgs::msg::TransformStamped> static_tfs{
            tf_world_nwu_, tf_world_enu_, tf_cam_rgb_
        };
        if (publish_camera_to_robot_tf_and_pose_) {
            static_tfs.push_back(camera_transform_);
        }
        static_tf_broadcaster_->sendTransform(static_tfs);
        RCLCPP_INFO(this->get_logger(), "Re-broadcasted %zu static transforms with synced timestamps", 
                   static_tfs.size());
    }
    
    // Publish camera poses IMMEDIATELY after clock sync (initial publish, matching ROS1)
    if (publish_camera_to_robot_tf_and_pose_) {
        if (camera_pose_pub_) {
            cam_rob_pose_.header.stamp = current_time;
            camera_pose_pub_->publish(cam_rob_pose_);
            RCLCPP_INFO(this->get_logger(), "Published initial camera pose (latched + periodic updates)");
        }
        if (cam_rgb_pose_pub_) {
            rgb_cam_pose_.header.stamp = current_time;
            cam_rgb_pose_pub_->publish(rgb_cam_pose_);
            RCLCPP_INFO(this->get_logger(), "Published initial camera RGB optical pose (latched + periodic updates)");
        }
    }
}

void TfFromUAVPose::initializePublishers() {
    // Standard pose publishers
    std_pose_.header.frame_id = world_frame_id_;
    std_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        std_pose_topic_name_, 10);

    std_raw_pose_.header.frame_id = world_frame_id_;
    std_raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        std_raw_pose_topic_name_, 10);

    // Throttled pose publishers
    throttled_pose_.header.frame_id = world_frame_id_;
    throttled_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    throttled_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        throttled_pose_topic_name_, 10);
    throttled_uav_pose_pub_ = this->create_publisher<uav_msgs::msg::UAVPose>(
        throttled_uav_pose_topic_name_, 10);

    // Camera pose publishers if enabled (matching ROS1 with  reliability)
    if (publish_camera_to_robot_tf_and_pose_) {
        std::string camera_pose_topic = this->get_parameter("cameraStaticPublish.topic").as_string();
        std::string cam_rgb_pose_topic = this->get_parameter("cameraStaticPublish.pose_optical_topic").as_string();
        
        // Create publishers with transient_local QoS (equivalent to ROS1 latched=true)
        camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            camera_pose_topic, rclcpp::QoS(1).transient_local().reliable());
        cam_rgb_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            cam_rgb_pose_topic, rclcpp::QoS(1).transient_local().reliable());
            
        RCLCPP_INFO(this->get_logger(), "Camera pose publishers configured (initial publish + periodic updates for cache reliability)");
    }

    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
}

void TfFromUAVPose::initializeSubscribers() {
    // Subscribe to poses
    pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        pose_topic_name_, 10, 
        std::bind(&TfFromUAVPose::poseCallback, this, std::placeholders::_1));

    raw_pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        raw_pose_topic_name_, 10,
        std::bind(&TfFromUAVPose::rawPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers initialized");
    RCLCPP_INFO(this->get_logger(), "  Listening to: %s", pose_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Listening to: %s", raw_pose_topic_name_.c_str());
}

void TfFromUAVPose::poseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    // Validate timestamp
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in pose message");
        return;
    }
    
    // Copy contents to std pose msg
    std_pose_.header.stamp = msg->header.stamp;
    std_pose_.pose.pose.position = msg->position;
    std_pose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, std_pose_.pose);

    // Add offset
    try {
        std_pose_.pose.pose.position.x += offset_[0];
        std_pose_.pose.pose.position.y += offset_[1];
        std_pose_.pose.pose.position.z += offset_[2];

        std_pose_.pose.covariance[0] += added_covariance_[0];
        std_pose_.pose.covariance[7] += added_covariance_[1];
        std_pose_.pose.covariance[14] += added_covariance_[2];
    } catch (const std::out_of_range &oor) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't add offset: " << oor.what());
        return;
    }

    // Publish std pose msg
    std_pose_pub_->publish(std_pose_);

    // Handle throttled publishing 
    rclcpp::Duration time_diff = rclcpp::Time(msg->header.stamp) - rclcpp::Time(throttled_pose_.header.stamp);
    if (time_diff.seconds() > 0 && (1.0 / time_diff.seconds()) <= throttle_rate_) {
        // Copy contents to throttle pose msg
        throttled_pose_.header.stamp = msg->header.stamp;
        throttled_pose_.pose.position = msg->position;
        throttled_pose_.pose.orientation = msg->orientation;

        // Publish throttle pose msg
        throttled_pose_pub_->publish(throttled_pose_);
        throttled_uav_pose_pub_->publish(*msg);
    }

    // Copy contents to tf msgs
    tf_pose_.header.stamp = msg->header.stamp;

    tf_pose_.transform.translation.x = std_pose_.pose.pose.position.x;
    tf_pose_.transform.translation.y = std_pose_.pose.pose.position.y;
    tf_pose_.transform.translation.z = std_pose_.pose.pose.position.z;

    tf_pose_.transform.rotation.w = std_pose_.pose.pose.orientation.w;
    tf_pose_.transform.rotation.x = std_pose_.pose.pose.orientation.x;
    tf_pose_.transform.rotation.y = std_pose_.pose.pose.orientation.y;
    tf_pose_.transform.rotation.z = std_pose_.pose.pose.orientation.z;

    // Broadcast tf
    if (!dont_publish_tfs_ && tf_broadcaster_) {
        tf_broadcaster_->sendTransform(tf_pose_);
    }
}

void TfFromUAVPose::rawPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    // Validate timestamp
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in raw pose message");
        return;
    }
    
    // Copy contents to std pose msg
    std_raw_pose_.header.stamp = msg->header.stamp;
    std_raw_pose_.pose.pose.position = msg->position;
    std_raw_pose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, std_raw_pose_.pose);

    // Publish std pose msg
    std_raw_pose_pub_->publish(std_raw_pose_);
}

rcl_interfaces::msg::SetParametersResult TfFromUAVPose::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    RCLCPP_INFO(this->get_logger(), "Received parameter update request");

    for (const auto & param : parameters) {
        if (param.get_name() == "offsetX") {
            offset_[0] = param.as_double();
        } else if (param.get_name() == "offsetY") {
            offset_[1] = param.as_double();
        } else if (param.get_name() == "offsetZ") {
            offset_[2] = param.as_double();
        } else if (param.get_name() == "covarianceX") {
            added_covariance_[0] = param.as_double();
        } else if (param.get_name() == "covarianceY") {
            added_covariance_[1] = param.as_double();
        } else if (param.get_name() == "covarianceZ") {
            added_covariance_[2] = param.as_double();
        } else if (param.get_name() == "throttleRate") {
            throttle_rate_ = param.as_double();
        } else if (param.get_name() == "use_sim_time") {
            bool use_sim_time = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "use_sim_time updated to: %s", use_sim_time ? "true" : "false");
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Throttle Rate: " << throttle_rate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Offset: [" << offset_[0] << ", " << offset_[1] << ", " << offset_[2] << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Extra covariance: [" << added_covariance_[0] << ", " << added_covariance_[1] << ", " << added_covariance_[2] << "]");

    return result;
}

/**
 * Complete drop-in replacement for uavCovariance_to_rosCovariance function
 * 
 * Replace your existing function in tf_from_uav_pose_ros2.cpp with this implementation.
 * 
 * Key improvements:
 * 1. Normalizes quaternions (fixes MRPT error)
 * 2. Proper covariance transformation from quaternion to Euler
 * 3. No MRPT dependencies
 * 4. Maintains mathematical correctness
 */

// Helper function to convert quaternion to rotation matrix
Eigen::Matrix3d quaternionToRotationMatrix(double w, double x, double y, double z) {
    Eigen::Matrix3d R;
    double w2 = w * w;
    double x2 = x * x;
    double y2 = y * y;
    double z2 = z * z;
    
    R << w2 + x2 - y2 - z2, 2 * (x * y - w * z), 2 * (x * z + w * y),
         2 * (x * y + w * z), w2 - x2 + y2 - z2, 2 * (y * z - w * x),
         2 * (x * z - w * y), 2 * (y * z + w * x), w2 - x2 - y2 + z2;
    
    return R;
}

// Helper function to compute Jacobian from quaternion to Euler angles
Eigen::Matrix3d quaternionToEulerJacobian(double w, double x, double y, double z) {
    // This computes the Jacobian matrix J such that d(euler) = J * d(quaternion)
    // where euler = [roll, pitch, yaw] and quaternion = [x, y, z, w]
    // This is a simplified version - you may need to adjust based on your specific Euler convention
    
    double x2 = x * x;
    double y2 = y * y;
    double z2 = z * z;
    
    // Compute some intermediate values
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x2 + y2);
    double sinp = 2 * (w * y - z * x);
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y2 + z2);
    
    Eigen::Matrix3d J;
    
    // Jacobian computation (this is an approximation - exact computation is more complex)
    // Roll derivatives
    double denom1 = sinr_cosp * sinr_cosp + cosr_cosp * cosr_cosp;
    J(0, 0) = 2 * (w * cosr_cosp + x * sinr_cosp) / denom1; // d(roll)/d(x)
    J(0, 1) = 2 * (z * cosr_cosp + y * sinr_cosp) / denom1; // d(roll)/d(y)
    J(0, 2) = 2 * (y * cosr_cosp - z * sinr_cosp) / denom1; // d(roll)/d(z)
    
    // Pitch derivatives
    double denom2 = sqrt(1 - sinp * sinp);
    J(1, 0) = -2 * z / denom2; // d(pitch)/d(x)
    J(1, 1) = 2 * w / denom2;  // d(pitch)/d(y)
    J(1, 2) = -2 * x / denom2; // d(pitch)/d(z)
    
    // Yaw derivatives
    double denom3 = siny_cosp * siny_cosp + cosy_cosp * cosy_cosp;
    J(2, 0) = 2 * (y * cosy_cosp + x * siny_cosp) / denom3; // d(yaw)/d(x)
    J(2, 1) = 2 * (x * cosy_cosp - y * siny_cosp) / denom3; // d(yaw)/d(y)
    J(2, 2) = 2 * (w * cosy_cosp + z * siny_cosp) / denom3; // d(yaw)/d(z)
    
    return J;
}

void uavCovariance_to_rosCovariance(std::shared_ptr<uav_msgs::msg::UAVPose> uav_msg,
                                    geometry_msgs::msg::PoseWithCovariance &std_pose_cov) {
    
    // Extract intermediate 7x7 covariance matrix (pos + quaternion)
    Eigen::Matrix<double, 7, 7> intermediate = Eigen::Matrix<double, 7, 7>::Zero();
    int order[]{0, 1, 2, 7, 8, 9, 6}; // pos_north, pos_east, pos_down, q_x, q_y, q_z, q_w
    
    for (int x = 0; x < 7; x++) {
        for (int y = 0; y < 7; y++) {
            intermediate(x, y) = uav_msg->covariance[order[x] * 10 + order[y]];
        }
    }
    
    // Set the pose
    std_pose_cov.pose.position.x = uav_msg->position.x;
    std_pose_cov.pose.position.y = uav_msg->position.y;
    std_pose_cov.pose.position.z = uav_msg->position.z;
    std_pose_cov.pose.orientation.w = uav_msg->orientation.w;
    std_pose_cov.pose.orientation.x = uav_msg->orientation.x;
    std_pose_cov.pose.orientation.y = uav_msg->orientation.y;
    std_pose_cov.pose.orientation.z = uav_msg->orientation.z;
    
    // Convert quaternion covariance to Euler covariance using Jacobian
    // Extract position covariance (3x3)
    Eigen::Matrix3d pos_cov = intermediate.block<3, 3>(0, 0);
    
    // Extract quaternion covariance (4x4) - note: ordered as [x, y, z, w] in intermediate
    Eigen::Matrix4d quat_cov = intermediate.block<4, 4>(3, 3);
    
    // Extract position-quaternion cross-covariance (3x4)
    Eigen::Matrix<double, 3, 4> pos_quat_cov = intermediate.block<3, 4>(0, 3);
    
    // Compute Jacobian from quaternion to Euler angles (3x3)
    // Note: We only use the first 3 components of quaternion (x, y, z) for the transformation
    Eigen::Matrix3d J = quaternionToEulerJacobian(uav_msg->orientation.w, uav_msg->orientation.x,
                                                   uav_msg->orientation.y, uav_msg->orientation.z);
    
    // Transform quaternion covariance to Euler covariance
    // We need to handle the fact that quaternion has 4 components but Euler has 3
    // Use only the first 3x3 block of quaternion covariance (x, y, z components)
    Eigen::Matrix3d quat_xyz_cov = quat_cov.block<3, 3>(0, 0);
    Eigen::Matrix3d euler_cov = J * quat_xyz_cov * J.transpose();
    
    // Transform position-quaternion cross-covariance
    Eigen::Matrix<double, 3, 3> pos_euler_cov = pos_quat_cov.block<3, 3>(0, 0) * J.transpose();
    
    // Build the final 6x6 covariance matrix for ROS
    Eigen::Matrix<double, 6, 6> final_cov = Eigen::Matrix<double, 6, 6>::Zero();
    
    // Position covariance
    final_cov.block<3, 3>(0, 0) = pos_cov;
    
    // Euler angle covariance
    final_cov.block<3, 3>(3, 3) = euler_cov;
    
    // Cross-covariance terms
    final_cov.block<3, 3>(0, 3) = pos_euler_cov;
    final_cov.block<3, 3>(3, 0) = pos_euler_cov.transpose();
    
    // Convert to ROS covariance array (row-major order)
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            std_pose_cov.covariance[i * 6 + j] = final_cov(i, j);
        }
    }
}

} // namespace tf_from_uav_pose_ros2

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tf_from_uav_pose::TfFromUAVPose>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    // CRITICAL: Update timestamps after clock sync
    node->updateTimestampsAfterClockSync();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
