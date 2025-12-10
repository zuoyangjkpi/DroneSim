#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_msgs/msg/uav_pose.hpp>

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("slam_sim_bridge") {
    // Parameters - Simulation topics (inputs from Gazebo)
    declare_parameter("sim_odom_topic", "/X3/odometry");
    declare_parameter("sim_camera_left_topic", "/camera/left/image_raw");
    declare_parameter("sim_camera_right_topic", "/camera/right/image_raw");
    declare_parameter("sim_camera_left_info_topic", "/camera/left/camera_info");
    declare_parameter("sim_camera_right_info_topic", "/camera/right/camera_info");
    declare_parameter("sim_imu_topic", "/imu/data");

    // Parameters - SLAM topics (outputs to SLAM system)
    // Publish to ORB-SLAM3 internal topics (before remapping)
    // Launch file will remap: /camera1 → /cam0/image_raw, etc.
    declare_parameter("slam_pose_topic", "/machine_1/pose");
    declare_parameter("slam_camera_left_topic", "/camera1");
    declare_parameter("slam_camera_right_topic", "/camera2");
    declare_parameter("slam_camera_left_info_topic", "/camera1/camera_info");
    declare_parameter("slam_camera_right_info_topic", "/camera2/camera_info");
    declare_parameter("slam_imu_topic", "/imu");

    // Parameters - Command topics
    declare_parameter("slam_command_topic", "/machine_1/command");
    declare_parameter("sim_waypoint_topic", "/drone/control/waypoint_command");

    // Publishers - to SLAM system
    slam_pose_pub_ = create_publisher<uav_msgs::msg::UAVPose>(
        get_parameter("slam_pose_topic").as_string(), 10);

    slam_camera_left_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("slam_camera_left_topic").as_string(), 10);
    slam_camera_right_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("slam_camera_right_topic").as_string(), 10);
    slam_camera_left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        get_parameter("slam_camera_left_info_topic").as_string(), 10);
    slam_camera_right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        get_parameter("slam_camera_right_info_topic").as_string(), 10);

    slam_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        get_parameter("slam_imu_topic").as_string(), 10);

    // Publishers - to simulation controller
    sim_waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        get_parameter("sim_waypoint_topic").as_string(), 10);

    // Subscribers - from simulation (Gazebo)
    sim_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("sim_odom_topic").as_string(), 10,
        std::bind(&BridgeNode::odomCallback, this, std::placeholders::_1));

    sim_camera_left_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("sim_camera_left_topic").as_string(), 10,
        std::bind(&BridgeNode::cameraLeftCallback, this, std::placeholders::_1));
    sim_camera_right_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("sim_camera_right_topic").as_string(), 10,
        std::bind(&BridgeNode::cameraRightCallback, this, std::placeholders::_1));

    sim_camera_left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("sim_camera_left_info_topic").as_string(), 10,
        std::bind(&BridgeNode::cameraLeftInfoCallback, this, std::placeholders::_1));
    sim_camera_right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("sim_camera_right_info_topic").as_string(), 10,
        std::bind(&BridgeNode::cameraRightInfoCallback, this, std::placeholders::_1));

    sim_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("sim_imu_topic").as_string(), 10,
        std::bind(&BridgeNode::imuCallback, this, std::placeholders::_1));

    // Subscribers - from SLAM command
    slam_command_sub_ = create_subscription<uav_msgs::msg::UAVPose>(
        get_parameter("slam_command_topic").as_string(), 10,
        std::bind(&BridgeNode::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "🔗 SLAM-Sim Bridge initialized");
    RCLCPP_INFO(get_logger(), "  📷 Stereo Camera:");
    RCLCPP_INFO(get_logger(), "     Gazebo → SLAM:");
    RCLCPP_INFO(get_logger(), "       %s → %s",
                get_parameter("sim_camera_left_topic").as_string().c_str(),
                get_parameter("slam_camera_left_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "       %s → %s",
                get_parameter("sim_camera_right_topic").as_string().c_str(),
                get_parameter("slam_camera_right_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "  📍 Odometry: %s → %s",
                get_parameter("sim_odom_topic").as_string().c_str(),
                get_parameter("slam_pose_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "  🎯 Waypoint: %s → %s",
                get_parameter("slam_command_topic").as_string().c_str(),
                get_parameter("sim_waypoint_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "  📡 IMU: %s → %s",
                get_parameter("sim_imu_topic").as_string().c_str(),
                get_parameter("slam_imu_topic").as_string().c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    uav_msgs::msg::UAVPose pose_msg;
    pose_msg.header = msg->header;
    pose_msg.position = msg->pose.pose.position;
    pose_msg.orientation = msg->pose.pose.orientation;
    // POI: Set to 10m in front based on orientation (simplified)
    // For now, copy position as POI
    pose_msg.poi = msg->pose.pose.position;

    slam_pose_pub_->publish(pose_msg);
  }

  void commandCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped waypoint_msg;
    waypoint_msg.header = msg->header;
    // Ensure frame_id is correct for controller (usually "world" or "map")
    if (waypoint_msg.header.frame_id.empty()) {
      waypoint_msg.header.frame_id = "world";
    }
    waypoint_msg.pose.position = msg->position;
    waypoint_msg.pose.orientation = msg->orientation;

    sim_waypoint_pub_->publish(waypoint_msg);
  }

  void cameraLeftCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    slam_camera_left_pub_->publish(*msg);
  }

  void cameraRightCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    slam_camera_right_pub_->publish(*msg);
  }

  void cameraLeftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    slam_camera_left_info_pub_->publish(*msg);
  }

  void cameraRightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    slam_camera_right_info_pub_->publish(*msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    slam_imu_pub_->publish(*msg);
  }

  // Publishers - to SLAM
  rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr slam_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr slam_camera_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr slam_camera_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr slam_camera_left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr slam_camera_right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr slam_imu_pub_;

  // Publishers - to simulation
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sim_waypoint_pub_;

  // Subscribers - from simulation
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sim_camera_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sim_camera_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sim_camera_left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sim_camera_right_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_imu_sub_;

  // Subscribers - from SLAM
  rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr slam_command_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
