#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_msgs/msg/uav_pose.hpp>

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("slam_sim_bridge") {
    // Parameters
    declare_parameter("sim_odom_topic", "/X3/odometry");
    declare_parameter("slam_pose_topic", "/machine_1/pose");
    declare_parameter("slam_command_topic", "/machine_1/command");
    declare_parameter("sim_waypoint_topic", "/target_waypoint");
    declare_parameter("sim_camera_topic", "/camera/image_raw");
    declare_parameter("slam_camera_topic", "/camera/image_raw"); // Pass-through
    declare_parameter("sim_imu_topic",
                      "/imu/data"); // Assuming standard Gazebo IMU
    declare_parameter("slam_imu_topic", "/imu");

    // Publishers
    slam_pose_pub_ = create_publisher<uav_msgs::msg::UAVPose>(
        get_parameter("slam_pose_topic").as_string(), 10);

    sim_waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        get_parameter("sim_waypoint_topic").as_string(), 10);

    slam_camera_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("slam_camera_topic").as_string(), 10);

    slam_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        get_parameter("slam_imu_topic").as_string(), 10);

    // Subscribers
    sim_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("sim_odom_topic").as_string(), 10,
        std::bind(&BridgeNode::odomCallback, this, std::placeholders::_1));

    slam_command_sub_ = create_subscription<uav_msgs::msg::UAVPose>(
        get_parameter("slam_command_topic").as_string(), 10,
        std::bind(&BridgeNode::commandCallback, this, std::placeholders::_1));

    sim_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("sim_camera_topic").as_string(), 10,
        std::bind(&BridgeNode::cameraCallback, this, std::placeholders::_1));

    sim_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("sim_imu_topic").as_string(), 10,
        std::bind(&BridgeNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "SLAM-Sim Bridge initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    uav_msgs::msg::UAVPose pose_msg;
    pose_msg.header = msg->header;
    pose_msg.position = msg->pose.pose.position;
    pose_msg.orientation = msg->pose.pose.orientation;
    // Velocity could be mapped if UAVPose has it, but it seems to be just Pose
    // + POI POI: Set to 10m in front based on orientation (simplified) For now,
    // leave POI as 0 or copy position
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

  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    slam_camera_pub_->publish(*msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    slam_imu_pub_->publish(*msg);
  }

  rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr slam_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      sim_waypoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr slam_camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr slam_imu_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_odom_sub_;
  rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr slam_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sim_camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_imu_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
