#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_msgs/msg/uav_pose.hpp>

#include <array>
#include <cmath>

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("slam_sim_bridge") {
    // Parameters - Simulation topics (outputs to Gazebo / controller)
    declare_parameter("sim_odom_topic", "/X3/odometry");   // ENU odom output
    declare_parameter("sim_imu_topic", "/imu/data");       // passthrough IMU

    // Parameters - SLAM topics (inputs from SLAM system)
    // NOTE: Camera topics are NOT bridged here.
    //       SLAM workspaces (hardware + SITL) both use:
    //         /camera/left/right/image_raw
    //         /camera/left/right/camera_info
    //       provided directly by hardware drivers or ros_gz_bridge.
    declare_parameter("slam_pose_topic", "/machine_1/pose");      // NED pose input
    declare_parameter("slam_imu_topic", "/machine_1/imu");        // FRD IMU output

    // Parameters - Command topics
    declare_parameter("slam_command_topic", "/machine_1/command");
    declare_parameter("sim_waypoint_topic", "/drone/control/waypoint_command");

    // Publisher - odometry into simulation/controller (ENU)
    sim_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        get_parameter("sim_odom_topic").as_string(), 10);

    // Publisher - waypoint into simulation controller
    sim_waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        get_parameter("sim_waypoint_topic").as_string(), 10);

    // Subscribers - from SLAM (NED pose / commands)
    slam_pose_sub_ = create_subscription<uav_msgs::msg::UAVPose>(
        get_parameter("slam_pose_topic").as_string(), 10,
        std::bind(&BridgeNode::slamPoseCallback, this, std::placeholders::_1));

    slam_command_sub_ = create_subscription<uav_msgs::msg::UAVPose>(
        get_parameter("slam_command_topic").as_string(), 10,
        std::bind(&BridgeNode::commandCallback, this, std::placeholders::_1));

    // IMU conversion: ENU/FLU (sim) -> NED/FRD (SLAM)
    sim_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("sim_imu_topic").as_string(), 10,
        std::bind(&BridgeNode::imuCallback, this, std::placeholders::_1));
    slam_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        get_parameter("slam_imu_topic").as_string(), 10);

    RCLCPP_INFO(get_logger(), "🔗 SLAM-Sim Bridge initialized");
    RCLCPP_INFO(get_logger(), "  📍 SLAM pose (NED) %s → sim odom (ENU) %s",
                get_parameter("slam_pose_topic").as_string().c_str(),
                get_parameter("sim_odom_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "  🎯 Waypoint: %s → %s",
                get_parameter("slam_command_topic").as_string().c_str(),
                get_parameter("sim_waypoint_topic").as_string().c_str());
    RCLCPP_INFO(get_logger(), "  📡 IMU passthrough: %s → %s",
                get_parameter("sim_imu_topic").as_string().c_str(),
                get_parameter("slam_imu_topic").as_string().c_str());
  }

private:
  // Convert SLAM NED pose -> Gazebo ENU odometry
  void slamPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "X3/base_link";

    // Position NED(x=north,y=east,z=down) -> ENU(x=east,y=north,z=up)
    odom_msg.pose.pose.position.x = msg->position.y;
    odom_msg.pose.pose.position.y = msg->position.x;
    odom_msg.pose.pose.position.z = -msg->position.z;

    // Orientation: R_enu = T * R_ned * T (since T^{-1} = T)
    const auto &q = msg->orientation;
    std::array<std::array<double, 3>, 3> R_ned{
        {{1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)},
         {2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)},
         {2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)}}};

    const std::array<std::array<double, 3>, 3> T{{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};

    std::array<std::array<double, 3>, 3> R_tmp{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_tmp[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_tmp[i][j] += T[i][k] * R_ned[k][j];
        }
      }
    }
    std::array<std::array<double, 3>, 3> R_enu{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_enu[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_enu[i][j] += R_tmp[i][k] * T[j][k];
        }
      }
    }

    // Rotation matrix -> quaternion
    double trace = R_enu[0][0] + R_enu[1][1] + R_enu[2][2];
    geometry_msgs::msg::Quaternion q_enu;
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      q_enu.w = 0.25 / s;
      q_enu.x = (R_enu[2][1] - R_enu[1][2]) * s;
      q_enu.y = (R_enu[0][2] - R_enu[2][0]) * s;
      q_enu.z = (R_enu[1][0] - R_enu[0][1]) * s;
    } else if (R_enu[0][0] > R_enu[1][1] && R_enu[0][0] > R_enu[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_enu[0][0] - R_enu[1][1] - R_enu[2][2]);
      q_enu.w = (R_enu[2][1] - R_enu[1][2]) / s;
      q_enu.x = 0.25 * s;
      q_enu.y = (R_enu[0][1] + R_enu[1][0]) / s;
      q_enu.z = (R_enu[0][2] + R_enu[2][0]) / s;
    } else if (R_enu[1][1] > R_enu[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_enu[1][1] - R_enu[0][0] - R_enu[2][2]);
      q_enu.w = (R_enu[0][2] - R_enu[2][0]) / s;
      q_enu.x = (R_enu[0][1] + R_enu[1][0]) / s;
      q_enu.y = 0.25 * s;
      q_enu.z = (R_enu[1][2] + R_enu[2][1]) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + R_enu[2][2] - R_enu[0][0] - R_enu[1][1]);
      q_enu.w = (R_enu[1][0] - R_enu[0][1]) / s;
      q_enu.x = (R_enu[0][2] + R_enu[2][0]) / s;
      q_enu.y = (R_enu[1][2] + R_enu[2][1]) / s;
      q_enu.z = 0.25 * s;
    }
    odom_msg.pose.pose.orientation = q_enu;

    // No velocity info from SLAM pose; leave twist zero
    sim_odom_pub_->publish(odom_msg);
  }

  void commandCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped waypoint_msg;
    waypoint_msg.header = msg->header;
    waypoint_msg.header.frame_id = "world";

    // Position NED -> ENU
    waypoint_msg.pose.position.x = msg->position.y;
    waypoint_msg.pose.position.y = msg->position.x;
    waypoint_msg.pose.position.z = -msg->position.z;

    // Orientation NED -> ENU using same T transform
    const auto &q = msg->orientation;
    std::array<std::array<double, 3>, 3> R_ned{
        {{1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)},
         {2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)},
         {2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)}}};
    const std::array<std::array<double, 3>, 3> T{{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};

    std::array<std::array<double, 3>, 3> R_tmp{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_tmp[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_tmp[i][j] += T[i][k] * R_ned[k][j];
        }
      }
    }
    std::array<std::array<double, 3>, 3> R_enu{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_enu[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_enu[i][j] += R_tmp[i][k] * T[j][k];
        }
      }
    }

    geometry_msgs::msg::Quaternion q_enu;
    double trace = R_enu[0][0] + R_enu[1][1] + R_enu[2][2];
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      q_enu.w = 0.25 / s;
      q_enu.x = (R_enu[2][1] - R_enu[1][2]) * s;
      q_enu.y = (R_enu[0][2] - R_enu[2][0]) * s;
      q_enu.z = (R_enu[1][0] - R_enu[0][1]) * s;
    } else if (R_enu[0][0] > R_enu[1][1] && R_enu[0][0] > R_enu[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_enu[0][0] - R_enu[1][1] - R_enu[2][2]);
      q_enu.w = (R_enu[2][1] - R_enu[1][2]) / s;
      q_enu.x = 0.25 * s;
      q_enu.y = (R_enu[0][1] + R_enu[1][0]) / s;
      q_enu.z = (R_enu[0][2] + R_enu[2][0]) / s;
    } else if (R_enu[1][1] > R_enu[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_enu[1][1] - R_enu[0][0] - R_enu[2][2]);
      q_enu.w = (R_enu[0][2] - R_enu[2][0]) / s;
      q_enu.x = (R_enu[0][1] + R_enu[1][0]) / s;
      q_enu.y = 0.25 * s;
      q_enu.z = (R_enu[1][2] + R_enu[2][1]) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + R_enu[2][2] - R_enu[0][0] - R_enu[1][1]);
      q_enu.w = (R_enu[1][0] - R_enu[0][1]) / s;
      q_enu.x = (R_enu[0][2] + R_enu[2][0]) / s;
      q_enu.y = (R_enu[1][2] + R_enu[2][1]) / s;
      q_enu.z = 0.25 * s;
    }
    waypoint_msg.pose.orientation = q_enu;

    sim_waypoint_pub_->publish(waypoint_msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sensor_msgs::msg::Imu imu_out = *msg;
    imu_out.header.frame_id = "machine_1/imu";

    // ENU/FLU -> NED/FRD using same T transform
    const std::array<std::array<double, 3>, 3> T{{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};

    // Orientation
    const auto &q = imu_out.orientation;
    std::array<std::array<double, 3>, 3> R_enu{
        {{1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)},
         {2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)},
         {2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)}}};
    std::array<std::array<double, 3>, 3> R_tmp{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_tmp[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_tmp[i][j] += T[i][k] * R_enu[k][j];
        }
      }
    }
    std::array<std::array<double, 3>, 3> R_ned{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R_ned[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) {
          R_ned[i][j] += R_tmp[i][k] * T[j][k];
        }
      }
    }

    geometry_msgs::msg::Quaternion q_ned;
    double trace = R_ned[0][0] + R_ned[1][1] + R_ned[2][2];
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      q_ned.w = 0.25 / s;
      q_ned.x = (R_ned[2][1] - R_ned[1][2]) * s;
      q_ned.y = (R_ned[0][2] - R_ned[2][0]) * s;
      q_ned.z = (R_ned[1][0] - R_ned[0][1]) * s;
    } else if (R_ned[0][0] > R_ned[1][1] && R_ned[0][0] > R_ned[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_ned[0][0] - R_ned[1][1] - R_ned[2][2]);
      q_ned.w = (R_ned[2][1] - R_ned[1][2]) / s;
      q_ned.x = 0.25 * s;
      q_ned.y = (R_ned[0][1] + R_ned[1][0]) / s;
      q_ned.z = (R_ned[0][2] + R_ned[2][0]) / s;
    } else if (R_ned[1][1] > R_ned[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R_ned[1][1] - R_ned[0][0] - R_ned[2][2]);
      q_ned.w = (R_ned[0][2] - R_ned[2][0]) / s;
      q_ned.x = (R_ned[0][1] + R_ned[1][0]) / s;
      q_ned.y = 0.25 * s;
      q_ned.z = (R_ned[1][2] + R_ned[2][1]) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + R_ned[2][2] - R_ned[0][0] - R_ned[1][1]);
      q_ned.w = (R_ned[1][0] - R_ned[0][1]) / s;
      q_ned.x = (R_ned[0][2] + R_ned[2][0]) / s;
      q_ned.y = (R_ned[1][2] + R_ned[2][1]) / s;
      q_ned.z = 0.25 * s;
    }
    imu_out.orientation = q_ned;

    // Angular velocity and linear acceleration ENU/FLU -> NED/FRD
    auto transform_vec = [&T](const geometry_msgs::msg::Vector3 &v) {
      geometry_msgs::msg::Vector3 out;
      out.x = T[0][0] * v.x + T[0][1] * v.y + T[0][2] * v.z;
      out.y = T[1][0] * v.x + T[1][1] * v.y + T[1][2] * v.z;
      out.z = T[2][0] * v.x + T[2][1] * v.y + T[2][2] * v.z;
      return out;
    };
    imu_out.angular_velocity = transform_vec(imu_out.angular_velocity);
    imu_out.linear_acceleration = transform_vec(imu_out.linear_acceleration);

    slam_imu_pub_->publish(imu_out);
  }

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sim_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr slam_imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sim_waypoint_pub_;

  // Subscribers
  rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr slam_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_imu_sub_;
  rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr slam_command_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
