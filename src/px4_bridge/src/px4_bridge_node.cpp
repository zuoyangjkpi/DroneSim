/**
 * @file px4_bridge_node.cpp
 * @brief Bridge node between DroneSim system and PX4 autopilot
 *
 * This node subscribes to DroneSim NMPC outputs and publishes to PX4 via uXRCE-DDS:
 * 1. NMPC waypoint commands (position setpoints)
 * 2. NMPC attitude commands (yaw setpoints)
 * 3. State machine status
 *
 * PX4 handles all low-level control (position→velocity→attitude→rate→motors)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

// PX4 messages - available from bundled custom_msgs/px4_msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class PX4BridgeNode : public rclcpp::Node
{
public:
    PX4BridgeNode() : Node("px4_bridge_node")
    {
        RCLCPP_INFO(this->get_logger(), "🚁 PX4 Bridge Node starting...");

        // Initialize subscribers from DroneSim system
        init_dronesim_subscribers();

        // Initialize publishers to PX4
        init_px4_publishers();

        // Timer for periodic publishing (100Hz for control loop)
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&PX4BridgeNode::control_loop, this));

        // Timer for status publishing (10Hz)
        status_timer_ = this->create_wall_timer(
            100ms, std::bind(&PX4BridgeNode::status_loop, this));

        RCLCPP_INFO(this->get_logger(), "✅ PX4 Bridge Node initialized");
        RCLCPP_INFO(this->get_logger(), "📡 Waiting for uXRCE-DDS connection to PX4...");
    }

private:
    void init_dronesim_subscribers()
    {
        // Subscribe to drone odometry feedback from PX4
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 10,
            std::bind(&PX4BridgeNode::odometry_callback, this, std::placeholders::_1));

        // Subscribe to NMPC waypoint commands (position setpoints in ENU)
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/control/waypoint_command", 10,
            std::bind(&PX4BridgeNode::waypoint_callback, this, std::placeholders::_1));

        // Subscribe to NMPC attitude commands (yaw setpoint)
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/drone/control/attitude_command", 10,
            std::bind(&PX4BridgeNode::attitude_callback, this, std::placeholders::_1));

        // Subscribe to control enable signals
        waypoint_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/control/waypoint_enable", 10,
            std::bind(&PX4BridgeNode::waypoint_enable_callback, this, std::placeholders::_1));

        attitude_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/control/attitude_enable", 10,
            std::bind(&PX4BridgeNode::attitude_enable_callback, this, std::placeholders::_1));

        // Subscribe to state machine status
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drone/state", 10,
            std::bind(&PX4BridgeNode::state_callback, this, std::placeholders::_1));

        // Subscribe to NMPC status for monitoring
        nmpc_status_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/drone/controller/status", 10,
            std::bind(&PX4BridgeNode::nmpc_status_callback, this, std::placeholders::_1));
    }

    void init_px4_publishers()
    {
        // PX4 publishers using px4_msgs
        // These topics are bridged via uXRCE-DDS to PX4 firmware

        // Offboard control mode - tells PX4 what we're controlling
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        // Trajectory setpoint - position/velocity/acceleration commands
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // Vehicle command - for arming, mode changes, etc.
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Subscribe to PX4 status
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&PX4BridgeNode::px4_status_callback, this, std::placeholders::_1));
    }

    // ========== DroneSim Callbacks ==========

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
        odom_received_ = true;
    }

    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_waypoint_ = msg;
        waypoint_received_ = true;
        last_waypoint_time_ = this->get_clock()->now();
    }

    void attitude_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        current_attitude_ = msg;
        attitude_received_ = true;
        last_attitude_time_ = this->get_clock()->now();
    }

    void waypoint_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        waypoint_enabled_ = msg->data;
        if (waypoint_enabled_) {
            RCLCPP_INFO(this->get_logger(), "✅ Waypoint control ENABLED");
        }
    }

    void attitude_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        attitude_enabled_ = msg->data;
        if (attitude_enabled_) {
            RCLCPP_INFO(this->get_logger(), "✅ Attitude control ENABLED");
        }
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string new_state = msg->data;
        if (new_state != current_state_) {
            RCLCPP_INFO(this->get_logger(), "🔄 State transition: %s → %s",
                current_state_.c_str(), new_state.c_str());
            current_state_ = new_state;
        }
    }

    void nmpc_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // msg.data: [target_detected, desired_distance, tracking_altitude, ..., current_distance?]
        if (msg->data.size() >= 3) {
            target_detected_ = (msg->data[0] > 0.5);
            desired_tracking_distance_ = msg->data[1];
            tracking_altitude_ = msg->data[2];
        }
        if (msg->data.size() >= 7) {
            current_tracking_distance_ = msg->data[6];
        } else {
            current_tracking_distance_ = desired_tracking_distance_;
        }
    }

    void px4_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        px4_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        px4_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    }

    // ========== Control Loop ==========

    void control_loop()
    {
        // Send offboard control mode (required for PX4 offboard mode)
        publish_offboard_control_mode();

        // Check for command timeouts (500ms)
        auto now = this->get_clock()->now();
        bool waypoint_timeout = waypoint_received_ &&
            (now - last_waypoint_time_).seconds() > 0.5;
        bool attitude_timeout = attitude_received_ &&
            (now - last_attitude_time_).seconds() > 0.5;

        if (waypoint_timeout) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⚠️  Waypoint command timeout!");
            waypoint_received_ = false;
        }

        if (attitude_timeout) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⚠️  Attitude command timeout!");
            attitude_received_ = false;
        }

        // Send control commands if enabled and received
        if (waypoint_enabled_ && waypoint_received_) {
            publish_position_setpoint();
        }

        offboard_counter_++;
    }

    void status_loop()
    {
        if (odom_received_ && offboard_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "📊 State: %s | PX4: %s%s | Tracking XY: %.2fm(desired)/%.2fm(current) | Alt: %.2fm",
                current_state_.c_str(),
                px4_armed_ ? "ARMED" : "DISARMED",
                px4_offboard_ ? "+OFFBOARD" : "",
                desired_tracking_distance_,
                current_tracking_distance_,
                tracking_altitude_);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Always use position control mode
        // PX4's mc_pos_control will handle position setpoints
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_pub_->publish(msg);
    }

    void publish_position_setpoint()
    {
        if (!current_waypoint_) return;

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Position setpoint from NMPC waypoint controller
        // Convert from ENU (ROS2) to NED (PX4)
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        msg.position[0] = current_waypoint_->pose.position.y;   // North (ENU Y)
        msg.position[1] = current_waypoint_->pose.position.x;   // East (ENU X)
        msg.position[2] = -current_waypoint_->pose.position.z;  // Down (negated ENU Z)

        // Yaw setpoint from attitude command if available
        if (attitude_received_ && current_attitude_) {
            // Attitude command is [roll, pitch, yaw] in ENU
            // Convert yaw from ENU to NED: yaw_NED = -yaw_ENU + π/2
            float yaw_enu = current_attitude_->vector.z;
            float yaw_ned = -yaw_enu + M_PI / 2.0;

            // Normalize to [-π, π]
            while (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
            while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;

            msg.yaw = yaw_ned;
        } else {
            // NaN signals PX4 to maintain current yaw
            msg.yaw = std::nan("");
        }

        trajectory_setpoint_pub_->publish(msg);
    }

    // ========== Member Variables ==========

    // Subscribers from DroneSim
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attitude_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nmpc_status_sub_;

    // Publishers to PX4
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscriber from PX4
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // State variables - NMPC command messages
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_waypoint_;
    geometry_msgs::msg::Vector3Stamped::SharedPtr current_attitude_;
    std::string current_state_ = "IDLE";

    // Command reception tracking
    bool odom_received_ = false;
    bool waypoint_received_ = false;
    bool attitude_received_ = false;
    bool waypoint_enabled_ = false;
    bool attitude_enabled_ = false;

    // PX4 status
    bool target_detected_ = false;
    bool px4_armed_ = false;
    bool px4_offboard_ = false;

    // Timeout tracking
    rclcpp::Time last_waypoint_time_;
    rclcpp::Time last_attitude_time_;

    // Monitoring
    double desired_tracking_distance_ = 0.0;
    double current_tracking_distance_ = 0.0;
    double tracking_altitude_ = 0.0;
    uint64_t offboard_counter_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
