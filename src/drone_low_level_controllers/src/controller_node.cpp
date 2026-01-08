#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <mutex>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

// ==========================================
// Data Structures
// ==========================================

struct ImuData {
  Quaterniond orientation;
  Vector3d angular_velocity;
  rclcpp::Time timestamp;
  bool valid = false;
};

struct OdomData {
  Vector3d velocity_world;
  Quaterniond orientation;
  rclcpp::Time timestamp;
  bool valid = false;
};

struct ControllerGains {
  // Outer loop: world velocity PID
  double kp_vel_xy;
  double ki_vel_xy;
  double kd_vel_xy;
  double kp_vel_z;
  double ki_vel_z;
  double kd_vel_z;

  double vel_integral_limit;
  double vel_d_filter_tau;

  // Attitude P
  double kp_att_roll;
  double kp_att_pitch;
  double kp_att_yaw;

  // Rate PID
  double kp_rate_roll;
  double ki_rate_roll;
  double kd_rate_roll;
  double kp_rate_pitch;
  double ki_rate_pitch;
  double kd_rate_pitch;
  double kp_rate_yaw;
  double ki_rate_yaw;
  double kd_rate_yaw;

  double rate_integral_limit;

  // Limits
  double max_tilt_angle;
  double max_thrust;
  double min_thrust;

  double max_acc_xy;
  double max_acc_z_up;
  double max_acc_z_down;

  double max_torque_xy;
  double max_torque_z;

  double max_rate_xy;
  double max_rate_yaw;

  double thrust_slew_rate;
};

struct VehicleParameters {
  double mass;
  double gravity;
  double inertia_xx;
  double inertia_yy;
  double inertia_zz;

  double force_constant;
  double moment_constant;
  double max_motor_speed;
  double min_motor_speed;
  int num_motors;
};

// ==========================================
// Controller Implementation
// ==========================================

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("multicopter_controller_node") {
    this->declare_parameters();
    this->load_parameters();

    // Publishers
    motor_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
        "/X3/command/motor_speed", 10);
    debug_accel_pub_ =
        this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/drone/debug/accel_setpoint", 10);
    debug_att_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/drone/debug/att_setpoint", 10);
    debug_mixer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/drone/debug/mixer_input", 10);

    // Subscribers

    // VSP (Command)
    vsp_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/drone/control/velocity_setpoint", 10,
        std::bind(&ControllerNode::vsp_callback, this, std::placeholders::_1));

    // WSP (Command)
    wsp_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/drone/control/angular_velocity_setpoint", 10,
        std::bind(&ControllerNode::wsp_callback, this, std::placeholders::_1));

    // Enable
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/drone/control/velocity_enable", 10,
        std::bind(&ControllerNode::enable_callback, this,
                  std::placeholders::_1));

    // Odametry (50Hz) -> Drives Velocity Loop
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/X3/odometry", 10,
        std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1));

    // IMU (100Hz) -> Drives Attitude/Rate Loop
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, // Assuming bridge or SDF provides this
        std::bind(&ControllerNode::imu_callback, this, std::placeholders::_1));

    // Tuning Subscribers
    tuning_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/drone/debug/mode", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          tuning_mode_ = msg->data;
          RCLCPP_INFO(this->get_logger(), "Tuning Mode set to: %d",
                      tuning_mode_);
        });

    cmd_att_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/drone/debug/cmd_att", 10,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
          if (tuning_mode_ == 1 || tuning_mode_ == 3) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            cached_att_sp_ << msg->x, msg->y, msg->z;
          }
        });

    cmd_rate_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/drone/debug/cmd_rate", 10,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
          if (tuning_mode_ == 2) {
            tuning_rate_sp_ = Vector3d(msg->x, msg->y, msg->z);
          }
        });

    cmd_thrust_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/drone/debug/cmd_thrust", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (tuning_mode_ == 1 || tuning_mode_ == 2 || tuning_mode_ == 3) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            cached_thrust_ =
                std::clamp(msg->data, g_.min_thrust, g_.max_thrust);
          }
        });

    // Init Mixer
    build_mixer();

    // Read control loop frequencies
    double outer_freq = this->get_parameter("outer_loop_frequency").as_double();
    double inner_freq = this->get_parameter("inner_loop_frequency").as_double();

    // Validate frequencies
    if (outer_freq < 10.0 || outer_freq > 500.0) {
      RCLCPP_WARN(
          this->get_logger(),
          "outer_loop_frequency %.1f Hz out of range [10-500], using 100 Hz",
          outer_freq);
      outer_freq = 100.0;
    }
    if (inner_freq < 50.0 || inner_freq > 2000.0) {
      RCLCPP_WARN(
          this->get_logger(),
          "inner_loop_frequency %.1f Hz out of range [50-2000], using 500 Hz",
          inner_freq);
      inner_freq = 500.0;
    }
    if (inner_freq <= outer_freq) {
      RCLCPP_WARN(this->get_logger(),
                  "inner_loop_frequency (%.1f Hz) must be > "
                  "outer_loop_frequency (%.1f Hz), adjusting to %.1f Hz",
                  inner_freq, outer_freq, outer_freq * 2.0);
      inner_freq = outer_freq * 2.0;
    }

    // Create Timers with configurable frequencies
    int outer_period_us = static_cast<int>(1e6 / outer_freq);
    int inner_period_us = static_cast<int>(1e6 / inner_freq);

    outer_loop_timer_ = this->create_wall_timer(
        std::chrono::microseconds(outer_period_us),
        std::bind(&ControllerNode::outer_loop_timer_callback, this));

    inner_loop_timer_ = this->create_wall_timer(
        std::chrono::microseconds(inner_period_us),
        std::bind(&ControllerNode::inner_loop_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Controller (C++) initialized with DUAL-TIMER Mode:");
    RCLCPP_INFO(this->get_logger(),
                "  - Outer Loop: %.1f Hz (%.2f ms, Velocity Control)",
                outer_freq, 1000.0 / outer_freq);
    RCLCPP_INFO(this->get_logger(),
                "  - Inner Loop: %.1f Hz (%.2f ms, Attitude + Rate Control)",
                inner_freq, 1000.0 / inner_freq);
    RCLCPP_INFO(this->get_logger(), "  - Sensor callbacks: Cache data only");
  }

private:
  // ----------------------------------------------------------------------
  // Parameters
  // ----------------------------------------------------------------------
  ControllerGains g_;
  VehicleParameters p_;
  bool odom_twist_in_world_;

  // ----------------------------------------------------------------------
  // State
  // ----------------------------------------------------------------------
  // State
  // ----------------------------------------------------------------------
  bool enabled_ = true;
  int tuning_mode_ = 0; // 0: Normal, 1/3: Attitude, 2: Rate, 4: Velocity, 5: Position
  Vector3d tuning_rate_sp_ =
      Vector3d::Zero(); // Direct rate setpoint for mode 2

  // Sensor Data Buffers (Thread-safe)
  std::mutex imu_mutex_;
  ImuData latest_imu_;

  std::mutex odom_mutex_;
  OdomData latest_odom_;

  // Setpoints
  Vector3d v_sp_w_ = Vector3d::Zero();
  double yaw_rate_sp_ = 0.0;

  // Timeouts
  rclcpp::Time last_vsp_time_;
  rclcpp::Time last_wsp_time_;
  double vsp_timeout_;
  double wsp_timeout_;

  // Velocity Loop State
  Vector3d vel_int_err_ = Vector3d::Zero();
  Vector3d vel_prev_err_ = Vector3d::Zero();
  Vector3d vel_prev_d_ = Vector3d::Zero();
  rclcpp::Time last_vel_loop_time_;

  // Cached Outer Loop Outputs (for Inner Loop)
  std::mutex cache_mutex_; // Protects cached_att_sp_ and cached_thrust_
  Vector3d cached_att_sp_ = Vector3d::Zero(); // Roll, Pitch, Yaw
  double cached_thrust_ = 0.0;

  double thrust_prev_ = 0.0;

  // Rate Loop State
  Vector3d rate_int_err_ = Vector3d::Zero();
  Vector3d rate_prev_err_ = Vector3d::Zero();
  rclcpp::Time last_rate_loop_time_;

  // Mixer
  Eigen::MatrixXd mixer_pinv_;
  Eigen::VectorXd motor_directions_;

  // ----------------------------------------------------------------------
  // Logic
  // ----------------------------------------------------------------------

  void declare_parameters() {
    // Control loop frequencies
    this->declare_parameter("outer_loop_frequency", 100.0);
    this->declare_parameter("inner_loop_frequency", 500.0);

    // Timeouts
    this->declare_parameter("velocity_setpoint_timeout", 0.8);
    this->declare_parameter("angular_setpoint_timeout", 0.8);
    this->declare_parameter("control_enabled",
                            false); // Controlled by action modules
    this->declare_parameter("odom_twist_in_world", true);

    // Velocity PID
    this->declare_parameter("kp_vel_xy", 3.0);
    this->declare_parameter("ki_vel_xy", 0.0);
    this->declare_parameter("kd_vel_xy", 0.4);
    this->declare_parameter("kp_vel_z", 4.0);
    this->declare_parameter("ki_vel_z", 0.0);
    this->declare_parameter("kd_vel_z", 0.6);
    this->declare_parameter("vel_integral_limit", 3.0);
    this->declare_parameter("vel_d_filter_tau", 0.05);

    // Attitude P
    this->declare_parameter("kp_att_roll", 5.0);
    this->declare_parameter("kp_att_pitch", 5.0);
    this->declare_parameter("kp_att_yaw", 3.0);

    // Rate PID
    this->declare_parameter("kp_rate_roll", 0.20);
    this->declare_parameter("ki_rate_roll", 0.02);
    this->declare_parameter("kd_rate_roll", 0.00);
    this->declare_parameter("kp_rate_pitch", 0.20);
    this->declare_parameter("ki_rate_pitch", 0.02);
    this->declare_parameter("kd_rate_pitch", 0.00);
    this->declare_parameter("kp_rate_yaw", 0.15);
    this->declare_parameter("ki_rate_yaw", 0.02);
    this->declare_parameter("kd_rate_yaw", 0.00);
    this->declare_parameter("rate_integral_limit", 3.0);

    // Limits
    this->declare_parameter("max_tilt_angle", 0.5);
    this->declare_parameter("max_thrust", 150.0);
    this->declare_parameter("min_thrust", 0.0);
    this->declare_parameter("max_acc_xy", 3.0);
    this->declare_parameter("max_acc_z_up", 5.0);
    this->declare_parameter("max_acc_z_down", 3.0);
    this->declare_parameter("max_torque_xy", 3.0);
    this->declare_parameter("max_torque_z", 2.0);
    this->declare_parameter("max_rate_xy", 4.0);
    this->declare_parameter("max_rate_yaw", 3.0);
    this->declare_parameter("thrust_slew_rate", 0.0);

    // Vehicle
    this->declare_parameter("vehicle_mass", 4.6);
    this->declare_parameter("gravity", 9.81);
    this->declare_parameter("inertia_xx", 0.423);
    this->declare_parameter("inertia_yy", 0.423);
    this->declare_parameter("inertia_zz", 0.828);
    this->declare_parameter("force_constant", 2.85e-05);
    this->declare_parameter("moment_constant", 0.0533);
    this->declare_parameter("max_motor_speed", 800.0);
    this->declare_parameter("min_motor_speed", 0.0);
    this->declare_parameter("num_motors", 8);
  }

  void load_parameters() {
    vsp_timeout_ = this->get_parameter("velocity_setpoint_timeout").as_double();
    wsp_timeout_ = this->get_parameter("angular_setpoint_timeout").as_double();
    enabled_ = this->get_parameter("control_enabled").as_bool();
    odom_twist_in_world_ = this->get_parameter("odom_twist_in_world").as_bool();

    g_.kp_vel_xy = this->get_parameter("kp_vel_xy").as_double();
    g_.ki_vel_xy = this->get_parameter("ki_vel_xy").as_double();
    g_.kd_vel_xy = this->get_parameter("kd_vel_xy").as_double();
    g_.kp_vel_z = this->get_parameter("kp_vel_z").as_double();
    g_.ki_vel_z = this->get_parameter("ki_vel_z").as_double();
    g_.kd_vel_z = this->get_parameter("kd_vel_z").as_double();
    g_.vel_integral_limit =
        this->get_parameter("vel_integral_limit").as_double();
    g_.vel_d_filter_tau = this->get_parameter("vel_d_filter_tau").as_double();

    g_.kp_att_roll = this->get_parameter("kp_att_roll").as_double();
    g_.kp_att_pitch = this->get_parameter("kp_att_pitch").as_double();
    g_.kp_att_yaw = this->get_parameter("kp_att_yaw").as_double();

    g_.kp_rate_roll = this->get_parameter("kp_rate_roll").as_double();
    g_.ki_rate_roll = this->get_parameter("ki_rate_roll").as_double();
    g_.kd_rate_roll = this->get_parameter("kd_rate_roll").as_double();
    g_.kp_rate_pitch = this->get_parameter("kp_rate_pitch").as_double();
    g_.ki_rate_pitch = this->get_parameter("ki_rate_pitch").as_double();
    g_.kd_rate_pitch = this->get_parameter("kd_rate_pitch").as_double();
    g_.kp_rate_yaw = this->get_parameter("kp_rate_yaw").as_double();
    g_.ki_rate_yaw = this->get_parameter("ki_rate_yaw").as_double();
    g_.kd_rate_yaw = this->get_parameter("kd_rate_yaw").as_double();
    g_.rate_integral_limit =
        this->get_parameter("rate_integral_limit").as_double();

    g_.max_tilt_angle = this->get_parameter("max_tilt_angle").as_double();
    g_.max_thrust = this->get_parameter("max_thrust").as_double();
    g_.min_thrust = this->get_parameter("min_thrust").as_double();
    g_.max_acc_xy = this->get_parameter("max_acc_xy").as_double();
    g_.max_acc_z_up = this->get_parameter("max_acc_z_up").as_double();
    g_.max_acc_z_down = this->get_parameter("max_acc_z_down").as_double();
    g_.max_torque_xy = this->get_parameter("max_torque_xy").as_double();
    g_.max_torque_z = this->get_parameter("max_torque_z").as_double();
    g_.max_rate_xy = this->get_parameter("max_rate_xy").as_double();
    g_.max_rate_yaw = this->get_parameter("max_rate_yaw").as_double();
    g_.thrust_slew_rate = this->get_parameter("thrust_slew_rate").as_double();

    p_.mass = this->get_parameter("vehicle_mass").as_double();
    p_.gravity = this->get_parameter("gravity").as_double();
    p_.inertia_xx = this->get_parameter("inertia_xx").as_double();
    p_.inertia_yy = this->get_parameter("inertia_yy").as_double();
    p_.inertia_zz = this->get_parameter("inertia_zz").as_double();
    p_.force_constant = this->get_parameter("force_constant").as_double();
    p_.moment_constant = this->get_parameter("moment_constant").as_double();
    p_.max_motor_speed = this->get_parameter("max_motor_speed").as_double();
    p_.min_motor_speed = this->get_parameter("min_motor_speed").as_double();
    p_.num_motors = this->get_parameter("num_motors").as_int();
  }

  void build_mixer() {
    // Hardcoded X8 geometry from python script
    int n = p_.num_motors;
    Eigen::MatrixXd motor_pos(n, 3);
    motor_pos << 0.6, 0.0, 0.0, 0.4243, 0.4243, 0.0, 0.0, 0.6, 0.0, -0.4243,
        0.4243, 0.0, -0.6, 0.0, 0.0, -0.4243, -0.4243, 0.0, 0.0, -0.6, 0.0,
        0.4243, -0.4243, 0.0;

    // FIXED MOTOR DIRECTIONS (Inverted from original python to fix yaw issue)
    // Original: +1, -1... -> Yaw unstable
    // Fixed:    -1, +1... -> Yaw stable
    motor_directions_ = Eigen::VectorXd(n);
    motor_directions_ << -1, 1, -1, 1, -1, 1, -1, 1;

    double kf = p_.force_constant;
    double km = kf * p_.moment_constant;

    Eigen::MatrixXd A(4, n);
    for (int i = 0; i < n; ++i) {
      double x = motor_pos(i, 0);
      double y = motor_pos(i, 1);
      double d = motor_directions_(i);

      A(0, i) = kf;
      A(1, i) = kf * y;
      A(2, i) = kf * (-x);
      A(3, i) = km * d;
    }

    // Pseudo-inverse
    mixer_pinv_ = A.completeOrthogonalDecomposition().pseudoInverse();
  }

  void enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    enabled_ = msg->data;
    if (!enabled_) {
      // Reset integrators
      vel_int_err_.setZero();
      rate_int_err_.setZero();
      vel_prev_err_.setZero();
      rate_prev_err_.setZero();
      publish_motors(Eigen::VectorXd::Zero(p_.num_motors));
      RCLCPP_INFO(this->get_logger(), "Controller DISABLED (motors stopped)");
    } else {
      RCLCPP_INFO(this->get_logger(), "Controller ENABLED by action module");
    }
  }

  void vsp_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    v_sp_w_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    last_vsp_time_ = this->get_clock()->now();
  }

  void wsp_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    yaw_rate_sp_ = msg->vector.z;
    last_wsp_time_ = this->get_clock()->now();
  }

  // ==========================================
  // ODOMETRY Callback: Cache sensor data only
  // ==========================================
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    // Current Velocity (in body or world frame depending on parameter)
    Vector3d v_raw(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z);

    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // Convert to world frame if needed
    if (!odom_twist_in_world_) {
      Matrix3d R_w_b = q.toRotationMatrix();
      latest_odom_.velocity_world = R_w_b * v_raw;
    } else {
      latest_odom_.velocity_world = v_raw;
    }

    latest_odom_.orientation = q;
    latest_odom_.timestamp = this->get_clock()->now();
    latest_odom_.valid = true;
  }

  // ==========================================
  // IMU Callback: Cache sensor data only
  // ==========================================
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_.orientation =
        Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                    msg->orientation.z);
    latest_imu_.angular_velocity =
        Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                 msg->angular_velocity.z);
    latest_imu_.timestamp = this->get_clock()->now();
    latest_imu_.valid = true;
  }

  // ==========================================
  // OUTER LOOP TIMER: 100Hz Velocity Control
  // ==========================================
  void outer_loop_timer_callback() {
    if (!enabled_)
      return;

    rclcpp::Time now = this->get_clock()->now();

    // TUNING MODE 1/2/3: Bypass Velocity Loop
    if (tuning_mode_ == 1 || tuning_mode_ == 2 || tuning_mode_ == 3) {
      // In tuning mode, we rely on callbacks updating cached_att_sp_ or we skip
      // to inner loop Just publish debug accel as zero or keep last
      return;
    }

    // Check setpoint timeout
    if (last_vsp_time_.nanoseconds() == 0 ||
        (now - last_vsp_time_).seconds() > vsp_timeout_) {
      v_sp_w_.setZero();
    }

    // Get latest odometry data (thread-safe)
    OdomData odom_local;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!latest_odom_.valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Outer loop: No valid odometry data");
        return;
      }
      odom_local = latest_odom_;
    }

    // Check sensor timeout (20ms = 50Hz expectation)
    if ((now - odom_local.timestamp).seconds() > 0.05) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Outer loop: Odometry data too old (%.3f s)",
                           (now - odom_local.timestamp).seconds());
      return;
    }

    // Time delta
    double dt = (last_vel_loop_time_.nanoseconds() == 0)
                    ? 0.01 // 100Hz = 10ms
                    : (now - last_vel_loop_time_).seconds();
    last_vel_loop_time_ = now;
    dt = std::clamp(dt, 0.001, 0.1); // Clamp to reasonable range

    // 1. Velocity PID
    Vector3d v_w = odom_local.velocity_world;
    Vector3d err = v_sp_w_ - v_w;

    Vector3d kp(g_.kp_vel_xy, g_.kp_vel_xy, g_.kp_vel_z);
    Vector3d ki(g_.ki_vel_xy, g_.ki_vel_xy, g_.ki_vel_z);
    Vector3d kd(g_.kd_vel_xy, g_.kd_vel_xy, g_.kd_vel_z);

    // Integral
    vel_int_err_ += err * dt;
    vel_int_err_ = vel_int_err_.cwiseMax(-g_.vel_integral_limit)
                       .cwiseMin(g_.vel_integral_limit);

    // Derivative (with LPF)
    Vector3d d_raw = (err - vel_prev_err_) / dt;
    vel_prev_err_ = err;

    double rc = std::max(0.0001, g_.vel_d_filter_tau);
    double alpha_py = std::exp(-dt / rc);
    Vector3d d_filt = alpha_py * vel_prev_d_ + (1.0 - alpha_py) * d_raw;
    vel_prev_d_ = d_filt;

    Vector3d a_cmd = kp.cwiseProduct(err) + ki.cwiseProduct(vel_int_err_) +
                     kd.cwiseProduct(d_filt);

    // Accel Saturation
    Vector2d a_xy = a_cmd.head<2>();
    double a_norm = a_xy.norm();
    if (a_norm > g_.max_acc_xy) {
      a_cmd.head<2>() = a_xy / a_norm * g_.max_acc_xy;
    }
    a_cmd.z() = std::clamp(a_cmd.z(), -g_.max_acc_z_down, g_.max_acc_z_up);

    // Gravity Compensation
    Vector3d F_w = a_cmd * p_.mass;
    F_w.z() += p_.mass * p_.gravity;
    F_w.z() = std::max(0.0, F_w.z());

    // 2. Force -> Attitude & Thrust
    Matrix3d R_w_b = odom_local.orientation.toRotationMatrix();
    double yaw_curr = std::atan2(R_w_b(1, 0), R_w_b(0, 0));

    // Debug accel
    {
      auto msg = geometry_msgs::msg::Vector3Stamped();
      msg.header.stamp = now;
      msg.header.frame_id = "world";
      msg.vector.x = a_cmd.x();
      msg.vector.y = a_cmd.y();
      msg.vector.z = a_cmd.z();
      debug_accel_pub_->publish(msg);
    }

    // Desired body z-axis direction
    double F_norm = F_w.norm();
    Vector3d b3;
    if (F_norm < 1e-3) {
      b3 = Vector3d::UnitZ();
    } else {
      b3 = F_w / F_norm;
    }

    // Tilt limit
    double max_tilt = g_.max_tilt_angle;
    if (std::acos(std::clamp(b3.z(), -1.0, 1.0)) > max_tilt) {
      Vector2d h = b3.head<2>();
      double h_norm = h.norm();
      if (h_norm > 1e-6) {
        b3.head<2>() = h / h_norm * std::sin(max_tilt);
        b3.z() = std::cos(max_tilt);
      }
    }

    // Reconstruct rotation matrix
    Vector3d b1_yaw(std::cos(yaw_curr), std::sin(yaw_curr), 0.0);
    Vector3d b2 = b3.cross(b1_yaw);
    if (b2.norm() < 1e-6) {
      b2 = b3.cross(Vector3d(-std::sin(yaw_curr), std::cos(yaw_curr), 0.0));
    }
    b2.normalize();
    Vector3d b1 = b2.cross(b3);

    Matrix3d R_sp;
    R_sp.col(0) = b1;
    R_sp.col(1) = b2;
    R_sp.col(2) = b3;

    // Extract Euler angles (ZYX)
    double pitch_sp = std::asin(std::clamp(-R_sp(2, 0), -1.0, 1.0));
    double roll_sp = std::atan2(R_sp(2, 1), R_sp(2, 2));

    double thrust_cmd = F_w.dot(b3);
    thrust_cmd = std::clamp(thrust_cmd, g_.min_thrust, g_.max_thrust);

    // Thrust slew rate limiting
    if (g_.thrust_slew_rate > 0.0) {
      double delta = thrust_cmd - thrust_prev_;
      double max_delta = g_.thrust_slew_rate * dt;
      delta = std::clamp(delta, -max_delta, max_delta);
      thrust_cmd = thrust_prev_ + delta;
    }
    thrust_prev_ = thrust_cmd;

    // Cache for inner loop (thread-safe write)
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cached_att_sp_ << roll_sp, pitch_sp, yaw_curr;
      cached_thrust_ = thrust_cmd;
    }

    // Debug attitude
    {
      auto msg = geometry_msgs::msg::Vector3Stamped();
      msg.header.stamp = now;
      msg.header.frame_id = "world";
      msg.vector.x = roll_sp;
      msg.vector.y = pitch_sp;
      msg.vector.z = yaw_curr;
      debug_att_pub_->publish(msg);
    }
  }

  // ==========================================
  // INNER LOOP TIMER: 500Hz Attitude + Rate Control
  // ==========================================
  void inner_loop_timer_callback() {
    if (!enabled_)
      return;

    rclcpp::Time now = this->get_clock()->now();

    // TUNING MODE 2: Bypass Attitude P, Use Rate Setpoint Directly
    if (tuning_mode_ == 2) {
      // We still need valid IMU data for rate error
      // Get latest IMU data (thread-safe)
      ImuData imu_local;
      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!latest_imu_.valid)
          return;
        imu_local = latest_imu_;
      }

      // Get thrust (updated by debug topic)
      double thrust_local;
      {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        thrust_local = cached_thrust_;
      }

      // Time delta logic
      double dt = (last_rate_loop_time_.nanoseconds() == 0)
                      ? 0.002
                      : (now - last_rate_loop_time_).seconds();
      last_rate_loop_time_ = now;
      dt = std::clamp(dt, 0.0001, 0.01);

      // Current Rate
      Vector3d pqr = imu_local.angular_velocity;

      // Use direct rate setpoint from tuning topic
      Vector3d pqr_sp = tuning_rate_sp_;

      // 4. Rate PID -> Torque (Duplicated logic, could be refactored but
      // keeping inline for clarity)
      Vector3d rate_err = pqr_sp - pqr;

      rate_int_err_ += rate_err * dt;
      rate_int_err_ = rate_int_err_.cwiseMax(-g_.rate_integral_limit)
                          .cwiseMin(g_.rate_integral_limit);

      Vector3d rate_d = (rate_err - rate_prev_err_) / dt;
      rate_prev_err_ = rate_err;

      Vector3d kp_r(g_.kp_rate_roll, g_.kp_rate_pitch, g_.kp_rate_yaw);
      Vector3d ki_r(g_.ki_rate_roll, g_.ki_rate_pitch, g_.ki_rate_yaw);
      Vector3d kd_r(g_.kd_rate_roll, g_.kd_rate_pitch, g_.kd_rate_yaw);

      Vector3d alpha_cmd = kp_r.cwiseProduct(rate_err) +
                           ki_r.cwiseProduct(rate_int_err_) +
                           kd_r.cwiseProduct(rate_d);

      Vector3d I_vec(p_.inertia_xx, p_.inertia_yy, p_.inertia_zz);
      Vector3d tau = alpha_cmd.cwiseProduct(I_vec);

      // Torque Saturation
      tau.x() = std::clamp(tau.x(), -g_.max_torque_xy, g_.max_torque_xy);
      tau.y() = std::clamp(tau.y(), -g_.max_torque_xy, g_.max_torque_xy);
      tau.z() = std::clamp(tau.z(), -g_.max_torque_z, g_.max_torque_z);

      // 5. Mixer
      Eigen::Vector4d mix_cmd;
      mix_cmd << thrust_local, tau.x(), tau.y(), tau.z();

      Eigen::VectorXd omega_sq = mixer_pinv_ * mix_cmd;
      Eigen::VectorXd omega = omega_sq.cwiseMax(0.0).cwiseSqrt();

      for (int i = 0; i < omega.size(); ++i) {
        omega(i) = std::clamp(omega(i), p_.min_motor_speed, p_.max_motor_speed);
      }

      publish_motors(omega);
      return;
    }

    // Check angular velocity setpoint timeout
    if (last_wsp_time_.nanoseconds() == 0 ||
        (now - last_wsp_time_).seconds() > wsp_timeout_) {
      yaw_rate_sp_ = 0.0;
    }

    // Get latest IMU data (thread-safe)
    ImuData imu_local;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!latest_imu_.valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Inner loop: No valid IMU data");
        return;
      }
      imu_local = latest_imu_;
    }

    // Check sensor timeout (15ms = 67Hz minimum, expecting 100Hz)
    if ((now - imu_local.timestamp).seconds() > 0.015) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Inner loop: IMU data too old (%.3f s)",
                           (now - imu_local.timestamp).seconds());
      return;
    }

    // Read cached attitude setpoint from outer loop (thread-safe)
    Vector3d att_sp_local;
    double thrust_local;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      att_sp_local = cached_att_sp_;
      thrust_local = cached_thrust_;
    }

    // Time delta
    double dt = (last_rate_loop_time_.nanoseconds() == 0)
                    ? 0.002 // 500Hz = 2ms
                    : (now - last_rate_loop_time_).seconds();
    last_rate_loop_time_ = now;
    dt = std::clamp(dt, 0.0001, 0.01); // Clamp to reasonable range

    // Current attitude
    Vector3d euler = quat_to_euler(imu_local.orientation);
    Vector3d pqr = imu_local.angular_velocity;

    // 3. Attitude P -> Rate Setpoint
    Vector3d att_err = att_sp_local - euler;
    att_err.z() = angle_wrap(att_err.z()); // Wrap yaw error

    Vector3d pqr_sp;
    pqr_sp.x() = g_.kp_att_roll * att_err.x();
    pqr_sp.y() = g_.kp_att_pitch * att_err.y();
    pqr_sp.z() = yaw_rate_sp_; // Direct yaw rate control

    // Saturation
    pqr_sp.head<2>() =
        pqr_sp.head<2>().cwiseMax(-g_.max_rate_xy).cwiseMin(g_.max_rate_xy);
    pqr_sp.z() = std::clamp(pqr_sp.z(), -g_.max_rate_yaw, g_.max_rate_yaw);

    // 4. Rate PID -> Torque
    Vector3d rate_err = pqr_sp - pqr;

    // Integral
    rate_int_err_ += rate_err * dt;
    rate_int_err_ = rate_int_err_.cwiseMax(-g_.rate_integral_limit)
                        .cwiseMin(g_.rate_integral_limit);

    // Derivative
    Vector3d rate_d = (rate_err - rate_prev_err_) / dt;
    rate_prev_err_ = rate_err;

    Vector3d kp_r(g_.kp_rate_roll, g_.kp_rate_pitch, g_.kp_rate_yaw);
    Vector3d ki_r(g_.ki_rate_roll, g_.ki_rate_pitch, g_.ki_rate_yaw);
    Vector3d kd_r(g_.kd_rate_roll, g_.kd_rate_pitch, g_.kd_rate_yaw);

    Vector3d alpha_cmd = kp_r.cwiseProduct(rate_err) +
                         ki_r.cwiseProduct(rate_int_err_) +
                         kd_r.cwiseProduct(rate_d);

    Vector3d I_vec(p_.inertia_xx, p_.inertia_yy, p_.inertia_zz);
    Vector3d tau = alpha_cmd.cwiseProduct(I_vec);

    // Torque Saturation
    tau.x() = std::clamp(tau.x(), -g_.max_torque_xy, g_.max_torque_xy);
    tau.y() = std::clamp(tau.y(), -g_.max_torque_xy, g_.max_torque_xy);
    tau.z() = std::clamp(tau.z(), -g_.max_torque_z, g_.max_torque_z);

    // 5. Mixer
    Eigen::Vector4d mix_cmd;
    mix_cmd << thrust_local, tau.x(), tau.y(), tau.z();

    Eigen::VectorXd omega_sq = mixer_pinv_ * mix_cmd;
    Eigen::VectorXd omega = omega_sq.cwiseMax(0.0).cwiseSqrt();

    // Clip Motors
    for (int i = 0; i < omega.size(); ++i) {
      omega(i) = std::clamp(omega(i), p_.min_motor_speed, p_.max_motor_speed);
    }

    publish_motors(omega);

    // Debug Mixer
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {thrust_local, tau.x(), tau.y(), tau.z()};
      debug_mixer_pub_->publish(msg);
    }
  }

  void publish_motors(const Eigen::VectorXd &omega) {
    actuator_msgs::msg::Actuators msg;
    for (int i = 0; i < omega.size(); ++i) {
      msg.velocity.push_back(omega(i));
    }
    motor_pub_->publish(msg);
  }

  // Helpers
  Vector3d quat_to_euler(const Quaterniond &q) {
    // Roll, Pitch, Yaw (ZYX)
    // Eigen's toRotationMatrix().eulerAngles(2, 1, 0) ranges are sometimes
    // generic Manual implementation closest to python's
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(M_PI / 2, sinp);
    else
      pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return Vector3d(roll, pitch, yaw);
  }

  double angle_wrap(double theta) {
    theta = std::fmod(theta + M_PI, 2 * M_PI);
    if (theta < 0)
      theta += 2 * M_PI;
    return theta - M_PI;
  }

  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      debug_accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      debug_att_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      debug_mixer_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tuning_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_att_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_thrust_sub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vsp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wsp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr outer_loop_timer_; // 100Hz - Velocity control
  rclcpp::TimerBase::SharedPtr
      inner_loop_timer_; // 500Hz - Attitude + Rate control
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
