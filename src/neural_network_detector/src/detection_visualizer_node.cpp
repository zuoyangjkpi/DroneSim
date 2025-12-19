#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>

class DetectionVisualizerNode : public rclcpp::Node
{
public:
    DetectionVisualizerNode() : Node("detection_visualizer_node"),
        last_image_time_(this->get_clock()->now()),
        last_detection_time_(this->get_clock()->now()),
        last_publish_time_(this->get_clock()->now()),
        desired_tracking_distance_(0.0),
        current_tracking_distance_(0.0),
        tracking_altitude_(0.0),
        nmpc_status_received_(false),
        drone_altitude_(0.0),
        drone_vx_(0.0),
        drone_vy_(0.0),
        drone_vz_(0.0),
        drone_speed_xy_(0.0),
        drone_speed_(0.0),
        drone_heading_rad_(0.0),
        odometry_received_(false),
        target_x_(0.0),
        target_y_(0.0),
        target_z_(0.0),
        target_yaw_(0.0),
        delta_x_(0.0),
        delta_y_(0.0),
        delta_z_(0.0),
        delta_yaw_(0.0),
        waypoint_received_(false),
        attitude_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Detection visualizer node starting...");

        // Load COCO class names
        loadClassNames();

        // Use a one-shot timer to initialize after the object is fully constructed
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DetectionVisualizerNode::initialize, this));
    }

    void loadClassNames() {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("neural_network_detector");
            std::string labels_path = package_share_dir + "/YOLOs-CPP/models/coco.names";

            std::ifstream file(labels_path);
            if (!file.is_open()) {
                RCLCPP_WARN(this->get_logger(), "Could not open coco.names at %s, using default class names", labels_path.c_str());
                return;
            }

            std::string line;
            while (std::getline(file, line)) {
                if (!line.empty()) {
                    class_names_.push_back(line);
                }
            }
            file.close();

            RCLCPP_INFO(this->get_logger(), "Loaded %zu COCO class names", class_names_.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load class names: %s", e.what());
        }
    }

    std::string getClassName(int class_id) const {
        if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
            return class_names_[class_id];
        }
        return "unknown";
    }

    void initialize()
    {
        // Cancel the initialization timer
        init_timer_.reset();

        // Now it's safe to use shared_from_this()
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

        // Subscribers
        image_sub_ = image_transport_->subscribe(
            "/camera/image_raw", 1,
            std::bind(&DetectionVisualizerNode::imageCallback, this, std::placeholders::_1));

        detection_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
            "/target_detections", 1,
            std::bind(&DetectionVisualizerNode::detectionCallback, this, std::placeholders::_1));

        // Subscribe to NMPC status for tracking distance and altitude
        nmpc_status_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/drone/controller/status", 1,
            std::bind(&DetectionVisualizerNode::nmpcStatusCallback, this, std::placeholders::_1));

        // Subscribe to drone odometry for altitude and speed
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 1,
            std::bind(&DetectionVisualizerNode::odometryCallback, this, std::placeholders::_1));

        // Subscribe to waypoint command
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/control/waypoint_command", 1,
            std::bind(&DetectionVisualizerNode::waypointCallback, this, std::placeholders::_1));

        // Subscribe to attitude command (Vector3Stamped: roll, pitch, yaw)
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/drone/control/attitude_command", 1,
            std::bind(&DetectionVisualizerNode::attitudeCallback, this, std::placeholders::_1));

        // Publisher for annotated image
        annotated_image_pub_ = image_transport_->advertise("/detection_image", 1);

        RCLCPP_INFO(this->get_logger(), "Detection visualizer node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /camera/image_raw, /target_detections, /drone/controller/status, /X3/odometry");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /detection_image");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        current_image_ = msg;
        last_image_time_ = this->get_clock()->now();
        publishAnnotatedImage();
    }
    
    void detectionCallback(const neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr& msg)
    {
        current_detections_ = msg;
        last_detection_time_ = this->get_clock()->now();
        publishAnnotatedImage();
    }

    void nmpcStatusCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
    {
        // msg.data format: [target_detected, desired_distance, tracking_altitude, optimization_time, iterations_used, cost_value, current_distance]
        if (msg->data.size() >= 3) {
            desired_tracking_distance_ = msg->data[1];
            tracking_altitude_ = msg->data[2];
            nmpc_status_received_ = true;
        }
        if (msg->data.size() >= 7) {
            current_tracking_distance_ = msg->data[6];
        } else {
            current_tracking_distance_ = desired_tracking_distance_;
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
    {
        // Extract altitude (z position)
        drone_altitude_ = msg->pose.pose.position.z;

        // Store velocity components
        drone_vx_ = msg->twist.twist.linear.x;
        drone_vy_ = msg->twist.twist.linear.y;
        drone_vz_ = msg->twist.twist.linear.z;

        // Extract yaw heading (radians) from orientation quaternion
        const auto& q = msg->pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        drone_heading_rad_ = std::atan2(siny_cosp, cosy_cosp);

        // Calculate horizontal (XY) speed and total 3D speed
        drone_speed_xy_ = std::sqrt(drone_vx_*drone_vx_ + drone_vy_*drone_vy_);
        drone_speed_ = std::sqrt(drone_vx_*drone_vx_ + drone_vy_*drone_vy_ + drone_vz_*drone_vz_);

        odometry_received_ = true;

        // Calculate errors if we have waypoint and attitude targets
        if (waypoint_received_) {
            delta_x_ = target_x_ - msg->pose.pose.position.x;
            delta_y_ = target_y_ - msg->pose.pose.position.y;
            delta_z_ = target_z_ - msg->pose.pose.position.z;
        }

        if (attitude_received_) {
            // Calculate yaw error
            double target_yaw = target_yaw_;
            double current_yaw = drone_heading_rad_;
            delta_yaw_ = target_yaw - current_yaw;
            // Wrap to [-pi, pi]
            while (delta_yaw_ > M_PI) delta_yaw_ -= 2.0 * M_PI;
            while (delta_yaw_ < -M_PI) delta_yaw_ += 2.0 * M_PI;
        }
    }

    void waypointCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        target_z_ = msg->pose.position.z;
        waypoint_received_ = true;
    }

    void attitudeCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg)
    {
        // Extract yaw directly from vector (vector.z = yaw in radians)
        target_yaw_ = msg->vector.z;
        attitude_received_ = true;
    }

    void publishAnnotatedImage()
    {
        if (!current_image_) {
            RCLCPP_DEBUG(this->get_logger(), "No current image available, skipping publish");
            return;
        }
        
        // Rate limiting to prevent excessive publishing
        auto now = this->get_clock()->now();
        if ((now - last_publish_time_).seconds() < 0.033) {  // Max 30 Hz for smoother display
            return;
        }
        last_publish_time_ = now;
        
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image_, sensor_msgs::image_encodings::BGR8);
            
            // Draw bounding boxes if detections are available
        int text_y = 30;
        const int line_spacing = 28;
        auto put_status_line = [&](const std::string& text, const cv::Scalar& color, double scale = 0.7) {
            cv::putText(cv_ptr->image, text,
                cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, scale, color, 2);
            text_y += line_spacing;
        };

        if (current_detections_ && !current_detections_->detections.empty()) {
            // Count detections per class
            std::map<int, int> class_counts;

            for (const auto& detection : current_detections_->detections) {
                if (detection.detection_score > 0.3) { // YOLO threshold
                    // Draw green bounding box with thicker line for stability
                    cv::rectangle(cv_ptr->image,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin)),
                            cv::Point(static_cast<int>(detection.xmax), static_cast<int>(detection.ymax)),
                            cv::Scalar(0, 255, 0), 4);

                        // Draw detection score with dynamic class name
                        std::string class_name = getClassName(detection.object_class);
                        char confidence_text[64];
                        snprintf(confidence_text, sizeof(confidence_text), "%s: %.2f",
                                class_name.c_str(), detection.detection_score);
                        cv::putText(cv_ptr->image, confidence_text,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin) - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                        // Count this class
                        class_counts[detection.object_class]++;
                    }
                }

                // Add detection count overlay with class names
                if (!class_counts.empty()) {
                    std::string status_text = "Detected: ";
                    bool first = true;
                    for (const auto& [class_id, count] : class_counts) {
                        if (!first) status_text += ", ";
                        status_text += std::to_string(count) + " " + getClassName(class_id);
                        if (count > 1) status_text += "s";
                        first = false;
                    }
                    put_status_line(status_text, cv::Scalar(0, 255, 0), 0.8);
                }
            } else {
                // Add "No detections" overlay when no object detected
                put_status_line("No detections", cv::Scalar(0, 0, 255), 0.8);
            }

            text_y += 5;

            // Add NMPC tracking status overlay (blue text)
            if (nmpc_status_received_) {
                const cv::Scalar kBlueText(255, 0, 0);  // BGR

                char desired_text[64];
                snprintf(desired_text, sizeof(desired_text),
                    "Desired Distance: %.2f m", desired_tracking_distance_);
                put_status_line(desired_text, kBlueText);

                char current_text[64];
                snprintf(current_text, sizeof(current_text),
                    "Current Distance: %.2f m", current_tracking_distance_);
                put_status_line(current_text, kBlueText);

                char tracking_altitude_text[64];
                snprintf(tracking_altitude_text, sizeof(tracking_altitude_text),
                    "Tracking Altitude: %.2f m", tracking_altitude_);
                put_status_line(tracking_altitude_text, kBlueText);

                text_y += 5;
            }

            // Add drone altitude and speed overlay (yellow text)
            if (odometry_received_) {
                char altitude_text[64];
                snprintf(altitude_text, sizeof(altitude_text),
                    "Drone Altitude: %.2f m", drone_altitude_);
                put_status_line(altitude_text, cv::Scalar(0, 255, 255));

                char speed_xy_text[64];
                snprintf(speed_xy_text, sizeof(speed_xy_text),
                    "Drone Speed XY: %.2f m/s", drone_speed_xy_);
                put_status_line(speed_xy_text, cv::Scalar(0, 255, 255));

                char speed_z_text[64];
                snprintf(speed_z_text, sizeof(speed_z_text),
                    "Drone Speed Z: %.2f m/s", drone_vz_);
                put_status_line(speed_z_text, cv::Scalar(0, 255, 255));

                const double heading_deg = drone_heading_rad_ * 57.29577951308232;
                char heading_text[64];
                snprintf(heading_text, sizeof(heading_text),
                    "Drone Heading: %.1f deg", heading_deg);
                put_status_line(heading_text, cv::Scalar(0, 255, 255));

                text_y += 5;
            }

            // Add control error overlay (magenta text) - world frame
            if (waypoint_received_ && attitude_received_) {
                char delta_x_text[64];
                snprintf(delta_x_text, sizeof(delta_x_text),
                    "Delta X: %.2f m", delta_x_);
                put_status_line(delta_x_text, cv::Scalar(255, 0, 255));

                char delta_y_text[64];
                snprintf(delta_y_text, sizeof(delta_y_text),
                    "Delta Y: %.2f m", delta_y_);
                put_status_line(delta_y_text, cv::Scalar(255, 0, 255));

                char delta_z_text[64];
                snprintf(delta_z_text, sizeof(delta_z_text),
                    "Delta Z: %.2f m", delta_z_);
                put_status_line(delta_z_text, cv::Scalar(255, 0, 255));

                const double delta_yaw_deg = delta_yaw_ * 57.29577951308232;
                char delta_yaw_text[64];
                snprintf(delta_yaw_text, sizeof(delta_yaw_text),
                    "Delta Yaw: %.1f deg", delta_yaw_deg);
                put_status_line(delta_yaw_text, cv::Scalar(255, 0, 255));
            }
            
            // Always publish the image (with or without detections)
            annotated_image_pub_.publish(cv_ptr->toImageMsg());
            
            RCLCPP_DEBUG(this->get_logger(), "Published detection image");
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher annotated_image_pub_;

    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nmpc_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    sensor_msgs::msg::Image::ConstSharedPtr current_image_;
    neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr current_detections_;

    rclcpp::Time last_image_time_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time last_publish_time_;

    // NMPC tracking status
    double desired_tracking_distance_;
    double current_tracking_distance_;
    double tracking_altitude_;
    bool nmpc_status_received_;

    // Drone state
    double drone_altitude_;
    double drone_vx_;
    double drone_vy_;
    double drone_vz_;
    double drone_speed_xy_;
    double drone_speed_;
    double drone_heading_rad_;
    bool odometry_received_;

    // Control targets and errors
    double target_x_;
    double target_y_;
    double target_z_;
    double target_yaw_;
    double delta_x_;
    double delta_y_;
    double delta_z_;
    double delta_yaw_;
    bool waypoint_received_;
    bool attitude_received_;

    // COCO class names for dynamic display
    std::vector<std::string> class_names_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
