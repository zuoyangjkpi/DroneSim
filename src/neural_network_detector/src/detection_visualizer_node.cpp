#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class DetectionVisualizerNode : public rclcpp::Node
{
public:
    DetectionVisualizerNode() : Node("detection_visualizer_node"),
        last_image_time_(this->get_clock()->now()),
        last_detection_time_(this->get_clock()->now()),
        last_publish_time_(this->get_clock()->now()),
        tracking_distance_(0.0),
        tracking_altitude_(0.0),
        drone_altitude_(0.0),
        drone_vx_(0.0),
        drone_vy_(0.0),
        drone_vz_(0.0),
        drone_speed_xy_(0.0),
        drone_speed_(0.0),
        drone_heading_rad_(0.0),
        nmpc_status_received_(false),
        odometry_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Detection visualizer node starting...");

        // Use a one-shot timer to initialize after the object is fully constructed
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DetectionVisualizerNode::initialize, this));
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
            "/person_detections", 1,
            std::bind(&DetectionVisualizerNode::detectionCallback, this, std::placeholders::_1));

        // Subscribe to NMPC status for tracking distance and altitude
        nmpc_status_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/drone/controller/status", 1,
            std::bind(&DetectionVisualizerNode::nmpcStatusCallback, this, std::placeholders::_1));

        // Subscribe to drone odometry for altitude and speed
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 1,
            std::bind(&DetectionVisualizerNode::odometryCallback, this, std::placeholders::_1));

        // Publisher for annotated image
        annotated_image_pub_ = image_transport_->advertise("/detection_image", 1);

        RCLCPP_INFO(this->get_logger(), "Detection visualizer node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /camera/image_raw, /person_detections, /drone/controller/status, /X3/odometry");
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
        // msg.data format: [person_detected, tracking_distance, tracking_altitude, optimization_time, iterations_used, cost_value]
        if (msg->data.size() >= 3) {
            tracking_distance_ = msg->data[1];
            tracking_altitude_ = msg->data[2];
            nmpc_status_received_ = true;
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
            if (current_detections_ && !current_detections_->detections.empty()) {
                int detection_count = 0;
                for (const auto& detection : current_detections_->detections) {
                    if (detection.object_class == 0 && detection.detection_score > 0.3) { // Person class matching YOLO threshold
                        // Draw green bounding box with thicker line for stability
                        cv::rectangle(cv_ptr->image,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin)),
                            cv::Point(static_cast<int>(detection.xmax), static_cast<int>(detection.ymax)),
                            cv::Scalar(0, 255, 0), 4);
                        
                        // Draw detection score with fixed precision
                        char confidence_text[32];
                        snprintf(confidence_text, sizeof(confidence_text), "Person: %.2f", detection.detection_score);
                        cv::putText(cv_ptr->image, confidence_text,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin) - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                        detection_count++;
                    }
                }
                
                // Add detection count overlay
                if (detection_count > 0) {
                    std::string count_text = "Persons detected: " + std::to_string(detection_count);
                    cv::putText(cv_ptr->image, count_text,
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
            } else {
                // Add "No detections" overlay when no person detected
                std::string no_detection_text = "No persons detected";
                cv::putText(cv_ptr->image, no_detection_text,
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }

            // Add NMPC tracking status overlay (yellow text)
            if (nmpc_status_received_) {
                char tracking_distance_text[64];
                snprintf(tracking_distance_text, sizeof(tracking_distance_text),
                    "Tracking Distance: %.2f m", tracking_distance_);
                cv::putText(cv_ptr->image, tracking_distance_text,
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

                char tracking_altitude_text[64];
                snprintf(tracking_altitude_text, sizeof(tracking_altitude_text),
                    "Tracking Altitude: %.2f m", tracking_altitude_);
                cv::putText(cv_ptr->image, tracking_altitude_text,
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            }

            // Add drone altitude and speed overlay (yellow text)
            if (odometry_received_) {
                char altitude_text[64];
                snprintf(altitude_text, sizeof(altitude_text),
                    "Drone Altitude: %.2f m", drone_altitude_);
                cv::putText(cv_ptr->image, altitude_text,
                    cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                char speed_xy_text[64];
                snprintf(speed_xy_text, sizeof(speed_xy_text),
                    "Drone Speed XY: %.2f m/s", drone_speed_xy_);
                cv::putText(cv_ptr->image, speed_xy_text,
                    cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                char speed_z_text[64];
                snprintf(speed_z_text, sizeof(speed_z_text),
                    "Drone Speed Z: %.2f m/s", drone_vz_);
                cv::putText(cv_ptr->image, speed_z_text,
                    cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                const double heading_deg = drone_heading_rad_ * 57.29577951308232;
                char heading_text[64];
                snprintf(heading_text, sizeof(heading_text),
                    "Drone Heading: %.1f deg", heading_deg);
                cv::putText(cv_ptr->image, heading_text,
                    cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
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
    rclcpp::TimerBase::SharedPtr init_timer_;

    sensor_msgs::msg::Image::ConstSharedPtr current_image_;
    neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr current_detections_;

    rclcpp::Time last_image_time_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time last_publish_time_;

    // NMPC tracking status
    double tracking_distance_;
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
