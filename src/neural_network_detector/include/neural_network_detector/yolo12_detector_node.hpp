#ifndef YOLO12_DETECTOR_NODE_HPP
#define YOLO12_DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

// Include the YOLO12 detector header
#include "det/YOLO12.hpp"
#include "cv/extensions/projection.h"

// Custom message
#include "neural_network_msgs/msg/neural_network_detection.hpp"
#include "neural_network_msgs/msg/neural_network_detection_array.hpp"
#include "neural_network_msgs/msg/neural_network_feedback.hpp"
#include "neural_network_msgs/msg/neural_network_number_of_detections.hpp"

namespace yolo12_detector_node
{

class YOLO12DetectorNode : public rclcpp::Node
{
public:
 
    explicit YOLO12DetectorNode(const rclcpp::NodeOptions & options);

  
    ~YOLO12DetectorNode() = default;

   
    void updateTimestampsAfterClockSync();

private:
    // Publishers and Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkFeedback>::SharedPtr feedback_sub_;
    
    rclcpp::Publisher<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<neural_network_msgs::msg::NeuralNetworkNumberOfDetections>::SharedPtr detection_count_pub_;
    
    // Image transport for debug image
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher debug_image_pub_;

    // YOLO12 Detector instance
    std::unique_ptr<YOLO12Detector> yolo_detector_;

    // Parameters
    std::string model_path_;
    std::string labels_path_;
    bool use_gpu_;
    float confidence_threshold_;
    float iou_threshold_;
    int desired_class_;
    cv::Size desired_resolution_;
    float aspect_ratio_;
    double border_dropoff_;
    bool publish_debug_image_;
    bool max_update_force_;
    double max_update_rate_hz_;

    // Variance constants for detection uncertainty
    float var_const_x_min_;
    float var_const_x_max_;
    float var_const_y_min_;
    float var_const_y_max_;

    // Buffer management 
    size_t length_final_img_;
    std::unique_ptr<uint8_t[]> buffer_final_img_;
    std::unique_ptr<uint8_t[]> buffer_results_;

    // Feedback handling
    neural_network_msgs::msg::NeuralNetworkFeedback latest_feedback_;
    bool feedback_received_;
    rclcpp::Time last_feedback_time_;
    rclcpp::Duration feedback_timeout_{rclcpp::Duration::from_seconds(5.0)};  

    // Rate limiting
    rclcpp::Time last_detection_time_;
    rclcpp::Duration min_detection_interval_{rclcpp::Duration::from_seconds(0.0)};

    /**
     * @brief Initialize parameters from ROS2 parameter server
     */
    void initializeParameters();

    /**
     * @brief Initialize the YOLO12 detector
     */
    void initializeDetector();

    /**
     * @brief Initialize timing after clock sync
     */
    void initializeAfterClockSync();

    /**
     * @brief Image callback for processing incoming images
     * @param msg Incoming image message
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Feedback callback for updating detection parameters
     * @param msg Feedback message
     */
    void feedbackCallback(const neural_network_msgs::msg::NeuralNetworkFeedback::SharedPtr msg);

    /**
     * @brief Get crop area based on feedback and image properties
     * @param original_resolution Original image resolution
     * @param feedback Latest feedback information
     * @return cv::Rect representing the crop area
     */
    cv::Rect getCropArea(
    const cv::Size& original_resolution,
    const neural_network_msgs::msg::NeuralNetworkFeedback& feedback,
    const cv::Size& desired_resolution,
    cv::projection2i& proj_crop,
    bool timed_out);

    /**
     * @brief Check if feedback has timed out
     * @return true if feedback has timed out
     */
    bool isFeedbackTimedOut() const;

    /**
     * @brief Check if detection rate limiting should be applied
     * @return true if detection should be skipped due to rate limiting
     */
    bool shouldSkipDetection() const;

    /**
     * @brief Filter detections by desired class and score threshold
     * @param detections Input detections
     * @return Filtered detections
     */
    std::vector<Detection> filterDetections(const std::vector<Detection>& detections) const;

    bool validateDetection(const neural_network_msgs::msg::NeuralNetworkDetection& detection,
    const cv::Size& image_size) const;

    /**
     * @brief Parameter change callback for runtime reconfiguration
     * @param parameters Vector of changed parameters
     * @return SetParametersResult indicating success or failure
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace yolo12_detector_node

#endif // YOLO12_DETECTOR_NODE_HPP