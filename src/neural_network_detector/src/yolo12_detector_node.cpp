#include "neural_network_detector/yolo12_detector_node.hpp"
#include <ros2_utils/clock_sync.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>


namespace yolo12_detector_node
{

static const int color_channels = 3;

YOLO12DetectorNode::YOLO12DetectorNode(const rclcpp::NodeOptions & options)
: Node("yolo12_detector_node", options),
  feedback_received_(false),
  last_feedback_time_(this->get_clock()->now()),
  last_detection_time_(this->get_clock()->now())
{
    // Declare use_sim_time parameter FIRST
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", false);
    }
    // Initialize parameters
    initializeParameters();
    
    // Initialize the YOLO12 detector
    initializeDetector();
    
    // Pre-allocate buffers for performance (like in ROS1 version)
    length_final_img_ = static_cast<size_t>(desired_resolution_.width * desired_resolution_.height * color_channels);
    buffer_final_img_ = std::make_unique<uint8_t[]>(length_final_img_);
    buffer_results_ = std::make_unique<uint8_t[]>(length_final_img_);
    
    RCLCPP_INFO(this->get_logger(), "Allocated buffers: final_img=%zu bytes, results=%zu bytes", 
                length_final_img_, length_final_img_);


    // NOTE: Initialize timing variables with placeholder - will be updated after clock sync
    initializeAfterClockSync();
    
    // Create subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        rclcpp::QoS(1).best_effort(),
        std::bind(&YOLO12DetectorNode::imageCallback, this, std::placeholders::_1));
    
    feedback_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkFeedback>(
        "feedback",
        rclcpp::QoS(10),
        std::bind(&YOLO12DetectorNode::feedbackCallback, this, std::placeholders::_1));
    
    // Create publishers
    detection_pub_ = this->create_publisher<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
        "/target_detections", rclcpp::QoS(10));
    
    detection_count_pub_ = this->create_publisher<neural_network_msgs::msg::NeuralNetworkNumberOfDetections>(
        "/target_detection_count", rclcpp::QoS(10));

    
     if (publish_debug_image_) {
       image_transport_ = std::make_unique<image_transport::ImageTransport>(
        rclcpp::Node::SharedPtr(this));
        debug_image_pub_ = image_transport_->advertise("debug_image", 1);
    }
    
    // Set up rate limiting
    if (max_update_rate_hz_ > 0.0) {
        min_detection_interval_ = rclcpp::Duration::from_seconds(1.0 / max_update_rate_hz_);
    } else {
        min_detection_interval_ = rclcpp::Duration::from_seconds(0.0);
    }

    // Register parameter change callback for runtime reconfiguration
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&YOLO12DetectorNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "YOLO12 Detector Node initialized successfully");
}


// NEW: Method to update timing after clock sync
void YOLO12DetectorNode::initializeAfterClockSync() {
    // This will be called after clock sync in main()
    // For now, just log that we're ready
    RCLCPP_DEBUG(this->get_logger(), "Ready for clock sync initialization");
}

// NEW: Method to update timestamps after clock sync
void YOLO12DetectorNode::updateTimestampsAfterClockSync() {
    auto current_time = ros2_utils::ClockSynchronizer::getSafeTime(shared_from_this());
    
    if (current_time.nanoseconds() > 0) {
        last_feedback_time_ = current_time;
        last_detection_time_ = current_time;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Updated internal timestamps after clock sync: %.3f", 
                   current_time.seconds());
    } else {
        RCLCPP_WARN(this->get_logger(), "Cannot update timestamps - invalid clock time");
    }
}

void YOLO12DetectorNode::initializeParameters()
{
    // Declare and get parameters - use package share directory for default paths
    std::string package_share_dir;
    try {
        package_share_dir = ament_index_cpp::get_package_share_directory("neural_network_detector");
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find neural_network_detector package: %s", e.what());
        package_share_dir = "";
    }
    
    std::string default_model_path = package_share_dir.empty() ? 
        "models/yolo12n.onnx" : 
        package_share_dir + "/models/yolo12n.onnx";
    std::string default_labels_path = package_share_dir.empty() ? 
        "models/coco.names" : 
        package_share_dir + "/models/coco.names";
        
    this->declare_parameter<std::string>("model_path", default_model_path);
    this->declare_parameter<std::string>("labels_path", default_labels_path);
    this->declare_parameter<bool>("use_gpu", false);
    this->declare_parameter<float>("confidence_threshold", 0.5f);
    this->declare_parameter<float>("iou_threshold", 0.45f);
    this->declare_parameter<int>("desired_class", 1);
    this->declare_parameter<int>("desired_width", 300);
    this->declare_parameter<int>("desired_height", 300);
    this->declare_parameter<float>("aspect_ratio", 1.33333f);
    this->declare_parameter<double>("border_dropoff", 0.05);
    this->declare_parameter<bool>("publish_debug_image", false);
    this->declare_parameter<bool>("max_update_force", false);
    this->declare_parameter<double>("max_update_rate_hz", 0.0);
    this->declare_parameter<double>("feedback_timeout_sec", 5.0);
    
    // Variance parameters
    this->declare_parameter<float>("var_const_x_min", 0.0f);
    this->declare_parameter<float>("var_const_x_max", 0.0f);
    this->declare_parameter<float>("var_const_y_min", 0.0f);
    this->declare_parameter<float>("var_const_y_max", 0.0f);
    
    // Get parameter values
    model_path_ = this->get_parameter("model_path").as_string();
    labels_path_ = this->get_parameter("labels_path").as_string();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    desired_class_ = this->get_parameter("desired_class").as_int();
    
    int width = this->get_parameter("desired_width").as_int();
    int height = this->get_parameter("desired_height").as_int();
    desired_resolution_ = cv::Size(width, height);
    
    aspect_ratio_ = this->get_parameter("aspect_ratio").as_double();
    border_dropoff_ = this->get_parameter("border_dropoff").as_double();
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    max_update_force_ = this->get_parameter("max_update_force").as_bool();
    max_update_rate_hz_ = this->get_parameter("max_update_rate_hz").as_double();
    
    double timeout_sec = this->get_parameter("feedback_timeout_sec").as_double();
    feedback_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
    
    var_const_x_min_ = this->get_parameter("var_const_x_min").as_double();
    var_const_x_max_ = this->get_parameter("var_const_x_max").as_double();
    var_const_y_min_ = this->get_parameter("var_const_y_min").as_double();
    var_const_y_max_ = this->get_parameter("var_const_y_max").as_double();
    
    // Validate required parameters
    if (model_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "model_path parameter is required");
        throw std::runtime_error("model_path parameter is required");
    }
    
    if (labels_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "labels_path parameter is required");
        throw std::runtime_error("labels_path parameter is required");
    }
    
    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Model path: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Labels path: %s", labels_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Use GPU: %s", use_gpu_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Desired class: %d", desired_class_);
}

void YOLO12DetectorNode::initializeDetector()
{
    try {
        yolo_detector_ = std::make_unique<YOLO12Detector>(model_path_, labels_path_, use_gpu_);
        RCLCPP_INFO(this->get_logger(), "YOLO12 detector initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO12 detector: %s", e.what());
        throw;
    }
}

void YOLO12DetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Invalid ImageConstPtr received, not handled.");
        return;
    }

    // Validate timestamp (relaxed validation - allow processing even with timestamp issues)
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Invalid timestamp in image message, but continuing processing");
        // Continue processing instead of returning - timestamp issues shouldn't block detection
    }

    // Check rate limiting
    if (shouldSkipDetection()) {
        return;
    }

    try {
        // Convert ROS image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat mat_img = cv_ptr->image;

        if (mat_img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }

#ifdef DEBUG_ROTATE_IMAGE_90_CW
        cv::transpose(mat_img, mat_img);
        cv::flip(mat_img, mat_img, 1); // transpose+flip(1) = Rotate 90 CW
#endif

        const cv::Size2i original_resolution(mat_img.cols, mat_img.rows);
        
        // Check if feedback is timed out
        bool timed_out = isFeedbackTimedOut();
        if (timed_out) {
            auto current_time = this->get_clock()->now();
            auto time_diff = current_time - last_feedback_time_;
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Using whole image because no tracker feedback for " << time_diff.seconds() << " seconds");
        }

        // Create projection object for coordinate transformations
        cv::projection2i proj_crop(cv::Point2f(1.0f, 1.0f), cv::Point2i(0, 0));
        
        // Get crop area based on feedback
        const cv::Rect crop_area = getCropArea(original_resolution, latest_feedback_, 
                                             desired_resolution_, proj_crop, timed_out);

        // Crop the image
        cv::Mat cropped = mat_img(crop_area);

        // Resize to desired resolution using pre-allocated buffer 
        const int sizes[2] = {desired_resolution_.height, desired_resolution_.width}; // y comes first
        cv::Mat resized(2, sizes, CV_8UC3, buffer_final_img_.get());
        cv::resize(cropped, resized, resized.size(), 0, 0, cv::INTER_LINEAR);

        // Calculate scaling projection
        cv::projection2i proj_scale(
            cv::Point2f(desired_resolution_.width / static_cast<float>(cropped.cols), 
                       desired_resolution_.height / static_cast<float>(cropped.rows)),
            cv::Point2i(0, 0));



        auto start = std::chrono::high_resolution_clock::now();

        // Run YOLO detection on resized image
        std::vector<Detection> detections = yolo_detector_->detect(
            resized, confidence_threshold_, iou_threshold_);


        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        RCLCPP_INFO(this->get_logger(), "Inference latency: %ld ms", duration_ms);

        // Create detection array message
        neural_network_msgs::msg::NeuralNetworkDetectionArray detection_array_msg;
        detection_array_msg.header.frame_id = msg->header.frame_id;
        detection_array_msg.header.stamp = msg->header.stamp;

        for (const auto& det : detections) {
            // Skip if below threshold or not desired class
            if (det.conf < confidence_threshold_ || 
                (desired_class_ >= 0 && det.classId != desired_class_)) {
                continue;
            }

            // Convert YOLO detection format (top_left corner + size) to min/max format
            float x_min = det.box.x;
            float y_min = det.box.y;
            float width = det.box.width ;
            float height = det.box.height ;

            // Calculate bounds in the resized image space (floating point for precision)
            float xmin_f = x_min ;
            float ymin_f = y_min ;
            float xmax_f = x_min + width;
            float ymax_f = y_min + height;

            // Clamp to resized image bounds first
            xmin_f = std::max(0.0f, std::min(xmin_f, static_cast<float>(desired_resolution_.width - 1)));
            ymin_f = std::max(0.0f, std::min(ymin_f, static_cast<float>(desired_resolution_.height - 1)));
            xmax_f = std::max(xmin_f + 1.0f, std::min(xmax_f, static_cast<float>(desired_resolution_.width)));
            ymax_f = std::max(ymin_f + 1.0f, std::min(ymax_f, static_cast<float>(desired_resolution_.height)));

            // Convert to integer coordinates in resized image space
            int xmin = static_cast<int>(std::round(xmin_f));
            int ymin = static_cast<int>(std::round(ymin_f));
            int xmax = static_cast<int>(std::round(xmax_f));
            int ymax = static_cast<int>(std::round(ymax_f));

            // Transform back to original image coordinates
            cv::Point2i detection_min = proj_crop << (proj_scale << cv::Point2i(xmin, ymin));
            cv::Point2i detection_max = proj_crop << (proj_scale << cv::Point2i(xmax, ymax));

            // Debug logging for problematic transformations
            if (detection_min.y < 0 || detection_max.y < 0) {
                RCLCPP_WARN(this->get_logger(), 
                        "Negative Y after transformation: resized[%d,%d,%d,%d] -> original[%d,%d,%d,%d]",
                        xmin, ymin, xmax, ymax,
                        detection_min.x, detection_min.y, detection_max.x, detection_max.y);
                
                // Log transformation details for debugging
                RCLCPP_WARN(this->get_logger(), 
                        "Crop offset: (%d,%d), Scale: (%.3f,%.3f)",
                        proj_crop.offset.x, proj_crop.offset.y,
                        proj_scale.scale.x, proj_scale.scale.y);
            }

            // Final validation - ensure bounds are within original image
            const cv::Size2i original_resolution(mat_img.cols, mat_img.rows);
            
            // Clamp coordinates to valid image bounds
            detection_min.x = std::max(0, std::min(detection_min.x, original_resolution.width - 1));
            detection_min.y = std::max(0, std::min(detection_min.y, original_resolution.height - 1));
            detection_max.x = std::max(detection_min.x + 1, std::min(detection_max.x, original_resolution.width));
            detection_max.y = std::max(detection_min.y + 1, std::min(detection_max.y, original_resolution.height));
            
            // Additional check: if detection is mostly outside image bounds, skip it
            int detection_width = detection_max.x - detection_min.x;
            int detection_height = detection_max.y - detection_min.y;
            
            // Skip detections that are too small after clamping (likely edge artifacts)
            if (detection_width < 10 || detection_height < 10) {
                RCLCPP_DEBUG(this->get_logger(), 
                            "Skipping detection too small after clamping: %dx%d", 
                            detection_width, detection_height);
                continue;
            }

            // Create detection message
            neural_network_msgs::msg::NeuralNetworkDetection detection_msg;
            detection_msg.header.frame_id = msg->header.frame_id;
            detection_msg.header.stamp = msg->header.stamp;
            detection_msg.detection_score = det.conf;
            detection_msg.object_class = static_cast<int16_t>(det.classId);

            detection_msg.xmin = static_cast<int16_t>(detection_min.x);
            detection_msg.ymin = static_cast<int16_t>(detection_min.y);
            detection_msg.xmax = static_cast<int16_t>(detection_max.x);
            detection_msg.ymax = static_cast<int16_t>(detection_max.y);

            // Final validation check
            if (detection_msg.xmin >= detection_msg.xmax || detection_msg.ymin >= detection_msg.ymax) {
                RCLCPP_WARN(this->get_logger(), 
                        "Invalid detection bounds after processing: xmin=%d, xmax=%d, ymin=%d, ymax=%d", 
                        detection_msg.xmin, detection_msg.xmax, detection_msg.ymin, detection_msg.ymax);
                continue; // Skip this detection
            }

            detection_msg.variance_xmin = var_const_x_min_ * crop_area.width * crop_area.width;
            detection_msg.variance_xmax = var_const_x_max_ * crop_area.width * crop_area.width;
            detection_msg.variance_ymin = var_const_y_min_ * crop_area.height * crop_area.height;
            detection_msg.variance_ymax = var_const_y_max_ * crop_area.height * crop_area.height;

            // Apply border dropoff variance adjustments
            if (xmin < (border_dropoff_ * desired_resolution_.width)) {
                detection_msg.variance_xmin = (crop_area.width * crop_area.width / 4);
            }
            if (xmax > ((1.0 - border_dropoff_) * desired_resolution_.width)) {
                detection_msg.variance_xmax = (crop_area.width * crop_area.width / 4);
            }
            if (ymin < (border_dropoff_ * desired_resolution_.height)) {
                detection_msg.variance_ymin = (crop_area.height * crop_area.height / 4);
            }
            if (ymax > ((1.0 - border_dropoff_) * desired_resolution_.height)) {
                detection_msg.variance_ymax = (crop_area.height * crop_area.height / 4);
            }

            // Add to detection array
            detection_array_msg.detections.push_back(detection_msg);

            // Debug drawing code...
            if (publish_debug_image_ && debug_image_pub_.getNumSubscribers() > 0) {
                cv::Point2i debug_min(std::max(detection_min.x, 0), std::max(detection_min.y, 0));
                cv::Point2i debug_max(std::min(detection_max.x, original_resolution.width), 
                                    std::min(detection_max.y, original_resolution.height));
                
                // Rectangle on the original image - detection in red
                cv::rectangle(mat_img, debug_min, debug_max, cv::Scalar(50, 50, 255), 3);
            }
        }

        // Always publish detection array (even if empty) so subscribers get updates
        detection_pub_->publish(detection_array_msg);

        // Publish detection count
        auto count_msg = neural_network_msgs::msg::NeuralNetworkNumberOfDetections();
        count_msg.header = msg->header;
        count_msg.data = static_cast<uint16_t>(detection_array_msg.detections.size());
        detection_count_pub_->publish(count_msg);

        // Publish debug image if enabled and has subscribers
        if (publish_debug_image_ && debug_image_pub_.getNumSubscribers() > 0) {
            // Rectangle on the original image - zoom level in green 
            cv::rectangle(mat_img, crop_area, cv::Scalar(50, 255, 50), 5);

            // Draw feedback debug info if available 
            if (latest_feedback_.xcenter > 0 && latest_feedback_.xcenter < original_resolution.width
                && latest_feedback_.debug_included
                && latest_feedback_.ycenter > 0 && latest_feedback_.ycenter < original_resolution.height) {

                // Lines for the raw coordinates, without uncertainty, for debugging
                cv::line(mat_img, cv::Point2i(crop_area.x, latest_feedback_.head_raw), 
                        cv::Point2i(crop_area.x + crop_area.width, latest_feedback_.head_raw), 
                        cv::Scalar(0, 128, 255), 5);
                cv::line(mat_img, cv::Point2i(crop_area.x, latest_feedback_.feet_raw), 
                        cv::Point2i(crop_area.x + crop_area.width, latest_feedback_.feet_raw), 
                        cv::Scalar(0, 128, 255), 5);

                // Lines crossing the center that cover the whole image
                cv::line(mat_img, cv::Point2i(latest_feedback_.xcenter, 0), 
                        cv::Point2i(latest_feedback_.xcenter, original_resolution.height), 
                        cv::Scalar(255, 100, 100), 8);
                cv::line(mat_img, cv::Point2i(0, latest_feedback_.ycenter), 
                        cv::Point2i(original_resolution.width, latest_feedback_.ycenter), 
                        cv::Scalar(255, 100, 100), 8);
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Debug information on topic %s is active due to having at least 1 subscriber",
                                debug_image_pub_.getTopic().c_str());

            // Resize debug image for performance (like ROS1)
            cv::Mat small_img;
            cv::resize(mat_img, small_img, cv::Size(mat_img.cols / 4, mat_img.rows / 4));
            
            // Publish using image_transport
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", small_img).toImageMsg();
            debug_image_pub_.publish(debug_msg);
        }

        // Update detection time
        last_detection_time_ = this->get_clock()->now();

        RCLCPP_DEBUG(this->get_logger(), "Processed image with %zu detections",
                     detection_array_msg.detections.size());

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
}

void YOLO12DetectorNode::feedbackCallback(
    const neural_network_msgs::msg::NeuralNetworkFeedback::SharedPtr msg)
{

     // Validate timestamp
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in feedback message");
        return;
    }

    latest_feedback_ = *msg;
    feedback_received_ = true;
    last_feedback_time_ = this->get_clock()->now();
    
    RCLCPP_DEBUG(this->get_logger(), "Received feedback: ymin=%d, ymax=%d, xcenter=%d", 
                msg->ymin, msg->ymax, msg->xcenter);
}

cv::Rect YOLO12DetectorNode::getCropArea(
    const cv::Size& original_resolution,
    const neural_network_msgs::msg::NeuralNetworkFeedback& feedback,
    const cv::Size& desired_resolution,
    cv::projection2i& proj_crop,
    bool timed_out)
{
    if (timed_out || feedback.ymin > original_resolution.height || feedback.ymax < 0) {
        // Target not visible — full image crop (same logic as ROS1)
        bool landscape_origin = original_resolution.height <= original_resolution.width;
        bool landscape_target = aspect_ratio_ >= 1.0;
        bool same_aspect = (landscape_origin == landscape_target);

        if (same_aspect) {
            int crop_length = static_cast<int>(aspect_ratio_ * original_resolution.height);
            crop_length = std::min(crop_length, original_resolution.width);
            proj_crop.offset.x = (original_resolution.width - crop_length) / 2;
            proj_crop.offset.y = 0;

            return cv::Rect(
                proj_crop << cv::Point2i(0, 0),
                proj_crop << cv::Point2i(crop_length, original_resolution.height));
        } else {
            int crop_length = static_cast<int>((1.0 / aspect_ratio_) * original_resolution.width);
            crop_length = std::min(crop_length, original_resolution.height);
            proj_crop.offset.x = 0;
            proj_crop.offset.y = (original_resolution.height - crop_length) / 2;

            return cv::Rect(
                proj_crop << cv::Point2i(0, 0),
                proj_crop << cv::Point2i(original_resolution.width, crop_length));
        }
    } else {
        // Target visible - crop around feedback region 
        // Clamp bounds
        int16_t ymin = std::max<int16_t>(feedback.ymin, 0);
        int16_t ymax = std::min<int16_t>(feedback.ymax, original_resolution.height);
        int16_t delta_y = std::max<int16_t>(
            static_cast<int16_t>(0.1 * desired_resolution.height),
            ymax - ymin);
        int16_t delta_x = static_cast<int16_t>(aspect_ratio_ * delta_y);

        int16_t half_delta_x = delta_x / 2;
        int16_t xcenter = std::max<int16_t>(
            half_delta_x,
            std::min<int16_t>(feedback.xcenter, original_resolution.width - half_delta_x));

        int16_t xmin = std::max<int16_t>(xcenter - half_delta_x, 0);

        proj_crop.offset.x = xmin;
        proj_crop.offset.y = ymin;

        delta_x = std::min<int16_t>(delta_x, original_resolution.width - xmin);
        delta_y = std::min<int16_t>(delta_y, original_resolution.height - ymin);

        return cv::Rect(
            proj_crop << cv::Point2i(0, 0),
            proj_crop << cv::Point2i(delta_x, delta_y));
    }
}


bool YOLO12DetectorNode::validateDetection(
    const neural_network_msgs::msg::NeuralNetworkDetection& detection,
    const cv::Size& image_size) const
{
    // Check if coordinates are within image bounds
    if (detection.xmin < 0 || detection.ymin < 0 || 
        detection.xmax >= image_size.width || detection.ymax >= image_size.height) {
        RCLCPP_WARN(this->get_logger(), 
                   "Detection bounds outside image: [%d,%d,%d,%d] vs image size [%d,%d]",
                   detection.xmin, detection.ymin, detection.xmax, detection.ymax,
                   image_size.width, image_size.height);
        return false;
    }
    
    // Check if min < max
    if (detection.xmin >= detection.xmax || detection.ymin >= detection.ymax) {
        RCLCPP_WARN(this->get_logger(), 
                   "Invalid detection bounds: xmin=%d >= xmax=%d or ymin=%d >= ymax=%d",
                   detection.xmin, detection.xmax, detection.ymin, detection.ymax);
        return false;
    }
    
    // Check reasonable size (not too small)
    int width = detection.xmax - detection.xmin;
    int height = detection.ymax - detection.ymin;
    if (width < 5 || height < 5) {
        RCLCPP_WARN(this->get_logger(), 
                   "Detection too small: %dx%d pixels", width, height);
        return false;
    }
    
    return true;
}

bool YOLO12DetectorNode::isFeedbackTimedOut() const
{
    if (!feedback_received_) {
        return true;
    }
    
    auto current_time = this->get_clock()->now();
    auto time_since_feedback = current_time - last_feedback_time_;
    return time_since_feedback > feedback_timeout_;
}

bool YOLO12DetectorNode::shouldSkipDetection() const
{
    if (!max_update_force_ || min_detection_interval_.nanoseconds() == 0) {
        return false;
    }
    
    auto current_time = this->get_clock()->now();
    auto time_since_last = current_time - last_detection_time_;
    return time_since_last < min_detection_interval_;
}

std::vector<Detection> YOLO12DetectorNode::filterDetections(
    const std::vector<Detection>& detections) const
{
    std::vector<Detection> filtered;
    
    for (const auto& detection : detections) {
        // Filter by class if desired_class is specified (>= 0)
        if (desired_class_ >= 0 && detection.classId != desired_class_) {
            continue;
        }
        
        // Filter by confidence threshold
        if (detection.conf < confidence_threshold_) {
            continue;
        }
        
        filtered.push_back(detection);
    }
    
    return filtered;
}

rcl_interfaces::msg::SetParametersResult YOLO12DetectorNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        if (param.get_name() == "desired_class") {
            int new_class = param.as_int();
            if (new_class != desired_class_) {
                desired_class_ = new_class;
                RCLCPP_INFO(this->get_logger(),
                    "Runtime parameter update: desired_class changed to %d", desired_class_);
            }
        } else if (param.get_name() == "confidence_threshold") {
            float new_threshold = static_cast<float>(param.as_double());
            if (new_threshold != confidence_threshold_) {
                confidence_threshold_ = new_threshold;
                RCLCPP_INFO(this->get_logger(),
                    "Runtime parameter update: confidence_threshold changed to %.2f", confidence_threshold_);
            }
        }
    }

    return result;
}

} // namespace yolo12_detector_node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        // Pass NodeOptions explicitly
        auto node = std::make_shared<yolo12_detector_node::YOLO12DetectorNode>(
            rclcpp::NodeOptions()
        );
        WAIT_FOR_CLOCK_DELAYED(node);
        
        // CRITICAL: Update timestamps after clock sync
        node->updateTimestampsAfterClockSync();

        RCLCPP_INFO(node->get_logger(), "Starting YOLO12 Detector Node...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo12_detector_node"), 
                     "Failed to start node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
