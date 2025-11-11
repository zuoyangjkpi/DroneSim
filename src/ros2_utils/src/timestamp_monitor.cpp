// timestamp_monitor.cpp - Debug node to analyze timing across all topics
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>
#include <neural_network_msgs/msg/neural_network_feedback.hpp>
#include <uav_msgs/msg/uav_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_utils/clock_sync.hpp>
#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>

/**
 * @brief Comprehensive timestamp monitoring and analysis node
 * Subscribes to all critical topics and analyzes timing relationships
 */
class TimestampMonitor : public rclcpp::Node {
private:
    struct TopicStats {
        std::string topic_name;
        std::string message_type;
        rclcpp::Time last_timestamp{0};
        rclcpp::Time last_received{0};
        size_t message_count{0};
        double avg_frequency{0.0};
        double timestamp_drift{0.0};  // Difference between message time and receive time
        bool is_active{false};
        std::vector<double> recent_intervals;  // For frequency calculation
        std::vector<double> recent_drifts;     // For drift analysis
    };
    
    std::map<std::string, TopicStats> topic_stats_;
    rclcpp::Time monitor_start_time_;
    rclcpp::Time reference_time_{0};  // Reference timestamp for sync analysis
    std::string reference_topic_;
    
    // Parameters
    double analysis_window_sec_{10.0};
    double sync_tolerance_ms_{100.0};
    bool log_to_file_{true};
    bool show_detailed_stats_{false};
    std::string log_file_path_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr analysis_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // File logging
    std::ofstream log_file_;
    
    // Subscribers for all critical topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_raw_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr optical_pose_sub_;
    rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr uav_pose_sub_;
    rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr uav_raw_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkFeedback>::SharedPtr feedback_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tracker_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tracker_offset_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr projected_objects_sub_;
    
public:
    TimestampMonitor() : Node("timestamp_monitor") {
        
        // CRITICAL: Declare use_sim_time parameter FIRST
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", false);
        }
        
        // Parameters
        this->declare_parameter("analysis_window_sec", analysis_window_sec_);
        this->declare_parameter("sync_tolerance_ms", sync_tolerance_ms_);
        this->declare_parameter("reference_topic", std::string("/firefly_1/xtion/rgb/image_raw"));
        this->declare_parameter("log_to_file", log_to_file_);
        this->declare_parameter("show_detailed_stats", show_detailed_stats_);
        this->declare_parameter("log_file_path", std::string("/tmp/timestamp_analysis.csv"));
        
        // Get parameters
        analysis_window_sec_ = this->get_parameter("analysis_window_sec").as_double();
        sync_tolerance_ms_ = this->get_parameter("sync_tolerance_ms").as_double();
        reference_topic_ = this->get_parameter("reference_topic").as_string();
        log_to_file_ = this->get_parameter("log_to_file").as_bool();
        show_detailed_stats_ = this->get_parameter("show_detailed_stats").as_bool();
        log_file_path_ = this->get_parameter("log_file_path").as_string();
        
        monitor_start_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Timestamp Monitor initialized:");
        RCLCPP_INFO(this->get_logger(), 
                   "  Analysis window: %.1fs", analysis_window_sec_);
        RCLCPP_INFO(this->get_logger(), 
                   "  Sync tolerance: %.1fms", sync_tolerance_ms_);
        RCLCPP_INFO(this->get_logger(), 
                   "  Reference topic: %s", reference_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                   "  Log to file: %s (%s)", log_to_file_ ? "yes" : "no", log_file_path_.c_str());
        
        // Initialize log file
        if (log_to_file_) {
            log_file_.open(log_file_path_);
            log_file_ << "timestamp,topic,message_timestamp,receive_timestamp,drift_ms,frequency_hz,sync_offset_ms\n";
        }
        
        // Initialize topic subscribers
        initializeSubscribers();
        
        // Create analysis timer (every 5 seconds)
        analysis_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TimestampMonitor::performAnalysis, this));
            
        // Create detailed stats timer (every 30 seconds)
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&TimestampMonitor::printDetailedStats, this));
        
        RCLCPP_INFO(this->get_logger(), "Monitoring %zu topics for timing analysis", topic_stats_.size());
    }
    
    ~TimestampMonitor() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }
    
private:
    void initializeSubscribers() {
        
        // Camera topics
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/firefly_1/xtion/rgb/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                recordMessage("/firefly_1/xtion/rgb/image_raw", "sensor_msgs/Image", msg->header);
            });
        
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                recordMessage("/camera/camera_info", "sensor_msgs/CameraInfo", msg->header);
            });
        
        // Robot pose topics
        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/pose/raww/std", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/pose/raww/std", "PoseWithCovarianceStamped", msg->header);
            });
            
        robot_raw_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/pose/corr/std", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/pose/corr/std", "PoseWithCovarianceStamped", msg->header);
            });
        
        // Camera frame topics
        camera_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/camera/pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/camera/pose", "PoseWithCovarianceStamped", msg->header);
            });
            
        optical_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/camera/pose_optical", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/camera/pose_optical", "PoseWithCovarianceStamped", msg->header);
            });
        
        // UAV pose topics
        uav_pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
            "/machine_1/pose", 10,
            [this](const uav_msgs::msg::UAVPose::SharedPtr msg) {
                recordMessage("/machine_1/pose", "uav_msgs/UAVPose", msg->header);
            });
            
        uav_raw_pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
            "/machine_1/pose/raw", 10,
            [this](const uav_msgs::msg::UAVPose::SharedPtr msg) {
                recordMessage("/machine_1/pose/raw", "uav_msgs/UAVPose", msg->header);
            });
        
        // Odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                recordMessage("/X3/odometry", "nav_msgs/Odometry", msg->header);
            });
        
        // Detection pipeline topics
        detection_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
            "/machine_1/detections", 10,
            [this](const neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr msg) {
                recordMessage("/machine_1/detections", "NeuralNetworkDetectionArray", msg->header);
            });
            
        feedback_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkFeedback>(
            "/machine_1/neural_network_feedback", 10,
            [this](const neural_network_msgs::msg::NeuralNetworkFeedback::SharedPtr msg) {
                recordMessage("/machine_1/neural_network_feedback", "NeuralNetworkFeedback", msg->header);
            });
        
        // Tracking topics
        tracker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/target_tracker/pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/target_tracker/pose", "PoseWithCovarianceStamped", msg->header);
            });
            
        tracker_offset_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/target_tracker/offset", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/target_tracker/offset", "PoseWithCovarianceStamped", msg->header);
            });
        
        // Projected objects
        projected_objects_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/machine_1/object_detections/projected_to_world", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                recordMessage("/machine_1/object_detections/projected_to_world", "PoseWithCovarianceStamped", msg->header);
            });
        
        RCLCPP_INFO(this->get_logger(), "Initialized subscribers for all critical topics");
    }
    
    void recordMessage(const std::string& topic_name, 
                      const std::string& message_type,
                      const std_msgs::msg::Header& header) {
        
        rclcpp::Time receive_time = this->get_clock()->now();
        rclcpp::Time message_time(header.stamp);
        
        // Validate timestamp
        if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), message_time)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Invalid timestamp in %s", topic_name.c_str());
            return;
        }
        
        auto& stats = topic_stats_[topic_name];
        stats.topic_name = topic_name;
        stats.message_type = message_type;
        stats.message_count++;
        stats.is_active = true;
        
        // Calculate timestamp drift (message time vs receive time)
        double drift_ms = (message_time - receive_time).seconds() * 1000.0;
        stats.timestamp_drift = drift_ms;
        stats.recent_drifts.push_back(drift_ms);
        if (stats.recent_drifts.size() > 10) {
            stats.recent_drifts.erase(stats.recent_drifts.begin());
        }
        
        // Calculate frequency
        if (stats.last_received.nanoseconds() > 0) {
            double interval = (receive_time - stats.last_received).seconds();
            if (interval > 0) {
                stats.recent_intervals.push_back(interval);
                if (stats.recent_intervals.size() > 10) {
                    stats.recent_intervals.erase(stats.recent_intervals.begin());
                }
                
                // Calculate average frequency
                if (!stats.recent_intervals.empty()) {
                    double avg_interval = 0;
                    for (double i : stats.recent_intervals) avg_interval += i;
                    avg_interval /= stats.recent_intervals.size();
                    stats.avg_frequency = (avg_interval > 0) ? 1.0 / avg_interval : 0.0;
                }
            }
        }
        
        stats.last_timestamp = message_time;
        stats.last_received = receive_time;
        
        // Update reference time for sync analysis
        if (topic_name == reference_topic_) {
            reference_time_ = message_time;
        }
        
        // Log to file if enabled
        if (log_to_file_ && log_file_.is_open()) {
            double sync_offset_ms = 0.0;
            if (reference_time_.nanoseconds() > 0) {
                sync_offset_ms = (message_time - reference_time_).seconds() * 1000.0;
            }
            
            log_file_ << std::fixed << std::setprecision(6)
                     << receive_time.seconds() << ","
                     << topic_name << ","
                     << message_time.seconds() << ","
                     << receive_time.seconds() << ","
                     << drift_ms << ","
                     << stats.avg_frequency << ","
                     << sync_offset_ms << "\n";
            log_file_.flush();
        }
    }
    
    void performAnalysis() {
        if (topic_stats_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No topics received yet...");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "\n========== TIMESTAMP ANALYSIS ==========");
        
        // Find active topics
        std::vector<std::string> active_topics;
        std::vector<std::string> inactive_topics;
        
        for (const auto& [topic, stats] : topic_stats_) {
            auto time_since_last = this->get_clock()->now() - stats.last_received;
            if (time_since_last.seconds() < 2.0) {
                active_topics.push_back(topic);
            } else {
                inactive_topics.push_back(topic);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Active topics: %zu, Inactive: %zu", 
                   active_topics.size(), inactive_topics.size());
        
        // Sync analysis relative to reference topic
        if (reference_time_.nanoseconds() > 0) {
            RCLCPP_INFO(this->get_logger(), "\n--- Synchronization Analysis (ref: %s) ---", 
                       reference_topic_.c_str());
            
            std::vector<std::pair<std::string, double>> sync_offsets;
            
            for (const auto& topic : active_topics) {
                const auto& stats = topic_stats_[topic];
                if (stats.last_timestamp.nanoseconds() > 0) {
                    double offset_ms = (stats.last_timestamp - reference_time_).seconds() * 1000.0;
                    sync_offsets.push_back({topic, offset_ms});
                    
                    std::string status = (std::abs(offset_ms) <= sync_tolerance_ms_) ? "✓ SYNC" : "✗ OUT_OF_SYNC";
                    RCLCPP_INFO(this->get_logger(), "  %-40s: %+8.1fms [%s]", 
                               topic.c_str(), offset_ms, status.c_str());
                }
            }
            
            // Find worst offenders
            std::sort(sync_offsets.begin(), sync_offsets.end(), 
                     [](const auto& a, const auto& b) { 
                         return std::abs(a.second) > std::abs(b.second); 
                     });
                     
            if (sync_offsets.size() > 1) {
                RCLCPP_INFO(this->get_logger(), "\nWorst sync offenders:");
                for (size_t i = 0; i < std::min(size_t(3), sync_offsets.size()); ++i) {
                    RCLCPP_INFO(this->get_logger(), "  %d. %-30s: %+8.1fms", 
                               int(i+1), sync_offsets[i].first.c_str(), sync_offsets[i].second);
                }
            }
        }
        
        // Frequency analysis
        RCLCPP_INFO(this->get_logger(), "\n--- Frequency Analysis ---");
        for (const auto& topic : active_topics) {
            const auto& stats = topic_stats_[topic];
            std::string freq_str = (stats.avg_frequency > 0) ? 
                std::to_string(stats.avg_frequency) + " Hz" : "Unknown";
            
            // Calculate average drift
            double avg_drift = 0.0;
            if (!stats.recent_drifts.empty()) {
                for (double d : stats.recent_drifts) avg_drift += d;
                avg_drift /= stats.recent_drifts.size();
            }
            
            RCLCPP_INFO(this->get_logger(), "  %-40s: %8s (drift: %+6.1fms, msgs: %zu)", 
                       topic.c_str(), freq_str.c_str(), avg_drift, stats.message_count);
        }
        
        // Inactive topics warning
        if (!inactive_topics.empty()) {
            RCLCPP_WARN(this->get_logger(), "\n--- INACTIVE TOPICS ---");
            for (const auto& topic : inactive_topics) {
                auto time_since_last = this->get_clock()->now() - topic_stats_[topic].last_received;
                RCLCPP_WARN(this->get_logger(), "  %-40s: silent for %.1fs", 
                           topic.c_str(), time_since_last.seconds());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "==========================================\n");
    }
    
    void printDetailedStats() {
        if (!show_detailed_stats_) return;
        
        RCLCPP_INFO(this->get_logger(), "\n========== DETAILED STATISTICS ==========");
        
        auto runtime = this->get_clock()->now() - monitor_start_time_;
        RCLCPP_INFO(this->get_logger(), "Monitor runtime: %.1f seconds", runtime.seconds());
        
        for (const auto& [topic, stats] : topic_stats_) {
            if (!stats.is_active) continue;
            
            RCLCPP_INFO(this->get_logger(), "\n--- %s ---", topic.c_str());
            RCLCPP_INFO(this->get_logger(), "  Type: %s", stats.message_type.c_str());
            RCLCPP_INFO(this->get_logger(), "  Messages: %zu", stats.message_count);
            RCLCPP_INFO(this->get_logger(), "  Frequency: %.2f Hz", stats.avg_frequency);
            RCLCPP_INFO(this->get_logger(), "  Last timestamp: %.6f", stats.last_timestamp.seconds());
            RCLCPP_INFO(this->get_logger(), "  Current drift: %+.1f ms", stats.timestamp_drift);
            
            if (!stats.recent_drifts.empty()) {
                double min_drift = *std::min_element(stats.recent_drifts.begin(), stats.recent_drifts.end());
                double max_drift = *std::max_element(stats.recent_drifts.begin(), stats.recent_drifts.end());
                RCLCPP_INFO(this->get_logger(), "  Drift range: [%.1f, %.1f] ms", min_drift, max_drift);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "==========================================\n");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TimestampMonitor>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting Timestamp Monitor...");
    RCLCPP_INFO(node->get_logger(), "This node will analyze timing relationships across all topics");
    RCLCPP_INFO(node->get_logger(), "Watch for 'OUT_OF_SYNC' warnings and frequency anomalies");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
