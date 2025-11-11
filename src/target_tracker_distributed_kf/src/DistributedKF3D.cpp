//
// ROS2 migration of DistributedKF3D
// Originally created by glawless on 23.05.17.
// Migrated to ROS2 maintaining exact functionality
//

#include <target_tracker_distributed_kf/DistributedKF3D.h>
#include <ros2_utils/clock_sync.hpp>
#include <cmath>

namespace target_tracker_distributed_kf {

static const std::string world_frame{"world"};

DistributedKF3D::DistributedKF3D() : 
    Node("distributed_kf3d"),
    I((int) state_size, (int) state_size),
    Hself((int) measurement_state_size, (int) state_size),
    Hother((int) measurement_state_size, (int) state_size),
    R((int) state_size, (int) state_size) {
    
    // Declare use_sim_time parameter FIRST
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", false);
    }
    
    // Declare all parameters before using them
    this->declare_parameter("initialUncertaintyPosXY", initialUncertaintyPosXY);
    this->declare_parameter("initialUncertaintyPosZ", initialUncertaintyPosZ);
    this->declare_parameter("initialUncertaintyVelXY", initialUncertaintyVelXY);
    this->declare_parameter("initialUncertaintyVelZ", initialUncertaintyVelZ);
    this->declare_parameter("initialUncertaintyOffsetXY", initialUncertaintyOffsetXY);
    this->declare_parameter("initialUncertaintyOffsetZ", initialUncertaintyOffsetZ);
    this->declare_parameter("falsePositiveThresholdSigma", falsePositiveThresholdSigma);
    
    // Dynamic reconfigure equivalent parameters
    this->declare_parameter("posGlobalOffsetBiasX", posGlobalOffsetBiasX);
    this->declare_parameter("posGlobalOffsetBiasY", posGlobalOffsetBiasY);
    this->declare_parameter("posGlobalOffsetBiasZ", posGlobalOffsetBiasZ);
    this->declare_parameter("noisePosXVar", noisePosXVar);
    this->declare_parameter("noisePosYVar", noisePosYVar);
    this->declare_parameter("noisePosZVar", noisePosZVar);
    this->declare_parameter("noiseVelXVar", noiseVelXVar);
    this->declare_parameter("noiseVelYVar", noiseVelYVar);
    this->declare_parameter("noiseVelZVar", noiseVelZVar);
    this->declare_parameter("noiseOffXVar", noiseOffXVar);
    this->declare_parameter("noiseOffYVar", noiseOffYVar);
    this->declare_parameter("noiseOffZVar", noiseOffZVar);
    this->declare_parameter("velocityDecayTime", velocityDecayTime);
    this->declare_parameter("offsetDecayTime", offsetDecayTime);
    
    // Topic parameters
    this->declare_parameter("pub_topic", "/machine_1/target_tracker/pose");
    this->declare_parameter("velPub_topic", "/machine_1/target_tracker/twist");
    this->declare_parameter("offset_topic", "/machine_1/target_tracker/offset");
    this->declare_parameter("pose_topic", "/machine_1/pose");
    this->declare_parameter("measurement_topic_suffix_self", "/machine_1/object_detections/projected_to_world");
    this->declare_parameter("measurement_topic_suffix", "object_detections/projected_to_world_network");
    
    // Robot configuration
    this->declare_parameter("robotID", 1);
    this->declare_parameter("numRobots", 1);
    
    // Other parameters
    this->declare_parameter("reset_time_threshold", time_threshold);
    this->declare_parameter("cache_size", 40);
    
    // Get parameter values
    this->get_parameter("initialUncertaintyPosXY", initialUncertaintyPosXY);
    this->get_parameter("initialUncertaintyPosZ", initialUncertaintyPosZ);
    this->get_parameter("initialUncertaintyVelXY", initialUncertaintyVelXY);
    this->get_parameter("initialUncertaintyVelZ", initialUncertaintyVelZ);
    this->get_parameter("initialUncertaintyOffsetXY", initialUncertaintyOffsetXY);
    this->get_parameter("initialUncertaintyOffsetZ", initialUncertaintyOffsetZ);
    this->get_parameter("falsePositiveThresholdSigma", falsePositiveThresholdSigma);
    this->get_parameter("reset_time_threshold", time_threshold);
    
    // Get use_sim_time status for logging
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using simulation time");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using system time");
    }
    
    // Initialize cache
    int cache_size = 20;
    this->get_parameter("cache_size", cache_size);
    state_cache_.set_cache_size((std::size_t) cache_size);
    
    // Initialize matrices
    initializeStaticMatrices();
    
    // Set up parameter callback for dynamic reconfigure equivalent
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DistributedKF3D::parametersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "DistributedKF3D node constructed");
}

void DistributedKF3D::initialize() {
    // This method is called after construction when shared_from_this is safe
    
    // Initialize last_time_ to current time
    last_time_ = this->get_clock()->now();
    
    // Initialize publishers
    std::string pub_topic = this->get_parameter("pub_topic").as_string();
    std::string velPub_topic = this->get_parameter("velPub_topic").as_string();
    std::string offset_topic = this->get_parameter("offset_topic").as_string();
    
    targetPub_ = this->create_publisher<PoseWithCovarianceStamped>(pub_topic, 10);
    targetVelPub_ = this->create_publisher<TwistWithCovarianceStamped>(velPub_topic, 10);
    offsetPub_ = this->create_publisher<PoseWithCovarianceStamped>(offset_topic, 10);
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to " << targetPub_->get_topic_name());
    RCLCPP_INFO_STREAM(this->get_logger(), "Offset Publishing to " << offsetPub_->get_topic_name());
    
    // Initialize the filter
    initializeFilter();
    
    // Initialize subscribers (which might use shared_from_this)
    initializeSubscribers();
    
    RCLCPP_INFO(this->get_logger(), "DistributedKF3D initialization complete");
}

void DistributedKF3D::initializeSubscribers() {
    // Self and other robots info
    int robotID = this->get_parameter("robotID").as_int();
    int numRobots = this->get_parameter("numRobots").as_int();
    
    // Pose subscriber
    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        pose_topic, 300,
        std::bind(&DistributedKF3D::predictAndPublish, this, std::placeholders::_1));
    
    // Measurement subscribers
    std::string measurement_suffix_self = this->get_parameter("measurement_topic_suffix_self").as_string();
    std::string measurement_suffix = this->get_parameter("measurement_topic_suffix").as_string();
    
    selfcallbackhandler = std::make_unique<Callbackhandler>(this, true, robotID);
    self_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
        measurement_suffix_self, 50,
        std::bind(&Callbackhandler::callback, selfcallbackhandler.get(), std::placeholders::_1));
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
        "Registered self measurement subscriber for topic " << self_sub_->get_topic_name());
    
    for (int robot = 1; robot <= numRobots; robot++) {
        if (robot == robotID)
            continue;
        
        auto cb = std::make_shared<Callbackhandler>(this, false, robot);
        callbackhandlers.push_back(cb);
        
        const auto other_topic = "/machine_" + to_string(robot) + '/' + measurement_suffix;
        auto sub = this->create_subscription<PoseWithCovarianceStamped>(
            other_topic, 50,
            std::bind(&Callbackhandler::callback, cb.get(), std::placeholders::_1));
        
        other_subs_.push_back(sub);
        
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Registered other robot's measurements subscriber for topic " << sub->get_topic_name());
    }
}

void DistributedKF3D::initializeFilter() {    
    // If there is a last element, grab its frame id; if not, use default world_frame
    std::string frame_id{world_frame};
    if (!state_cache_.empty())
        frame_id = state_cache_.back().frame_id;
    
    // Reset the cache
    state_cache_.clear();
    
    // Put an initial unknown estimate in the cache
    std_msgs::msg::Header h;
    h.frame_id = frame_id;
    h.stamp = this->get_clock()->now();
    CacheElement first_element(h, state_size, true, 0);
    setUnknownInitial(first_element);
    first_element.frame_id = frame_id;
    state_cache_.insert_ordered(first_element);
    
    RCLCPP_INFO(this->get_logger(), "The filter was (re)initialized");
}

void DistributedKF3D::measurementsCallback(const PoseWithCovarianceStamped::SharedPtr msg, const bool isSelf, const int robot) {
    if (detectBackwardsTimeJump()) {
        RCLCPP_WARN(this->get_logger(), "Backwards time jump in cache - ignoring update");
        return;
    }
        
    if (state_cache_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cache is empty - ignoring update");
        return;
    }
    
    
    // Create a new element for the cache
    CacheElement new_element(state_size, *msg, isSelf, robot);
    
    // Insert this element into cache, which returns the iterator at insert position
    auto it = state_cache_.insert_ordered(new_element);
    
    // Check if failure to insert - this would be due to a very old message
    if (it == state_cache_.end()) {
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Trying to insert a measurement that is too old! This is its stamp " 
            << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl
            << "Did you forget to reinitialize the node after going back to the past?");
        return;
    }
    
    // Rare, but may occur
    if(it == state_cache_.begin())
        ++it;
    
    // In a loop until we go through the whole cache, keep predicting and updating
    for (; it != state_cache_.end(); ++it) {
        if (!predict(*(it - 1), *it)) {
            RCLCPP_WARN(this->get_logger(), "Prediction step failed!");
            return;
        }
        if (it->measurements.size() > 0) {
            if (!update(*it)) {
                RCLCPP_WARN(this->get_logger(), "Rewind/Update failed!");
                return;
            }
        }
    }
}

void DistributedKF3D::setUnknownInitial(CacheElement &elem) {
    elem.cov << initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        , 0.0, initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        , 0.0, 0.0, initialUncertaintyPosZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        , 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0, 0.0
        , 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0
        , 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelZ, 0.0, 0.0, 0.0
        , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0, 0.0
        , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0
        , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetZ;
}

bool DistributedKF3D::predict(const CacheElement &in, CacheElement &out) {
    // Easy access
    const VectorXd &ins = in.state;
    VectorXd &outs = out.state;

    // Time past from one to next
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), out.stamp) || !ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), in.stamp) ) {
        RCLCPP_WARN(this->get_logger(),"One of the stamps is invalid, returning false from predict() without doing anything else");
        return false;
    }

    
        // Convert to seconds first to avoid clock source comparison
        int64_t out_ns = out.stamp.seconds();
        int64_t in_ns = in.stamp.seconds();
        const double deltaT = out_ns - in_ns ;  
        
        // if (deltaT < 0) {
        //     RCLCPP_WARN(this->get_logger(), "Negative time delta detected: %.3f", deltaT);
        //     return false;
        // }
    
    if (deltaT > time_threshold) {
        RCLCPP_WARN_STREAM(this->get_logger(), 
            "It's been a long time since there was an update (" << deltaT
            << " seconds). Resetting the filter to be safe... position(0,0,0) and high uncertainty");
        initializeFilter();
        return false;
    }
    
    const static double velocityDecayTo = 0.1;
    const double velocityDecayAlpha = pow(velocityDecayTo, 1.0 / velocityDecayTime);
    const double velocityDecayFactor = pow(velocityDecayAlpha, deltaT);
    const double velocityIntegralFactor = (velocityDecayFactor - 1) / log(velocityDecayAlpha);
    
    // Decreasing velocity model
    outs(0) = ins(0) + ins(3) * velocityIntegralFactor;
    outs(1) = ins(1) + ins(4) * velocityIntegralFactor;
    outs(2) = ins(2) + ins(5) * velocityIntegralFactor;
    
    outs(3) = ins(3) * velocityDecayFactor;
    outs(4) = ins(4) * velocityDecayFactor;
    outs(5) = ins(5) * velocityDecayFactor;
    
    const static double offsetDecayTo = 0.1;
    const double offsetDecayAlpha = pow(offsetDecayTo, 1.0 / offsetDecayTime);
    const double offsetDecayFactor = pow(offsetDecayAlpha, deltaT);
    
    outs(6) = posGlobalOffsetBiasX + ((ins(6) - posGlobalOffsetBiasX) * offsetDecayFactor);
    outs(7) = posGlobalOffsetBiasY + ((ins(7) - posGlobalOffsetBiasY) * offsetDecayFactor);
    outs(8) = posGlobalOffsetBiasZ + ((ins(8) - posGlobalOffsetBiasZ) * offsetDecayFactor);
    
    // Construct jacobian G based on deltaT
    MatrixXd G((int) state_size, (int) state_size);
    populateJacobianG(G, deltaT);
    
    // Update covariance from one to next
    out.cov = MatrixXd((G * in.cov * G.transpose()) + (deltaT / 1.0) * R);
    
    return true;
}

// Helper functions to replace MRPT dependency - add these to your class or as static functions
static double calculateMahalanobisDistance(const Eigen::Vector3d& x, 
                                         const Eigen::Vector3d& mean,
                                         const Eigen::Matrix3d& cov) {
    Eigen::Vector3d diff = x - mean;
    
    // Calculate inverse of covariance matrix
    Eigen::Matrix3d cov_inv;
    bool invertible;
    double determinant;
    cov.computeInverseAndDetWithCheck(cov_inv, determinant, invertible);
    
    if (!invertible || determinant <= 0) {
        // Covariance matrix is not positive definite
        return std::numeric_limits<double>::max();
    }
    
    // Mahalanobis distance squared
    double dist_sq = diff.transpose() * cov_inv * diff;
    return std::sqrt(dist_sq);
}

// Calculate normalized probability density ratio
static double calculateNormalizedDensityRatio(const Eigen::Vector3d& measurement,
                                            const Eigen::Vector3d& state,
                                            const Eigen::Matrix3d& cov) {
    // For multivariate normal distribution, the density ratio at measurement vs at mean is:
    // exp(-0.5 * (x-mu)' * Sigma^(-1) * (x-mu))
    
    double mahal_dist = calculateMahalanobisDistance(measurement, state, cov);
    
    // The density ratio is exp(-0.5 * d^2) where d is Mahalanobis distance
    double density_ratio = std::exp(-0.5 * mahal_dist * mahal_dist);
    
    return density_ratio;
}

// Replace the MRPT-dependent false positive detection logic in your update() method
// with this implementation that produces exactly the same numerical outputs

bool DistributedKF3D::update(CacheElement &elem) {
    if (elem.measurements.empty()) {
        RCLCPP_WARN(this->get_logger(), 
            "Tried to perform update step with no measurements in element. Returning without doing anything");
        return false;
    }
    
    // Find out closest measurement to current state estimate and use that one
    int closest_idx = -1;
    double min_error{DBL_MAX};
    for (size_t i = 0; i < elem.measurements.size(); ++i) {
        const auto measurement = elem.measurements[i];
        
        double difference[3]{measurement->pose.position.x - elem.state(0),
            measurement->pose.position.y - elem.state(1),
            measurement->pose.position.z - elem.state(2)};
        
        double sqr_error{
            sqrt(difference[0] * difference[0] + difference[1] * difference[1] + difference[2] * difference[2])};
        
        if (sqr_error < min_error) {
            min_error = sqr_error;
            closest_idx = i;
        }
    }
    
    if (closest_idx < 0 || closest_idx > (int) elem.measurements.size()) {
        RCLCPP_ERROR(this->get_logger(), "Something went wrong, couldn't find the closest measurement");
        return false;
    }
    
    const auto closest_measurement = elem.measurements[closest_idx];
    bool isSelf = elem.isSelfRobot;
    
    if (isSelf) {
        // Check if measurement is a false positive
        PoseWithCovariance state;
        state.pose.position.x = elem.state(0);
        state.pose.position.y = elem.state(1);
        state.pose.position.z = elem.state(2);
        state.pose.orientation.w = 1;
        state.pose.orientation.x = 0;
        state.pose.orientation.y = 0;
        state.pose.orientation.z = 0;
        state.covariance = { 
            elem.cov(0*9+0), elem.cov(0*9+1), elem.cov(0*9+2), 0, 0, 0,
            elem.cov(1*9+0), elem.cov(1*9+1), elem.cov(1*9+2), 0, 0, 0,
            elem.cov(2*9+0), elem.cov(2*9+1), elem.cov(2*9+2), 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0
        };
        
        // The measurement distribution is the distribution of the measurement with zero mean
        PoseWithCovariance distribution(*closest_measurement);
        distribution.pose.position.x = 0;
        distribution.pose.position.y = 0;
        distribution.pose.position.z = 0;
        distribution.pose.orientation = state.pose.orientation;
        
        // State merged with measurement distribution gives the expected mean distribution
        PoseWithCovariance merged;
        pose_cov_ops::compose(state, distribution, merged);
        
        // Extract position data for false positive detection
        Eigen::Vector3d state_mean;
        state_mean << elem.state(0), elem.state(1), elem.state(2);
        
        Eigen::Vector3d measurement_mean;
        measurement_mean << closest_measurement->pose.position.x, 
                           closest_measurement->pose.position.y, 
                           closest_measurement->pose.position.z;
        
        // Extract 3x3 position covariance from merged result
        Eigen::Matrix3d expectance_cov;
        expectance_cov << merged.covariance[0*6+0], merged.covariance[0*6+1], merged.covariance[0*6+2],
                         merged.covariance[1*6+0], merged.covariance[1*6+1], merged.covariance[1*6+2],
                         merged.covariance[2*6+0], merged.covariance[2*6+1], merged.covariance[2*6+2];
        
        // Calculate normalized density ratio using pure Eigen (no MRPT)
        double density = calculateNormalizedDensityRatio(measurement_mean, state_mean, expectance_cov);
        
        // For normalized density with sigma=1 and mu=0: e^(-1/2 * x^2)
        // then x = sqrt(-2*log(density))
        double x;
        if (density > 0 && density <= 1) {
            x = sqrt(-2.0 * log(density));
        } else {
            // Handle edge cases
            x = 10.0; // Large value to indicate very unlikely measurement
        }
        
        // Ignore measurement for self pose estimation if less likely than threshold
        if (x > falsePositiveThresholdSigma) {
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "Person Measurement likelihood " << x << " * sigma beyond threshold of " 
                << falsePositiveThresholdSigma << " * sigma. Discarding!");
            return true;
        }
    }
    
    // Rest of the update method remains exactly the same...
    const auto &H = isSelf ? Hself : Hother;
    
    MatrixXd Q((int) measurement_state_size, (int) measurement_state_size);
    Q << closest_measurement->covariance[0], closest_measurement->covariance[1], closest_measurement->covariance[2], 0.0, 0.0, 0.0,
         closest_measurement->covariance[6], closest_measurement->covariance[7], closest_measurement->covariance[8], 0.0, 0.0, 0.0,
         closest_measurement->covariance[12], closest_measurement->covariance[13], closest_measurement->covariance[14], 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, elem.cov(0) + closest_measurement->covariance[0], elem.cov(1) + closest_measurement->covariance[1], elem.cov(2) + closest_measurement->covariance[2],
         0.0, 0.0, 0.0, elem.cov(9) + closest_measurement->covariance[6], elem.cov(10) + closest_measurement->covariance[7], elem.cov(11) + closest_measurement->covariance[8],
         0.0, 0.0, 0.0, elem.cov(18) + closest_measurement->covariance[12], elem.cov(19) + closest_measurement->covariance[13], elem.cov(20) + closest_measurement->covariance[14];
    
    MatrixXd K = elem.cov * H.transpose() * (H * elem.cov * H.transpose() + Q).inverse();
    
    VectorXd e_measurement((int) measurement_state_size);
    
    // We aren't really measuring the offset, we can only measure the difference between observed and predicted target
    double measured_offset_x = elem.state(6) - (closest_measurement->pose.position.x - elem.state(0));
    double measured_offset_y = elem.state(7) - (closest_measurement->pose.position.y - elem.state(1));
    double measured_offset_z = elem.state(8) - (closest_measurement->pose.position.z - elem.state(2));
    
    e_measurement << closest_measurement->pose.position.x, closest_measurement->pose.position.y, 
                     closest_measurement->pose.position.z, measured_offset_x, measured_offset_y, measured_offset_z;
    
    VectorXd e_predicted((int) measurement_state_size);
    e_predicted << elem.state(0), elem.state(1), elem.state(2), elem.state(6), elem.state(7), elem.state(8);
    
    // Update
    elem.state = elem.state + K * (e_measurement - e_predicted);
    elem.cov = (I - K * H) * elem.cov;
    
    return true;
}


void DistributedKF3D::predictAndPublish(const uav_msgs::msg::UAVPose::SharedPtr pose) {    
    if (state_cache_.empty())
        return;
    
    // Always self robot because predict is only called for self poses
    CacheElement tmp_element(pose->header, state_size, true, 0);
    
    const auto last = state_cache_.back();
    if (!predict(last, tmp_element))
        return;
    
    publishStateAndCov(tmp_element);
}

void DistributedKF3D::initializeStaticMatrices() {
    I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    Hself << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    Hother << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
    R << noisePosXVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, noisePosYVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, noisePosZVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, noiseVelXVar, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, noiseVelYVar, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, noiseVelZVar, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffXVar, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffYVar, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffZVar;
}

void DistributedKF3D::populateJacobianG(MatrixXd &G, const double deltaT) {
    // Offset assumed independent from target detection
    G << 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

void DistributedKF3D::populateJacobianQ(MatrixXd &Q, const PoseWithCovariance &pcov) {
    Q << pcov.covariance[0], pcov.covariance[1], pcov.covariance[2], 0.0, 0.0, 0.0,
         pcov.covariance[6], pcov.covariance[7], pcov.covariance[8], 0.0, 0.0, 0.0,
         pcov.covariance[12], pcov.covariance[13], pcov.covariance[14], 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, pcov.covariance[0], pcov.covariance[1], pcov.covariance[2],
         0.0, 0.0, 0.0, pcov.covariance[6], pcov.covariance[7], pcov.covariance[8],
         0.0, 0.0, 0.0, pcov.covariance[12], pcov.covariance[13], pcov.covariance[14];
}

void DistributedKF3D::publishStateAndCov(const CacheElement &elem) {
    msg_.header.frame_id = elem.frame_id;
    msg_.header.stamp = elem.stamp;
    
    msg_.pose.pose.position.x = elem.state[0];
    msg_.pose.pose.position.y = elem.state[1];
    msg_.pose.pose.position.z = elem.state[2];
    msg_.pose.covariance[0 * 6 + 0] = elem.cov(0 * 9 + 0);
    msg_.pose.covariance[0 * 6 + 1] = elem.cov(0 * 9 + 1);
    msg_.pose.covariance[0 * 6 + 2] = elem.cov(0 * 9 + 2);
    msg_.pose.covariance[1 * 6 + 0] = elem.cov(1 * 9 + 0);
    msg_.pose.covariance[1 * 6 + 1] = elem.cov(1 * 9 + 1);
    msg_.pose.covariance[1 * 6 + 2] = elem.cov(1 * 9 + 2);
    msg_.pose.covariance[2 * 6 + 0] = elem.cov(2 * 9 + 0);
    msg_.pose.covariance[2 * 6 + 1] = elem.cov(2 * 9 + 1);
    msg_.pose.covariance[2 * 6 + 2] = elem.cov(2 * 9 + 2);
    
    msg_.pose.pose.orientation.w = 1.0;
    msg_.pose.pose.orientation.x = 0.0;
    msg_.pose.pose.orientation.y = 0.0;
    msg_.pose.pose.orientation.z = 0.0;
    
    targetPub_->publish(msg_);
    
    velMsg_.header.frame_id = elem.frame_id;
    velMsg_.header.stamp = elem.stamp;
    velMsg_.twist.twist.linear.x = elem.state[3];
    velMsg_.twist.twist.linear.y = elem.state[4];
    velMsg_.twist.twist.linear.z = elem.state[5];
    velMsg_.twist.covariance[0 * 6 + 0] = elem.cov(3 * 9 + 3);
    velMsg_.twist.covariance[0 * 6 + 1] = elem.cov(3 * 9 + 4);
    velMsg_.twist.covariance[0 * 6 + 2] = elem.cov(3 * 9 + 5);
    velMsg_.twist.covariance[1 * 6 + 0] = elem.cov(4 * 9 + 3);
    velMsg_.twist.covariance[1 * 6 + 1] = elem.cov(4 * 9 + 4);
    velMsg_.twist.covariance[1 * 6 + 2] = elem.cov(4 * 9 + 5);
    velMsg_.twist.covariance[2 * 6 + 0] = elem.cov(5 * 9 + 3);
    velMsg_.twist.covariance[2 * 6 + 1] = elem.cov(5 * 9 + 4);
    velMsg_.twist.covariance[2 * 6 + 2] = elem.cov(5 * 9 + 5);
    
    targetVelPub_->publish(velMsg_);
    
    // Publish offset
    msg_.pose.pose.position.x = elem.state[6];
    msg_.pose.pose.position.y = elem.state[7];
    msg_.pose.pose.position.z = elem.state[8];
    msg_.pose.covariance[0 * 6 + 0] = elem.cov(6 * 9 + 6);
    msg_.pose.covariance[0 * 6 + 1] = elem.cov(6 * 9 + 7);
    msg_.pose.covariance[0 * 6 + 2] = elem.cov(6 * 9 + 8);
    msg_.pose.covariance[1 * 6 + 0] = elem.cov(7 * 9 + 6);
    msg_.pose.covariance[1 * 6 + 1] = elem.cov(7 * 9 + 7);
    msg_.pose.covariance[1 * 6 + 2] = elem.cov(7 * 9 + 8);
    msg_.pose.covariance[2 * 6 + 0] = elem.cov(8 * 9 + 6);
    msg_.pose.covariance[2 * 6 + 1] = elem.cov(8 * 9 + 7);
    msg_.pose.covariance[2 * 6 + 2] = elem.cov(8 * 9 + 8);
    
    offsetPub_->publish(msg_);
}

rcl_interfaces::msg::SetParametersResult DistributedKF3D::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    RCLCPP_INFO(this->get_logger(), "Received parameter update request");
    
    for (const auto & param : parameters) {
        if (param.get_name() == "noisePosXVar") {
            noisePosXVar = param.as_double();
        } else if (param.get_name() == "noisePosYVar") {
            noisePosYVar = param.as_double();
        } else if (param.get_name() == "noisePosZVar") {
            noisePosZVar = param.as_double();
        } else if (param.get_name() == "noiseVelXVar") {
            noiseVelXVar = param.as_double();
        } else if (param.get_name() == "noiseVelYVar") {
            noiseVelYVar = param.as_double();
        } else if (param.get_name() == "noiseVelZVar") {
            noiseVelZVar = param.as_double();
        } else if (param.get_name() == "noiseOffXVar") {
            noiseOffXVar = param.as_double();
        } else if (param.get_name() == "noiseOffYVar") {
            noiseOffYVar = param.as_double();
        } else if (param.get_name() == "noiseOffZVar") {
            noiseOffZVar = param.as_double();
        } else if (param.get_name() == "posGlobalOffsetBiasX") {
            posGlobalOffsetBiasX = param.as_double();
        } else if (param.get_name() == "posGlobalOffsetBiasY") {
            posGlobalOffsetBiasY = param.as_double();
        } else if (param.get_name() == "posGlobalOffsetBiasZ") {
            posGlobalOffsetBiasZ = param.as_double();
        } else if (param.get_name() == "velocityDecayTime") {
            velocityDecayTime = param.as_double();
        } else if (param.get_name() == "offsetDecayTime") {
            offsetDecayTime = param.as_double();
        } else if (param.get_name() == "falsePositiveThresholdSigma") {
            falsePositiveThresholdSigma = param.as_double();
        }
    }
    
    // Reinitialize matrices
    initializeStaticMatrices();
    RCLCPP_INFO_STREAM(this->get_logger(), "Process noise matrix" << std::endl << R);
    
    // Reinitialize filter
    initializeFilter();
    
    return result;
}

bool DistributedKF3D::detectBackwardsTimeJump() {
    // Do not detect if not using sim time
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (!use_sim_time)
        return false;
    
    rclcpp::Time current_time = this->get_clock()->now();
    
    // Use nanoseconds for comparison to avoid clock source issues
    if (last_time_.nanoseconds() > 0 && 
        current_time.nanoseconds() < last_time_.nanoseconds()) {
        // Jump backwards detected, reset interface
        RCLCPP_WARN(this->get_logger(), "Backwards jump in time detected, performing reset");
        initializeFilter();
        last_time_ = current_time;
        return true;
    }
    last_time_ = current_time;
    return false;
}

void Callbackhandler::callback(const PoseWithCovarianceStamped::SharedPtr msg) {
    parent->measurementsCallback(msg, isSelf, robotID);
}


} // namespace target_tracker_distributed_kf
