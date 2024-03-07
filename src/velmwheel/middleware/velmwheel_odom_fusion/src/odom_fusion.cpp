/* ============================================================================================================================ *//**
 * @file       odom_fusion.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:53 pm
 * @modified   Monday, 18th July 2022 8:22:25 pm
 * @project    engineering-thesis
 * @brief      Definitions of the ROS2 node class implementing Kalman-Filter-based processing mechanism for filtering odometry data
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
// Common includes
#include "node_common/node.hpp"
// TF includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
// Private includes
#include "velmwheel/odom_fusion.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Converts std::array to std::vector
 * 
 * @tparam T 
 *    type of the array elements
 * @tparam N 
 *    size of the array
 * @param array 
 *    array to be converted
 * @returns 
 *    vector containing elements copied from @p array
 */
template<typename T, std::size_t N>
static inline std::vector<T> to_vector(const std::array<T, N> &array) {

    std::vector<T> ret;

    // Reserve space for the vector
    ret.resize(N);
    // Copy elements
    std::copy(array.begin(), array.end(), ret.begin());

    return ret;
}

/* ========================================================= Ctors & dtors ======================================================== */

OdomFusion::OdomFusion(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{
    /* ----------------------------- Initialize parameters --------------------------- */
    
    auto control_acc_gain = node_common::parameters::declare_parameter_and_get( *this, CONTROL_ACC_GAIN_PARAM_DESCRIPTOR );
    auto control_acc_limits = node_common::parameters::declare_parameter_and_get( *this, CONTROL_ACC_LIMITS_PARAM_DESCRIPTOR );
    auto control_dec_gain = node_common::parameters::declare_parameter_and_get( *this, CONTROL_DEC_GAIN_PARAM_DESCRIPTOR );
    auto control_dec_limits = node_common::parameters::declare_parameter_and_get( *this, CONTROL_DEC_LIMITS_PARAM_DESCRIPTOR );
    auto control_timeout_s = node_common::parameters::declare_parameter_and_get( *this, CONTROL_TIMEOUT_S_PARAM_DESCRIPTOR );
    auto process_noise_covariance_matrix = node_common::parameters::declare_parameter_and_get( *this, PROCESS_NOISE_COVARIANCE_MATRIX_PARAM_DESCRIPTOR );
    auto sensor_timeout_s = node_common::parameters::declare_parameter_and_get( *this, SENSOR_TIMEOUT_S_PARAM_DESCRIPTOR );
    auto mahalanobis_measurement_threshold = node_common::parameters::declare_parameter_and_get( *this, MAHALANOBIS_MEASUREMENT_THRESHOLD_PARAM_DESCRIPTOR );
    auto debug_log_file = node_common::parameters::declare_parameter_and_get( *this, DEBUG_LOG_FILE_PARAM_DESCRIPTOR );

    /* ----------------------------- Validate parameters ----------------------------- */

    // Check if a 'control_acc_gain' is of a valid size
    if(control_acc_gain->size() != robot_localization::TWIST_SIZE)
        rclcpp::exceptions::InvalidParametersException("'control_acc_gain' gain array should have 6 element");

    // Check if a 'control_acc_limits' is of a valid size
    if(control_acc_limits->size() != robot_localization::TWIST_SIZE)
        rclcpp::exceptions::InvalidParametersException("'control_acc_limits' gain array should have 6 element");

    // Check if a 'control_dec_gain' is of a valid size
    if(control_dec_gain->size() != robot_localization::TWIST_SIZE)
        rclcpp::exceptions::InvalidParametersException("'control_dec_gain' gain array should have 6 element");

    // Check if a 'control_dec_limits' is of a valid size
    if(control_dec_limits->size() != robot_localization::TWIST_SIZE)
        rclcpp::exceptions::InvalidParametersException("'control_dec_limits' gain array should have 6 element");

    // Check if 'control_timeout_s' is apositive value
    if(*control_timeout_s <= 0.0)
        rclcpp::exceptions::InvalidParametersException("'control_timeout_s' value shall be positive");
        
    // Check if 'sensor_timeout_s' is apositive value
    if(*sensor_timeout_s <= 0.0)
        rclcpp::exceptions::InvalidParametersException("'sensor_timeout_s' value shall be positive");
        
    // Check if 'mahalanobis_measurement_threshold' is apositive value
    if(*mahalanobis_measurement_threshold <= 0.0)
        rclcpp::exceptions::InvalidParametersException("'mahalanobis_measurement_threshold' value shall be positive");
        
    // Check if 'process_noise_covariance_matrix' is of a valid size
    if(process_noise_covariance_matrix->size() != STATE_VECTOR_SIZE * STATE_VECTOR_SIZE)
        rclcpp::exceptions::InvalidParametersException("'process_noise_covariance_matrix' shall be 225-element vector");

    /* ---------------------------- Initialize subscribers --------------------------- */

    // Initialize velocity setpoint subscriber
    *node_common::communication::make_subscriber_builder(velocity_setpoint_sub)
        .node(*this)
        .name(VELOCITY_SETPOINT_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &OdomFusion::velocity_setpoint_callback);
    
    // Initialize encoder-based odometry data subscriber
    *node_common::communication::make_subscriber_builder(odom_sub)
        .node(*this)
        .name(ODOM_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &OdomFusion::odom_callback);
    
    // Initialize laser-based odometry data subscriber
    *node_common::communication::make_subscriber_builder(laser_odom_sub)
        .node(*this)
        .name(LASER_ODOM_SUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE)
        .callback(*this, &OdomFusion::laser_odom_callback);
    
    /* ----------------------------- Initialize publishers --------------------------- */
            
    // Initialize filtered odometry data publisher
    *node_common::communication::make_publisher_builder(odom_filtered_pub)
        .node(*this)
        .name(ODOM_FILTERED_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
            
    // Initialize filtered odometry pose data publisher
    *node_common::communication::make_publisher_builder(odom_filtered_pose_pub)
        .node(*this)
        .name(ODOM_FILTERED_POSE_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
            
    // Initialize filtered odometry twist data publisher
    *node_common::communication::make_publisher_builder(odom_filtered_twist_pub)
        .node(*this)
        .name(ODOM_FILTERED_TWIST_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    // Initialize TF2 broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    /* ---------------------------- Initialize ROS messages -------------------------- */

    // Initialize header of the ROS output message
    odom_msg.header.frame_id = TF_BASE_FRAME;
    odom_msg.child_frame_id = TF_OUT_FRAME;
    // Initialize header of the TF output transformation
    tf_out_msg.header.frame_id = TF_BASE_FRAME;
    tf_out_msg.child_frame_id = TF_OUT_FRAME;
    
    /* ------------------------------ Initialize EK filter --------------------------- */

    // Configure control model of the robot
    filter.setControlParams(
        to_vector(CONTROL_AFFECTION_VECTOR),
        rclcpp::Duration::from_seconds(*control_timeout_s),
        *control_acc_gain,
        *control_acc_limits,
        *control_dec_gain,
        *control_dec_limits
    );

    // Configure process uncertianty matrix (Q) of the system
    filter.setProcessNoiseCovariance(
        Eigen::Map<const Eigen::MatrixXd> {
            process_noise_covariance_matrix->data(),
            STATE_VECTOR_SIZE,
            STATE_VECTOR_SIZE
        }
    );
    
    // Configure timeout of sensors measurements
    filter.setSensorTimeout( rclcpp::Duration::from_seconds(*sensor_timeout_s) );

    // Init the last measurement time so we don't get a huge initial delta
    filter.setLastMeasurementTime( this->now() );

    // (Optional) Configure output debug log file
    if(debug_log_file.has_value() and (not debug_log_file->empty())) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Output log file set to '" << *debug_log_file << "'");

        // Construct output stream
        debug_out_stream.emplace(*debug_log_file);
        // Configure the stream in the filter
        filter.setDebug(true, &debug_out_stream.value());

    }

    // Keep Mahalanobis measurement threshold
    this->mahalanobis_measurement_threshold = *mahalanobis_measurement_threshold;

    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


OdomFusion::~OdomFusion() {
    node_common::node::print_goodbye(*this);
}

/* =========================================================== Callbacks ========================================================== */

void OdomFusion::velocity_setpoint_callback(const geometry_msgs::msg::Twist &msg) {

    // Get current time
    auto now = this->get_clock()->now();
    
    // Convert incoming message to 3D vector
    last_controls << 
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z;

    // Set current controls
    filter.setControl(last_controls, now);
    // Keep timestamp of hte message
    last_controls_stamp = now;

}


void OdomFusion::odom_callback(const nav_msgs::msg::Odometry &msg) {
    return;
    // Get current time
    auto now = this->get_clock()->now();

    Eigen::VectorXd measurement(MEASUREMENT_VECTOR_SIZE);

    // Convert incoming message into the measurement
    measurement <<
        /*  x  */ 0, 
        /*  y  */ 0,
        /*  z  */ 0,
        /*  ɑ  */ 0,
        /*  β  */ 0,
        /*  θ  */ 0,
        /* v⃗_x */ msg.twist.twist.linear.x, 
        /* v⃗_y */ msg.twist.twist.linear.y, 
        /* v⃗_z */ 0,
        /* ω⃗_ɑ */ 0,
        /* ω⃗_β */ 0,
        /* ω⃗_θ */ msg.twist.twist.angular.z;

    Eigen::VectorXd measurement_covariance_diagonal(MEASUREMENT_VECTOR_SIZE);

    // Extract measurements covariance from incoming message
    measurement_covariance_diagonal <<
        /*  x  */ 1,
        /*  y  */ 1,
        /*  z  */ 1,
        /*  ɑ  */ 1,
        /*  β  */ 1,
        /*  θ  */ 1,
        /* v⃗_x */ msg.twist.covariance[0],
        /* v⃗_y */ msg.twist.covariance[7],
        /* v⃗_z */ 1,
        /* ω⃗_ɑ */ 1,
        /* ω⃗_β */ 1,
        /* ω⃗_θ */ msg.twist.covariance[35];

    robot_localization::Measurement measurement_descriptor;
    
    // Fill the descriptor of the measurement passed to the filter
    measurement_descriptor.topic_name_          = ODOM_SUB_TOPIC_NAME;
    measurement_descriptor.measurement_         = measurement;
    measurement_descriptor.covariance_          = measurement_covariance_diagonal.asDiagonal();
    measurement_descriptor.update_vector_       = to_vector(ODOM_MEASURED_STATE);
    measurement_descriptor.time_                = now;
    measurement_descriptor.latest_control_      = last_controls;
    measurement_descriptor.mahalanobis_thresh_  = mahalanobis_measurement_threshold;
    measurement_descriptor.latest_control_time_ = last_controls_stamp;
    
    // Process measurement
    filter.processMeasurement(measurement_descriptor);
    // Publish result
    publish(msg.header.stamp);

}


void OdomFusion::laser_odom_callback(const geometry_msgs::msg::PoseStamped &msg) {

    // Get current time
    auto now = this->get_clock()->now();

    tf2::Quaternion q;

    // Convert message to the tf2::Quaternion
    tf2::fromMsg(msg.pose.orientation, q);

    Eigen::VectorXd measurement(MEASUREMENT_VECTOR_SIZE);

    // Convert incoming message into the measurement
    measurement <<
        /*  x  */ msg.pose.position.x, 
        /*  y  */ msg.pose.position.y,
        /*  z  */ 0,
        /*  ɑ  */ 0,
        /*  β  */ 0,
        /*  θ  */ tf2::getYaw(q),
        /* v⃗_x */ 0, 
        /* v⃗_y */ 0, 
        /* v⃗_z */ 0,
        /* ω⃗_ɑ */ 0,
        /* ω⃗_β */ 0,
        /* ω⃗_θ */ 0;

    Eigen::VectorXd measurement_covariance_diagonal(MEASUREMENT_VECTOR_SIZE);

    // Extract measurements covariance from incoming message
    measurement_covariance_diagonal <<
        /*  x  */ 0.01,
        /*  y  */ 0.01,
        /*  z  */ 1,
        /*  ɑ  */ 1,
        /*  β  */ 1,
        /*  θ  */ 0.01,
        /* v⃗_x */ 1,
        /* v⃗_y */ 1,
        /* v⃗_z */ 1,
        /* ω⃗_ɑ */ 1,
        /* ω⃗_β */ 1,
        /* ω⃗_θ */ 1;

    robot_localization::Measurement measurement_descriptor;

    // Fill the descriptor of the measurement passed to the filter
    measurement_descriptor.topic_name_          = LASER_ODOM_SUB_TOPIC_NAME;
    measurement_descriptor.measurement_         = measurement;
    measurement_descriptor.covariance_          = measurement_covariance_diagonal.asDiagonal();
    measurement_descriptor.update_vector_       = to_vector(LASER_ODOM_MEASURED_STATE);
    measurement_descriptor.time_                = now;
    measurement_descriptor.latest_control_      = last_controls;
    measurement_descriptor.mahalanobis_thresh_  = mahalanobis_measurement_threshold;
    measurement_descriptor.latest_control_time_ = last_controls_stamp;

    // Process measurement
    filter.processMeasurement(measurement_descriptor);
    // Publish result
    publish(msg.header.stamp);

}

/* ======================================================== Helper methods ======================================================== */

void OdomFusion::publish(const rclcpp::Time &now) {

    // Get current estimation of system state
    auto state = filter.getState();

    tf2::Quaternion orientation;

    // Calculate orientation quaternion
    orientation.setRPY(
        state(StateRoll),
        state(StatePitch),
        state(StateYaw)
    );

    // Normalize quaternion
    orientation.normalize();

    /* -------------------------- Publish odometry message --------------------------- */

    // Prepare header of the output message
    odom_msg.header.stamp = now;
    // Prepare body of the output message
    odom_msg.pose.pose.position.x    = state(StateX);
    odom_msg.pose.pose.position.y    = state(StateY);
    odom_msg.pose.pose.orientation.x = orientation.getX();
    odom_msg.pose.pose.orientation.y = orientation.getY();
    odom_msg.pose.pose.orientation.z = orientation.getZ();
    odom_msg.pose.pose.orientation.w = orientation.getW();
    odom_msg.twist.twist.linear.x    = state(StateVelX);
    odom_msg.twist.twist.linear.y    = state(StateVelY);
    odom_msg.twist.twist.angular.z   = state(StateVelZ);

    // Publish output message
    odom_filtered_pub->publish(odom_msg);

    /* ---------------------------- Publish pose message ----------------------------- */

    geometry_msgs::msg::PoseStamped pose_msg;

    // Prepare header of the output message
    pose_msg.header.stamp    = now;
    pose_msg.header.frame_id = TF_BASE_FRAME;
    // Prepare body of the output message
    pose_msg.pose = odom_msg.pose.pose;

    // Publish output message
    odom_filtered_pose_pub->publish(pose_msg);
    
    /* --------------------------- Publish twist message ----------------------------- */

    geometry_msgs::msg::TwistStamped twist_msg;

    // Prepare header of the output message
    twist_msg.header.stamp    = now;
    twist_msg.header.frame_id = TF_OUT_FRAME;
    // Prepare body of the output message
    twist_msg.twist = odom_msg.twist.twist;

    // Publish output message
    odom_filtered_twist_pub->publish(twist_msg);

    /* -------------------------- Publish TF transformation -------------------------- */

    // Prepare header of the output transform
    tf_out_msg.header.stamp = now;
    // Prepare body of the output transform
    tf_out_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_out_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_out_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_out_msg.transform.rotation.x    = odom_msg.pose.pose.orientation.x;
    tf_out_msg.transform.rotation.y    = odom_msg.pose.pose.orientation.y;
    tf_out_msg.transform.rotation.z    = odom_msg.pose.pose.orientation.z;
    tf_out_msg.transform.rotation.w    = odom_msg.pose.pose.orientation.w;

    // Publish transformation
    tf_broadcaster->sendTransform(tf_out_msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::OdomFusion)

/* ================================================================================================================================ */

