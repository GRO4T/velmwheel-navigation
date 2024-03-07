/* ============================================================================================================================ *//**
 * @file       base_controller.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Monday, 18th July 2022 8:18:15 pm
 * @project    engineering-thesis
 * @brief      ROS2-based class implementing controll node responsible for basic conrol over the Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_BASE_CONTROLLER_H__
#define __VELMWHEEL_BASE_CONTROLLER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <utility>
#include <optional>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
// TF2 includes
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Private includes
#include "velmwheel/math.hpp"
#include "velmwheel/params.hpp"
#include "velmwheel_msgs/msg/wheels.hpp"
#include "velmwheel_msgs/msg/encoders.hpp"
#include "velmwheel_msgs/msg/encoders_stamped.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2-based class implementing basic controll node responsible for:
 * 
 *    - translation of velocity targets for the robot into the
 *      velocity targets for wheels and periodical sending controll
 *      signals on the predefined topic
 *    - translating current velocities of robot's wheels into
 *      the robot's velocity
 *    - providing basic odometry data
 * 
 */
class RCLCPP_PUBLIC BaseController : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "base_controller";

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Name of the topic subscribed by the node to acquire current encoders measurements from the robot
    static constexpr auto ENCODERS_SUB_TOPIC_NAME = "base/encoders";
    /// Name of the topic subscribed by the node to acquire current velocity sepoint for the robot
    static constexpr auto VELOCITY_SETPOINT_SUB_TOPIC_NAME = "base/velocity_setpoint";

    /// Name of the topic published by the node to provide controls for robot's wheels
    static constexpr auto CONTROLS_PUB_TOPIC_NAME = "base/controls";
    /// Name of the topic published by the node to broadcast current velocity of the robot
    static constexpr auto VELOCITY_PUB_TOPIC_NAME = "base/velocity";
    /// Name of the topic published by the node to broadcast basic odometry data
    static constexpr auto ODOM_PUB_TOPIC_NAME = "odom/encoders";
    /// Name of the topic published by the node to broadcast basic odometry data as a pose
    static constexpr auto ODOM_POSE_PUB_TOPIC_NAME = "odom/encoders/pose";
    /// Name of the topic to set new odometry position.
    static constexpr auto ODOM_SET_POSE_TOPIC_NAME = "odom/encoders/set_pose";

    /// Name of the TF reference frame for odometry data
    static constexpr auto ODOM_FRAME = "odom";
    /// Name of the TF child frame of the odometry reference frame
    static constexpr auto ODOM_CHILD_FRAME = "odom_pose";

public: /* ----------------------------------------------- Node' configuration ---------------------------------------------------- */

    /// Default covariance matrix for odometry estimations 
    static constexpr std::array<double, 36> DEFAULT_ODOM_COVARIANCE 
    { 
        0.000'1, 0.000'0, 0.000'0, 0.000'0, 0.000'0, 0.000'0,
        0.000'0, 0.000'1, 0.000'0, 0.000'0, 0.000'0, 0.000'0,
        0.000'0, 0.000'0, 0.000'1, 0.000'0, 0.000'0, 0.000'0,
        0.000'0, 0.000'0, 0.000'0, 0.000'1, 0.000'0, 0.000'0,
        0.000'0, 0.000'0, 0.000'0, 0.000'0, 0.000'1, 0.000'0,
        0.000'0, 0.000'0, 0.000'0, 0.000'0, 0.000'0, 0.000'1
    };

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new BaseController node
     * 
     * @param options 
     *    configuration of the node
     */
    BaseController(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructs BaseController node
     */
    ~BaseController();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback method called at arrival of the new measurements from robot's wheels encoders
     */
    void encoders_measurement_callback(const velmwheel_msgs::msg::EncodersStamped &msg);
    
    /**
     * @brief Callback method called at arrival of the new velocity setpoint for the robot
     */
    void velocity_setpoint_callback(const geometry_msgs::msg::Twist &msg);

    // TODO: add comment
    void set_pose_callback(const geometry_msgs::msg::Pose& msg);

    /**
     * @brief Callback method called periodically to compute current control signalsd
     */
    void controlls_calculation_callback();
    
private: /* ----------------------------------------------- Auxiliary methods ----------------------------------------------------- */

    /**
     * @retval true 
     *    if odometry data has been already initialized
     * @retval false 
     *    otherwise
     */
    inline bool is_odom_initialized() const;

    /**
     * @brief Initializes odoemtry measurement state based on the first incoming measurements
     * 
     * @param current_angles 
     *    current encoders' angles
     * @returns 
     *    initial positional odometry data
     */
    geometry_msgs::msg::Pose initialize_odom(const velmwheel_msgs::msg::Wheels &current_angles);

    /**
     * @brief Calculates current odometry data for the Velmwheel robot
     * 
     * @param current_angles 
     *    current encoders' angles
     * @returns 
     *    a new positional odometry data
     */
    geometry_msgs::msg::Pose update_odom(const velmwheel_msgs::msg::Wheels &current_angles);

private: /* ------------------------------------------------ Auxiliary types ------------------------------------------------------ */

    /**
     * @brief Data structure representing set of informations that needs to be
     *    kept by the controll task to calculate Velmwheel's odometry
     */
    typedef struct {

        // Previous position calculated by the odometry
        geometry_msgs::msg::Point previous_position{ rosidl_runtime_cpp::MessageInitialization::ZERO };
        // Previous orientation calculated by the odometry given as a Z angle
        double previous_z_angle { 0.0 };
        // Previous angles read from encoders
        velmwheel_msgs::msg::Wheels previous_angles{ rosidl_runtime_cpp::MessageInitialization::ZERO };

    } OdomKeepup;
    
private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

	/// Subscriber interface used to acquire current encoders measurements from the robot
	rclcpp::Subscription<velmwheel_msgs::msg::EncodersStamped>::SharedPtr encoders_sub;
	/// Subscriber interface used to acquire current velocity sepoint for the robot
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_setpoint_sub;
    /// Subscriber interface used to set new odometry position.
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr encoders_set_pose_sub;

	/// Publisher interface used to provide controls for robot's wheels
	rclcpp::Publisher<velmwheel_msgs::msg::Wheels>::SharedPtr controls_pub;
	/// Publisher interface used to broadcast current velocity of the robot
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
	/// Publisher interface used to broadcast basic odometry data
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
	/// Publisher interface used to broadcast basic odometry data as a pose
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub;

    /// TF2 publishing object for broadcastign odom => base_link relationship
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /// Last velocity setpoint set for the robot
    geometry_msgs::msg::Twist current_velocity_setpoint;    
    
    /// Odometry message (reused between iterations to save on coopying the covariance matrix)
    nav_msgs::msg::Odometry odom_msg;
    /// Odometry transformation (reused to save time on filling header data)
    geometry_msgs::msg::TransformStamped odom_transform;
    /// Basic informations needed to be kept between subsequent iterations of odometry calculation (empty until first measurements arrive)
    std::optional<OdomKeepup> odom_keepup;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
