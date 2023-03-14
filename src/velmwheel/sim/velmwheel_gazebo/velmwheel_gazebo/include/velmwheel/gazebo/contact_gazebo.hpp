/* ============================================================================================================================ *//**
 * @file       contact_gazebo.hpp
 * @author     Damian Kolaska (damian.kolaska99@gmail.com)
 * @maintainer Damian Kolaska (damian.kolaska99@gmail.com)
 * @date       Sunday, 12th March 2023 11:02:23 am
 * @modified   Tuesday, 14th March 2023 8:28:23 pm
 * @project    velmwheel
 * @brief      
 * 
 * 
 * @copyright Damian Kolaska © 2023
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GAZEBO_CONTACT_PLUGIN_H__
#define __VELMWHEEL_GAZEBO_CONTACT_PLUGIN_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
// ROS includes
#include <rclcpp/rclcpp.hpp>
// Private includes
#include "velmwheel_gazebo_msgs/msg/contact_state.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo
{

/* ============================================================= Nodes ============================================================ */

/**
 * @brief TODO(damiankolaska): add docstring
 */
class ContactPlugin : public SensorPlugin {

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    static constexpr auto OUT_TOPIC_NAME = "contacts";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new Plugin object
     */
    ContactPlugin() = default;

private: /* -------------------------------------------- Gazebo-specific methods -------------------------------------------------- */

	/**
	 * @brief On-load Gazebo initialization routine of the plugin
     * @param parent 
     *    pointer to the Gazebo Sensor model (injected by the Gazebo)
     * @param sdf
     *    pointer to the SDF <plugin> element of the sensor's description (injected by the Gazebo)
	 */
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

	/**
	 * @brief On simulation update transfers IMU measurements to the ROS topic
	 */
    void OnUpdate();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// ROS node
    gazebo_ros::Node::SharedPtr node;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

	/// Reference to the Gazebo contact sensor
    sensors::ContactSensorPtr sensor;
    /// TODO(damiankolaska) add docstring
    rclcpp::Publisher<velmwheel_gazebo_msgs::msg::ContactState>::SharedPtr pub;

    /**
     * @brief Connection that maintains a link between the contact sensor's
     * updated signal and the OnUpdate callback.
    */
    event::ConnectionPtr sensor_update_connection;

  };

/* ================================================================================================================================ */

} // End namespace gazebo

#endif