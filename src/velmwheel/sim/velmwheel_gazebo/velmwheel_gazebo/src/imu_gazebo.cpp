/* ============================================================================================================================ *//**
 * @file       imu_gazebo.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 7th March 2022 5:15:43 pm
 * @modified   Monday, 30th May 2022 11:38:57 pm
 * @project    engineering-thesis
 * @brief      Implementation of the IMU sensor's Gazebo plugin for WUT Velmobil robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include "velmwheel/gazebo/imu_gazebo.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo {

/* ============================================================= Nodes ============================================================ */

void Imu::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) {
    
    /* ------------------------- Initialize ROS interface ------------------------ */
    
    // Create ROS node (Create the node via the Gazebo interface to automatically handle namespace, remaps, etc...)
    node = gazebo_ros::Node::Get(sdf);

    // Create ROS publisher interface to braodcast IMU data on
    *node_common::communication::make_publisher_builder(pub)
        .node(*node)
        .name(OUT_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    /* ------------------------ Initialize Gazebo interface ---------------------- */

    // Get pointer to the sensor plugin from the upstream
    if(sensor = std::dynamic_pointer_cast<sensors::ImuSensor>(parent); !sensor)
        RCLCPP_FATAL_STREAM(node->get_logger(), "[imu_gazebo] Failed to soruce reference to the Gazebo sensor plugin");

    // Connect on-update action to the plugin
    update_connection = sensor->ConnectUpdated(std::bind(&Imu::OnUpdate, this));
    // Activate the sensor
    sensor->SetActive(true);

    /* --------------------------------------------------------------------------- */

    // Log info message
    RCLCPP_INFO(node->get_logger(), "Running 'imu_gazebo' plugin...");
}

void Imu::OnUpdate() {
    
    sensor_msgs::msg::Imu msg;
    
    // Fill the message's header
    msg.header.stamp.sec     = sensor->LastMeasurementTime().sec;
    msg.header.stamp.nanosec = sensor->LastMeasurementTime().nsec;
    msg.header.frame_id      = BASE_TF_FRAME;
    // Fill message's body
    msg.orientation.x         = sensor->Orientation().X();
    msg.orientation.y         = sensor->Orientation().Y();
    msg.orientation.z         = sensor->Orientation().Z();
    msg.orientation.w         = sensor->Orientation().W();
    msg.angular_velocity.x    = sensor->AngularVelocity().X();
    msg.angular_velocity.y    = sensor->AngularVelocity().Y();
    msg.angular_velocity.z    = sensor->AngularVelocity().Z();
    msg.linear_acceleration.x = sensor->LinearAcceleration().X();
    msg.linear_acceleration.y = sensor->LinearAcceleration().Y();
    msg.linear_acceleration.z = sensor->LinearAcceleration().Z();
    
    // Publish the message
    pub->publish(msg);
}

/* ================================================================================================================================ */

// Register class as Gazebo plugin
GZ_REGISTER_SENSOR_PLUGIN(Imu)

/* ================================================================================================================================ */

} // End namespace gazebo
