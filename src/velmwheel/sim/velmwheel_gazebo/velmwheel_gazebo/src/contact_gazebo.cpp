/* ============================================================================================================================ *//**
 * @file       contact_gazebo.cpp
 * @author     Damian Kolaska (damian.kolaska99@gmail.com)
 * @maintainer Damian Kolaska (damian.kolaska99@gmail.com)
 * @date       Sunday, 12th March 2023 3:00:51 pm
 * @modified   Tuesday, 14th March 2023 8:40:58 pm
 * @project    velmwheel
 * @brief      TODO(damiankolaska) - add brief description
 * 
 * 
 * @copyright Damian Kolaska © 2023
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include "velmwheel/gazebo/contact_gazebo.hpp"
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace gazebo {

void ContactPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    /* ------------------------- Initialize ROS interface ------------------------ */
    
    // Create ROS node (Create the node via the Gazebo interface to automatically handle namespace, remaps, etc...)
    node = gazebo_ros::Node::Get(sdf);

    *node_common::communication::make_publisher_builder(pub)
        .node(*node)
        .name(OUT_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    this->sensor =
        std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (!this->sensor)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "ContactPlugin requires a sensor.");
        return;
    }

    // Connect to the sensor update event.
    this->sensor_update_connection = this->sensor->ConnectUpdated(
        std::bind(&ContactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->sensor->SetActive(true);

    RCLCPP_INFO_STREAM(node->get_logger(), "Running 'contact_gazebo' plugin...");
}

void ContactPlugin::OnUpdate() {
  msgs::Contacts contacts = this->sensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
    const auto& contact = contacts.contact(i);

    RCLCPP_DEBUG_STREAM(
        node->get_logger(),
        "Collision between[" << contact.collision1()
            << "] and [" << contact.collision2() << "]\n"
    );

    for (unsigned int j = 0; j < contact.position_size(); ++j)
    {
        RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            j << "  Position:"
                << contact.position(j).x() << " "
                << contact.position(j).y() << " "
                << contact.position(j).z() << "\n"
        );
        RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            "   Normal:"
                << contact.normal(j).x() << " "
                << contact.normal(j).y() << " "
                << contact.normal(j).z() << "\n";
        );
        RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            "   Depth:" << contact.depth(j) << "\n"
        );
    }

    velmwheel_gazebo_msgs::msg::ContactState msg;
    msg.info = "Velmwheel robot collision";
    msg.collision1_name = contact.collision1();
    msg.collision2_name = contact.collision2();
    pub->publish(msg);
  }
}

/* ================================================================================================================================ */

// Register class as Gazebo plugin
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/* ================================================================================================================================ */

} // End namespace gazebo