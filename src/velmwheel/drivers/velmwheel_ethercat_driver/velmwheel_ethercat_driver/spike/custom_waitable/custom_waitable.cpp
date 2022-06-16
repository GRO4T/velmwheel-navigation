/** ==================================================================================================================================
 * @file       custom_waitable.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 2:24:56 pm
 * @modified   Thursday, 16th June 2022 6:43:00 pm
 * @project    engineering-thesis
 * @brief
 *    
 *    Test node implementing example custom Waitable mechanism for ROS2 node that is required to implement velmwheel_ethercat_driver
 *    
 * @soruce https://answers.ros.org/question/322815/ros2-how-to-create-custom-waitable/
 * @see https://docs.ros.org/en/galactic/Concepts/About-Executors.html
 * @see https://github.com/ros2/rclcpp/pull/1612
 * 
 * @copyright Krzysztof Pierczyk © 2022
 * ================================================================================================================================ */

/* =========================================================== Includes =========================================================== */

// System includes
#include <chrono>
#include <mutex>
// ROS includes
#include "rclcpp/rclcpp.hpp"

/* =========================================================== Waitable =========================================================== */

/**
 * @brief Example implementation of custom waitable
 */
class MyWaitable : public rclcpp::Waitable {

public: /* ------------------------ Waitable API iplementation ----------------------- */
    
    /// @brief Initialize waitable
    MyWaitable() { }

    /// @brief Gives API hint about number of associated guard conditions
    size_t get_number_of_ready_guard_conditions() override { return 1;}

    /// @brief Adds waitable to the wait set
    void add_to_wait_set(rcl_wait_set_t * wait_set) override {
        std::cout << "Adding to wait set! " << "(" << std::this_thread::get_id() << ")" << std::endl;
        gc.add_to_wait_set(wait_set);
    }

    /// @brief Tells API whether the waitable is ready to be executed
    bool is_ready([[maybe_unused]] rcl_wait_set_t * wait_set) override { 
        std::cout << "Requesting readiness! " << "(" << std::this_thread::get_id() << ")" << std::endl;
        return size > 0; 
    }

    /// @brief Takes data related to the waited conditon
    std::shared_ptr<void> take_data() override { 
        std::cout << "Taking data! " << "(" << std::this_thread::get_id() << ")" << std::endl;
        return nullptr; 
    }

    /// @brief Actual routine executed by the waitable
    void execute([[maybe_unused]] std::shared_ptr<void> & data) override {
        std::cout << "[---------] Executing waitable function! " << "(" << std::this_thread::get_id() << ")" << std::endl;
        size --;
    }

public: /* -------------------------------- Helper API ------------------------------- */

    /// @brief Triggers the guard condition
    void trigger() { gc.trigger(); }

    /// @brief Adds element to the fake queue
    void produce() { size++; }

private: /* ----------------------------- Data members ------------------------------- */

    // Guarding condition
    rclcpp::GuardCondition gc;
    // Current size of a fake queue
    int size = 0;

};

/* ============================================================= Node ============================================================= */

/**
 * @brief Example node utilizing custom waitable
 */
class MyNode : public rclcpp::Node {
public:

    /// @brief Initialize node with the custom waitable
    MyNode() : Node("my_node") {

        // Get waitable interface
        auto node_waitables_interface = this->get_node_waitables_interface();
        // Create waitable implementation
        waitable_ptr = std::make_shared<MyWaitable>();
        // Add waitable to the onde's interface
        node_waitables_interface->add_waitable(waitable_ptr, nullptr);

        // Add some periodic job
        timer = this->create_wall_timer(std::chrono::milliseconds(50), 
            [](){ std::cout << "  -> timer" << "(" << std::this_thread::get_id() << ")" << std::endl; }
        );
        
    }

    /// Returns handle to the waitable
    std::shared_ptr<MyWaitable> get_waitable() { return waitable_ptr; }

private:

    /// Periodic timer
    std::shared_ptr<rclcpp::TimerBase> timer;
    /// Custom waitable
    std::shared_ptr<MyWaitable> waitable_ptr;

};

/* ============================================================= Main ============================================================= */

int main(int argc, char * argv[]) {

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create a node with a custom waitable
    std::shared_ptr<MyNode> node = std::make_shared<MyNode>();
    // Let it spin in a separate thread
    std::thread t([=](){
        std::cout << "before spinning " << "(" << std::this_thread::get_id() << ")" << std::endl;   
        rclcpp::spin(node);
    });
    // Detach the spinning thread
    t.detach();

    // After some time, simulate that a new item is ready to be consumed and notify the guard condition
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto waitable = node->get_waitable();
    waitable->produce();
    waitable->trigger();

    // Wait some time to let the node process waitable
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Deinitialize ROS
    rclcpp::shutdown();
    
    return 0;
}

/* ================================================================================================================================ */
