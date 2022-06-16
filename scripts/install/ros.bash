# ====================================================================================================================================
# @file       ros.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 2nd January 2021 2:53:45 pm
# @modified   Thursday, 16th June 2022 1:01:33 pm
# @project    engineering-thesis
# @brief      Setups environment and installs ROS2 distribution along with recommended tools
# 
# 
# @note Script was tested on Ubuntu 22.0.4 LTS system
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ========================================================== Configuration ========================================================= #

# Set distro
export ROS2_DISTRO="$PROJECT_ROS_DISTRO"

# ============================================================== Main ============================================================== #

main() {

    # Install ros
    if ! is_pkg_installed "ros-${PROJECT_ROS_DISTRO}-desktop"; then
        $BASH_UTILS_BIN_HOME/install/ros/ros.bash --distro=$PROJECT_ROS_DISTRO install pkg
    # Else, update ros package
    else
        install_pkg  --su -yU "ros-${PROJECT_ROS_DISTRO}-desktop"
    fi

    # Install Gazebo package
    if ! is_pkg_installed 'gazebo11'; then
        $BASH_UTILS_BIN_HOME/install/utilities/gazebo.bash
    fi

    # ROS dependencies
    local -a ros_dependencies=(
        
        # Velmwheel utilities
        xterm

        # External libraries
        libpcl-dev
        librange-v3-dev

        # ROS packages
        ros-${ROS2_DISTRO}-angles
        ros-${ROS2_DISTRO}-gazebo-ros-pkgs
        ros-${ROS2_DISTRO}-joint-state-publisher
        ros-${ROS2_DISTRO}-xacro
        ros-${ROS2_DISTRO}-slam-toolbox
        ros-${ROS2_DISTRO}-pointcloud-to-laserscan
        ros-${ROS2_DISTRO}-robot-localization
        ros-${ROS2_DISTRO}-pcl-ros
        ros-${ROS2_DISTRO}-navigation2
        ros-${ROS2_DISTRO}-nav2-bringup

        # Auxiliary ROS packages (mainly for testing)
        ros-${ROS2_DISTRO}-turtlebot3
        ros-${ROS2_DISTRO}-turtlebot3-bringup
        ros-${ROS2_DISTRO}-turtlebot3-cartographer
        ros-${ROS2_DISTRO}-turtlebot3-description
        ros-${ROS2_DISTRO}-turtlebot3-example
        ros-${ROS2_DISTRO}-turtlebot3-fake-node
        ros-${ROS2_DISTRO}-turtlebot3-fake-node-dbgsym
        ros-${ROS2_DISTRO}-turtlebot3-gazebo
        ros-${ROS2_DISTRO}-turtlebot3-gazebo-dbgsym
        ros-${ROS2_DISTRO}-turtlebot3-msgs
        ros-${ROS2_DISTRO}-turtlebot3-msgs-dbgsym
        ros-${ROS2_DISTRO}-turtlebot3-navigation2
        ros-${ROS2_DISTRO}-turtlebot3-node
        ros-${ROS2_DISTRO}-turtlebot3-node-dbgsym
        ros-${ROS2_DISTRO}-turtlebot3-simulations
        ros-${ROS2_DISTRO}-turtlebot3-teleop

    )

    # Install ROS dependencies
    install_pkg_list -f --su -yv ros_dependencies
    # Update dependencies
    rosdep install                                 \
        --rosdistro=${ROS2_DISTRO}                 \
        --from-paths /opt/ros/${ROS2_DISTRO}/share \
        --ignore-src -y
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
