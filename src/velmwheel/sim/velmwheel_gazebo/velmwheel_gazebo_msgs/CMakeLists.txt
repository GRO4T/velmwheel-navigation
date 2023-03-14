# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Tuesday, 14th March 2023 7:02:04 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_gazebo_msgs` package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_gazebo_msgs)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
# Interfaces dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================================================== Interfaces =========================================================== #

# Add interfaces to the project
rosidl_generate_interfaces(${PROJECT_NAME}

    # Messages
    "msg/ContactState.msg"

    # Services
    "srv/FrictionConfig.srv"
    "srv/InertiaConfig.srv"
    "srv/VelocityConfig.srv"

    # Dependencies
    DEPENDENCIES std_msgs geometry_msgs

)

# Export ament dependencies
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
