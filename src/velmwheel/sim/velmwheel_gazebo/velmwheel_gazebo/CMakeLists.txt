# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Sunday, 12th March 2023 3:07:39 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_gazebo` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_gazebo)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Prepare common options for all targets
list(APPEND COMMON_OPTIONS 

    # Don't export CMake targets
    SKIP_EXPORT
    # Don't install plugins' headers
    SKIP_INCLUDES_INSTALL
)

# ========================================================== Dependencies ========================================================== #

# Linkable dependencies
list(APPEND DEPENDENCIES
    rclcpp
    rclpy
    node_common_cpp
    gazebo_ros
    geometry_msgs
    sensor_msgs
    tf2_ros
    velmwheel_model
    velmwheel_msgs
    velmwheel_gazebo_msgs
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Dependencies
find_packages(

    # Ament dependency (always present)
    ament_cmake
    ament_cmake_python
    # Find dependencies
    ${DEPENDENCIES}

)

# =========================================================== IMU driver =========================================================== #

# Add IMU simulator target
add_ament_library(

    # Library name
    LIBRARY_NAME imu_gazebo

    # Library sources
    SOURCES
        src/imu_gazebo.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>

    # Dependencies
    AMENT_DEPENDENCIES 
        rclcpp
        node_common_cpp
        sensor_msgs
        gazebo_ros
    
    # Set common options
    ${COMMON_OPTIONS}

)

# ========================================================== LIDAR driver ========================================================== #

# Add LIDAR simulator target
add_ament_library(

    # Library name
    LIBRARY_NAME lidar_gazebo

    # Library sources
    SOURCES
        src/lidar_gazebo.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>

    # Dependencies
    AMENT_DEPENDENCIES 
        rclcpp
        node_common_cpp
        sensor_msgs
        gazebo_ros
        tf2
        tf2_ros
    
    # Set common options
    ${COMMON_OPTIONS}
    
)

# =========================================================== Base driver ========================================================== #

# Set name of the next target
set(BASE_REAL_TARGET base_gazebo.real)

# Add robot's base simulation target (relistic implementation)
add_ament_library(

    # Library name
    LIBRARY_NAME ${BASE_REAL_TARGET}

    # Library sources
    SOURCES
        src/base_gazebo.real.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>

    # Dependencies
    AMENT_DEPENDENCIES 
        rclcpp
        node_common_cpp
        geometry_msgs
        sensor_msgs
        gazebo_ros
        tf2
        tf2_ros
        velmwheel_msgs
        velmwheel_model
    
    # Set common options
    ${COMMON_OPTIONS}
    
)

# Add target-specific definitions
target_compile_definitions(${BASE_REAL_TARGET}
    PRIVATE
        "PLUGIN_TYPE_REAL"
)

# Set name of the next target
set(BASE_IDEAL_TARGET base_gazebo.ideal)

# Add robot's base simulation target (idealistic implementation)
add_ament_library(

    # Library name
    LIBRARY_NAME ${BASE_IDEAL_TARGET}

    # Library sources
    SOURCES
        src/base_gazebo.ideal.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>

    # Dependencies
    AMENT_DEPENDENCIES 
        rclcpp
        node_common_cpp
        geometry_msgs
        sensor_msgs
        gazebo_ros
        tf2
        tf2_ros
        velmwheel_msgs
        velmwheel_model
    
    # Set common options
    ${COMMON_OPTIONS}
    
)

# Add target-specific definitions
target_compile_definitions(${BASE_IDEAL_TARGET}
    PRIVATE
        "PLUGIN_TYPE_IDEAL"
)

# ===================================================== Simulated-robot driver ===================================================== #

# Add robot's auxiliary plugin 
add_ament_library(

    # Library name
    LIBRARY_NAME robot_gazebo

    # Library sources
    SOURCES
        src/robot_gazebo.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>

    # Dependencies
    AMENT_DEPENDENCIES 
        rclcpp
        node_common_cpp
        geometry_msgs
        gazebo_ros
        tf2
        tf2_ros
        velmwheel_model
        velmwheel_gazebo_msgs
    
    # Set common options
    ${COMMON_OPTIONS}
    
)

# ===================================================== Contact plugin ===================================================== #

# Add robot's auxiliary plugin 
add_ament_library(

    # Library name
    LIBRARY_NAME contact_gazebo

    # Library sources
    SOURCES
        src/contact_gazebo.cpp

    # Includes
    OVERRIDE_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>

    # Dependencies
    AMENT_DEPENDENCIES 
        # rclcpp
        # node_common_cpp
        # geometry_msgs
        gazebo_ros
        # tf2
        # tf2_ros
        # velmwheel_model
        # velmwheel_gazebo_msgs
    
    # Set common options
    ${COMMON_OPTIONS}
    
)

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        gazebo
        launch
        urdf
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)

    # Find linter
    find_package(ament_lint_auto REQUIRED)
    # Find test dependencies using info from `package.xml`
    ament_lint_auto_find_test_dependencies()

endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
