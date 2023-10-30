cmake_minimum_required(VERSION 3.10)
project(stereo_visual_odometry)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_subdirectory(core)
add_subdirectory(ros2)

ament_package()