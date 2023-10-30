#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2/wrapper/stereo_visual_odometry_ros2.h"

StereoVisualOdometryRos2::StereoVisualOdometryRos2(const std::string& node_name) : Node(node_name) {}

StereoVisualOdometryRos2::~StereoVisualOdometryRos2() {}