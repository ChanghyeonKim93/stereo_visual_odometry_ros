/*
  Copyright 2023 Changhyeon Kim
  e-mail: hyun91015@gmail.com
*/

#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2/wrapper/stereo_visual_odometry_ros2.h"

#define NODE_NAME "stereo_visual_odometry_node"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<visual_odometry::StereoVisualOdometryRos2>(NODE_NAME));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
