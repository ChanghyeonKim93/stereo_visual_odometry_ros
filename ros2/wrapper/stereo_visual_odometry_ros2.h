#ifndef ROS2_STEREO_VISUAL_ODOMETRY_STEREO_VISUAL_ODOMETRY_ROS2_H_
#define ROS2_STEREO_VISUAL_ODOMETRY_STEREO_VISUAL_ODOMETRY_ROS2_H_

#include "rclcpp/rclcpp.hpp"
class StereoVisualOdometryRos2 : public rclcpp::Node {
 public:
  StereoVisualOdometryRos2(const std::string& node_name);
  ~StereoVisualOdometryRos2();
};

#endif