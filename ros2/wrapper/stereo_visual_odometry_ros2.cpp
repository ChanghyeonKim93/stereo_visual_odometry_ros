/*
Copyright 2023 Changhyeon Kim
*/

#include <iostream>
#include <memory>
#include <string>

#include "core/stereo_visual_odometry.h"
#include "ros2/wrapper/stereo_visual_odometry_ros2.h"

#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"

namespace visual_odometry {

StereoVisualOdometryRos2::StereoVisualOdometryRos2(const std::string& node_name)
    : Node(node_name) {
  std::cerr << "Start\n";
  if (!LoadConfigurationFiles())
    throw std::runtime_error("Load configuration is failed.");

  stereo_visual_odometry_ = std::make_unique<StereoVisualOdometry>();

  // Subscribers
  subscriber_left_image_.subscribe(this, topic_names.subscribe.left_image);
  subscriber_right_image_.subscribe(this, topic_names.subscribe.right_image);
  stereo_synchronizer_ =
      std::make_shared<message_filters::TimeSynchronizer<ImageMsg, ImageMsg>>(
          subscriber_left_image_, subscriber_right_image_, 10);
  stereo_synchronizer_->registerCallback(
      std::bind(&StereoVisualOdometryRos2::CallbackMessagesForStereoImages,
                this, std::placeholders::_1, std::placeholders::_2));
}

StereoVisualOdometryRos2::~StereoVisualOdometryRos2() {}

bool StereoVisualOdometryRos2::LoadConfigurationFiles() {
  YAML::Node config = YAML::LoadFile(
      "/home/kch/ros2_ws/src/stereo_visual_odometry_ros/config/"
      "user_parameter2.yaml");

  if (config["feature_extractor.af"]) {
    std::cerr << "Last logged in: " << config["feature_extractor.af"].as<int>()
              << "\n";
  } else {
    return false;
  }

  return true;
}

void StereoVisualOdometryRos2::CallbackMessagesForStereoImages(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg_left,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg_right) {
  (void)msg_left;
  (void)msg_right;
}

}  // namespace visual_odometry
