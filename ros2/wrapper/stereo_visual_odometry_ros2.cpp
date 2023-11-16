/**
 * This file is part of Stereo Visual Odometry.
 *
 * Copyright (C) 2023-2023 Changhyeon Kim, hyun91015@gmail.com
 * (ChanghyeonKim93@github.com)
 *
 * Stereo Visual Odometry is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Stereo Visual Odometry is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Stereo Visual Odometry. If not, see <http://www.gnu.org/licenses/>.
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
