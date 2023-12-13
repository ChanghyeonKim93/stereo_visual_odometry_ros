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

int encoding2mat_type(const std::string& encoding) {
  if (encoding == "mono8")
    return CV_8UC1;
  else if (encoding == "bgr8")
    return CV_8UC3;
  else if (encoding == "mono16")
    return CV_16SC1;
  else if (encoding == "rgba8")
    return CV_8UC4;
  else if (encoding == "bgra8")
    return CV_8UC4;
  else if (encoding == "32FC1")
    return CV_32FC1;
  else if (encoding == "rgb8")
    return CV_8UC3;
  else
    throw std::runtime_error("Unsupported encoding type");
}

std::string mat_type2encoding(int mat_type) {
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

StereoVisualOdometryRos2::StereoVisualOdometryRos2(const std::string& node_name)
    : Node(node_name) {
  stereo_vo_ = std::make_unique<StereoVisualOdometry>();

  // Subscribers
  subscriber_left_image_.subscribe(
      this, topic_names.subscribe.left_image,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
  subscriber_right_image_.subscribe(
      this, topic_names.subscribe.right_image,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
  stereo_synchronizer_ =
      std::make_shared<message_filters::TimeSynchronizer<ImageMsg, ImageMsg>>(
          subscriber_left_image_, subscriber_right_image_, 10);
  stereo_synchronizer_->registerCallback(
      std::bind(&StereoVisualOdometryRos2::CallbackMessagesForStereoImages,
                this, std::placeholders::_1, std::placeholders::_2));
}

StereoVisualOdometryRos2::~StereoVisualOdometryRos2() {}

void StereoVisualOdometryRos2::CallbackMessagesForStereoImages(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg_left,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg_right) {
  const double current_timestamp =
      static_cast<double>(msg_left->header.stamp.sec) +
      static_cast<double>(msg_left->header.stamp.nanosec) * 1e-9;

  cv::Mat left_image(
      msg_left->height, msg_left->width, encoding2mat_type(msg_left->encoding),
      const_cast<unsigned char*>(msg_left->data.data()), msg_left->step);
  cv::Mat right_image(msg_right->height, msg_right->width,
                      encoding2mat_type(msg_right->encoding),
                      const_cast<unsigned char*>(msg_right->data.data()),
                      msg_right->step);

  stereo_vo_->TrackStereoImages(current_timestamp, left_image, right_image);
}

}  // namespace visual_odometry
