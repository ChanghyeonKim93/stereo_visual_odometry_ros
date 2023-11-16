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

#ifndef ROS2_WRAPPER_STEREO_VISUAL_ODOMETRY_ROS2_H_
#define ROS2_WRAPPER_STEREO_VISUAL_ODOMETRY_ROS2_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "core/stereo_visual_odometry.h"

using ImageMsg = sensor_msgs::msg::Image;

namespace visual_odometry {

class StereoVisualOdometryRos2 : public rclcpp::Node {
 public:
  explicit StereoVisualOdometryRos2(const std::string& node_name);
  ~StereoVisualOdometryRos2();

 private:
  bool LoadConfigurationFiles();
  void CallbackMessagesForStereoImages(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_left,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_right);

 private:
  struct {
    struct {
      std::string left_image{""};
      std::string right_image{""};
    } subscribe;
    struct {
      std::string pose{""};
      std::string trajectory{""};
    } publish;
  } topic_names;

  std::unique_ptr<StereoVisualOdometry> stereo_visual_odometry_;

  message_filters::Subscriber<ImageMsg> subscriber_left_image_;
  message_filters::Subscriber<ImageMsg> subscriber_right_image_;
  std::shared_ptr<message_filters::TimeSynchronizer<ImageMsg, ImageMsg>>
      stereo_synchronizer_;
};

}  // namespace visual_odometry

#endif  // ROS2_WRAPPER_STEREO_VISUAL_ODOMETRY_ROS2_H_
