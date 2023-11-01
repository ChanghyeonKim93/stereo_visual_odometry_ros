/*
  Copyright 2023 Changhyeon Kim
  e-mail: hyun91015@gmail.com
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
