#include <memory>
#include <string>

#include "core/stereo_visual_odometry.h"

#include "rclcpp/rclcpp.hpp"

#include "ros2/wrapper/stereo_visual_odometry_ros2.h"

StereoVisualOdometryRos2::StereoVisualOdometryRos2(const std::string& node_name)
    : Node(node_name) {
  stereo_vo_ = std::make_unique<StereoVisualOdometry>();

  // Subscribers
  subscriber_left_image_.subscribe(this, topic_names.left_image);
  subscriber_right_image_.subscribe(this, topic_names.right_image);
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
  (void)msg_left;
  (void)msg_right;
}