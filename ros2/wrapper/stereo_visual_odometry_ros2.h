#ifndef ROS2_STEREO_VISUAL_ODOMETRY_STEREO_VISUAL_ODOMETRY_ROS2_H_
#define ROS2_STEREO_VISUAL_ODOMETRY_STEREO_VISUAL_ODOMETRY_ROS2_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "core/stereo_visual_odometry.h"

using ImageMsg = sensor_msgs::msg::Image;

class StereoVisualOdometryRos2 : public rclcpp::Node {
 public:
  StereoVisualOdometryRos2(const std::string& node_name);
  ~StereoVisualOdometryRos2();

 private:
  bool LoadConfigurationFiles();
  void CallbackMessagesForStereoImages(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_left,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_right);

 private:
  struct {
    std::string left_image;
    std::string right_image;
  } topic_names;

  std::unique_ptr<StereoVisualOdometry> stereo_vo_;

  message_filters::Subscriber<ImageMsg> subscriber_left_image_;
  message_filters::Subscriber<ImageMsg> subscriber_right_image_;
  std::shared_ptr<message_filters::TimeSynchronizer<ImageMsg, ImageMsg>>
      stereo_synchronizer_;
};

#endif