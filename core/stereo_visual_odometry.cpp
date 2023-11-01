/*
Copyright 2023 Changhyeon Kim
*/

#include "core/stereo_visual_odometry.h"

namespace visual_odometry {

StereoVisualOdometry::StereoVisualOdometry() {}

bool StereoVisualOdometry::TrackStereoImages(const double timestamp,
                                             const cv::Mat& left_image,
                                             const cv::Mat& right_image) {
  bool is_track_succeeded = true;
  (void)timestamp;
  (void)left_image;
  (void)right_image;

  return is_track_succeeded;
}

}  // namespace visual_odometry
