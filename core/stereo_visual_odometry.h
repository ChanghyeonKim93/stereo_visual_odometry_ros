/*
  Copyright 2023 Changhyeon Kim
  e-mail: hyun91015@gmail.com
*/

#ifndef CORE_STEREO_VISUAL_ODOMETRY_H_
#define CORE_STEREO_VISUAL_ODOMETRY_H_

#include "opencv4/opencv2/core.hpp"

#include "core/types.h"

namespace visual_odometry {

class StereoVisualOdometry {
 public:
  StereoVisualOdometry();

  bool TrackStereoImages(const double timestamp, const cv::Mat& left_image,
                         const cv::Mat& right_image);
};

}  // namespace visual_odometry

#endif  // CORE_STEREO_VISUAL_ODOMETRY_H_
