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
