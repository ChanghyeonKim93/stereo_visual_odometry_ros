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

#ifndef CORE_ORB_EXTRACTOR_H_
#define CORE_ORB_EXTRACTOR_H_

#include <utility>
#include <vector>

#include "core/orb_descriptor_pattern.h"
#include "core/types.h"
#include "opencv4/opencv2/core.hpp"

namespace visual_odometry {

class OrbExtractor {
 public:
  OrbExtractor();

  std::vector<cv::Mat> ComputeImagePyramid(const cv::Mat& image,
                                           const int num_scale_levels,
                                           const double scale_factor);

  std::vector<Feature> ExtractAndCompute(
      const std::vector<cv::Mat>& image_pyramid, const int num_max_features,
      const int num_scale_levels, const double scale_factor,
      const int fast_threshold_high, const int fast_threshold_low);

 private:
  std::vector<int> PrecomputeUmaxList();
  std::vector<int> ComputeNumFeaturesPerLevel(const int num_max_features,
                                              const int num_scale_levels,
                                              const double scale_factor);
  std::vector<cv::KeyPoint> DistributeFeaturesByOctTree(
      const std::vector<cv::KeyPoint>& keypoints_to_distribute, const int minX,
      const int maxX, const int minY, const int maxY,
      const int target_num_features);
  std::vector<Feature> ExtractFastFeatures(const cv::Mat& image_pyramid,
                                           const int num_features,
                                           const int level,
                                           const double scale_factor,
                                           const int fast_threshold_high,
                                           const int fast_threshold_low);
  std::vector<float> CalculateFeatureAngle(
      const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);

 private:
  std::vector<int> angle_patch_u_max_list_;
  std::vector<std::pair<Pixel, Pixel>> orb_descriptor_pattern_;
};

}  // namespace visual_odometry

#endif  // CORE_ORB_EXTRACTOR_H_
