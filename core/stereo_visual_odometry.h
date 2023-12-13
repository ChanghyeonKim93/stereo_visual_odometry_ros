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

#include <memory>
#include <string>

#include "core/orb_extractor.h"
#include "core/types.h"

#include "opencv4/opencv2/core.hpp"

namespace visual_odometry {

class StereoVisualOdometry {
 public:
  struct Parameters {
    struct {
      struct {
        int cell_height{10};
        int cell_width{10};
      } index_grid;
      struct {
        int num_scale_levels{8};
        double scale_factor{1.2};
      } image_pyramid;
      struct {
        int num_features{2000};
        int fast_threshold_high{20};
        int fast_threshold_low{7};
        struct {
          int cell_height{20};
          int cell_width{20};
        } feature_selection;
      } extractor;
      struct {
        int max_discriptor_distance{100};
      } matcher;
    } feature;
    struct {
      double threshold_huber{1.0};
    } optimization;
  };

 public:
  StereoVisualOdometry();

  bool LoadIntrinsicsAndExtrinsicsParameters(
      const std::string& intrinsic_file_path,
      const std::string& extrinsic_file_path);
  bool LoadUserParameters(const std::string& file_path,
                          StereoVisualOdometry::Parameters* parameters);

  bool TrackStereoImages(const double timestamp, const cv::Mat& left_image,
                         const cv::Mat& right_image);

 private:
  Parameters parameters_;

  std::shared_ptr<Camera> left_cam_{nullptr};
  std::shared_ptr<Camera> right_cam_{nullptr};

 private:
  std::unique_ptr<OrbExtractor> orb_extractor_{nullptr};
};

}  // namespace visual_odometry

#endif  // CORE_STEREO_VISUAL_ODOMETRY_H_
