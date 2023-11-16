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

#ifndef VISUAL_ODOMETRY_OPTIMIZATION_POSE_ONLY_OPTIMIZER_H_
#define VISUAL_ODOMETRY_OPTIMIZATION_POSE_ONLY_OPTIMIZER_H_

#include <iostream>

#include "eigen3/Eigen/Dense"

#include "core/optimization/options.h"
#include "core/optimization/summary.h"
#include "types.h"

namespace visual_odometry {
namespace optimization {

class PoseOnlyOptimizer {
 public:
  using Residual1x1 = float;
  using Residual2x1 = Eigen::Matrix<float, 2, 1>;
  using Jacobian2x3 = Eigen::Matrix<float, 2, 3>;
  using Jacobian2x6 = Eigen::Matrix<float, 2, 6>;
  using Gradient3x1 = Eigen::Matrix<float, 3, 1>;
  using Gradient6x1 = Eigen::Matrix<float, 6, 1>;
  using Hessian3x3 = Eigen::Matrix<float, 3, 3>;
  using Hessian6x6 = Eigen::Matrix<float, 6, 6>;

 public:
  virtual ~PoseOnlyOptimizer() {}
  virtual void Reset() = 0;
  virtual void RegisterCamera(const CameraId camera_id,
                              const Camera& camera) = 0;
  virtual bool Solve6DofProblem(Pose* world_to_body_frame_pose,
                                const Options& options,
                                Summary* summary = nullptr) = 0;

 public:
  virtual void AddPointFeatureObservation(
      const CameraId related_camera_id, const Point& query_world_point,
      const Feature& matched_point_feature) = 0;
};

}  // namespace optimization
}  // namespace visual_odometry

#endif  // VISUAL_ODOMETRY_OPTIMIZATION_POSE_ONLY_OPTIMIZER_H_