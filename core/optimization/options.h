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

#ifndef CORE_OPTIMIZATION_OPTIONS_H_
#define CORE_OPTIMIZATION_OPTIONS_H_

namespace visual_odometry {
namespace optimization {

enum class SolverStepType {
  GRADIENT_DESCENT = 0,
  GAUSS_NEWTON = 1,
  LEVENBERG_MARQUARDT = 2
};

struct Options {
  SolverStepType solver_step_type{SolverStepType::GAUSS_NEWTON};
  struct {
    float threshold_parameter_step{1e-9f};
    float threshold_cost_change{1e-9f};
  } convergence_handle;
  struct {
    float threshold_huber_loss{1.0f};
    float threshold_outlier_rejection{2.0f};
  } outlier_handle;
  struct {
    int max_num_iterations{100};
  } iteration_handle;
  struct {
    float initial_lambda{1e-3f};
    float lambda_decreasing_ratio{0.3f};
    float lambda_increasing_ratio{3.0f};
  } trust_region_handle;
};

}  // namespace optimization
}  // namespace visual_odometry

#endif  // CORE_OPTIMIZATION_OPTIONS_H_
