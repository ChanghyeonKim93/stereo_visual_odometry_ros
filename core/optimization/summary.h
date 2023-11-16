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

#ifndef CORE_OPTIMIZATION_SUMMARY_H_
#define CORE_OPTIMIZATION_SUMMARY_H_

#include <string>
#include <vector>

namespace visual_odometry {
namespace optimization {

enum class StepStatus {
  UPDATED = 0,
  UPDATED_WITH_MORE_CONFIDENCE = 1,
  NOT_UPDATED = 2
};

struct StepSummary {
  int iteration{0};
  double cost{0.0};
  double cost_change{0.0};
  double average_cost{0.0};
  double norm_of_gradient{0.0};
  double norm_of_parameter_step{0.0};
  double damping_term{0.0};
  double step_time_in_seconds{0.0};
  double cumulative_time_in_seconds{0.0};
  StepStatus step_status{StepStatus::NOT_UPDATED};
};

struct FinalSummary {
  int max_iteration{0};
  double total_time_in_seconds{0.0};
  double threshold_norm_of_parameter_step{0.0};
  double threshold_cost_change{0.0};
  bool convergence_status{false};
};

class OptimizationSummary {
 public:
  OptimizationSummary();
  ~OptimizationSummary();

 public:
  std::string ReportSummary() const;
  double GetTotalTimeInSeconds() const;

 public:
  void AddStepSummary(const StepSummary& step_summary);
  void SetFinalSummary(const FinalSummary& final_summary);

 private:
  std::vector<StepSummary> step_summary_list_;
  FinalSummary final_summary_;
};

}  // namespace optimization
}  // namespace visual_odometry

#endif  // CORE_OPTIMIZATION_SUMMARY_H_
