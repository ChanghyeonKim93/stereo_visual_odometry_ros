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

#include "core/optimization/summary.h"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace visual_odometry {
namespace optimization {

OptimizationSummary::OptimizationSummary() {}

OptimizationSummary::~OptimizationSummary() {}

double OptimizationSummary::GetTotalTimeInSeconds() const {
  return final_summary_.total_time_in_seconds;
}

void OptimizationSummary::SetFinalSummary(const FinalSummary& final_summary) {
  final_summary_ = final_summary;
}

void OptimizationSummary::AddStepSummary(const StepSummary& iteration_summary) {
  step_summary_list_.push_back(iteration_summary);
}

std::string OptimizationSummary::ReportSummary() const {
  const std::streamsize default_precision{std::cout.precision()};
  std::stringstream ss;
  ss << "itr ";
  ss << "  total_cost  ";
  ss << " avg.cost    ";
  ss << " cost_change ";
  ss << " |step|  ";
  ss << " |gradient| ";
  ss << " damp_term ";
  ss << " step_time[s] ";
  ss << "step_stat\n";

  const size_t num_iterations = step_summary_list_.size();
  const auto& numeric_precision = std::setprecision(2);
  for (size_t iteration = 0; iteration < num_iterations; ++iteration) {
    const StepSummary& step_summary = step_summary_list_[iteration];
    ss << std::setw(3) << iteration << " ";
    ss << " " << std::scientific << step_summary.cost;
    ss << "    " << numeric_precision << std::scientific
       << step_summary.average_cost;
    ss << "    " << numeric_precision << std::scientific
       << step_summary.cost_change;
    ss << "   " << numeric_precision << std::scientific
       << step_summary.norm_of_parameter_step;
    ss << "   " << numeric_precision << std::scientific
       << step_summary.norm_of_gradient;
    ss << "   " << numeric_precision << std::scientific
       << step_summary.damping_term;
    ss << "   " << numeric_precision << std::scientific
       << step_summary.step_time_in_seconds;
    switch (step_summary.step_status) {
      case StepStatus::UPDATED:
        ss << "     UPDATE";
        break;
      case StepStatus::NOT_UPDATED:
        ss << "      SKIP ";
        break;
      case StepStatus::UPDATED_WITH_MORE_CONFIDENCE:
        ss << "     UPDATE";
        break;
      default:
        ss << "     ";
    }
    ss << "\n" << std::setprecision(default_precision);
  }

  ss << std::setprecision(5);
  ss << "Optimizer Report:\n";
  ss << "  Iterations      : " << num_iterations << "\n";
  ss << "  Total time      : " << final_summary_.total_time_in_seconds
     << " [s]\n";
  ss << "  Initial cost    : " << step_summary_list_.front().cost << "\n";
  ss << "  Final cost      : " << step_summary_list_.back().cost << "\n";
  ss << "  Initial avg.cost: " << step_summary_list_.front().average_cost
     << "\n";
  ss << "    Final avg.cost: " << step_summary_list_.back().average_cost
     << "\n";
  ss << "  Termination     : "
     << (final_summary_.convergence_status ? "Converged\n" : "Not converged\n");
  if (final_summary_.max_iteration == static_cast<int>(num_iterations))
    ss << " max iteration ! Maybe local minima.\n";
  ss << std::setprecision(default_precision);

  return ss.str();
}

}  // namespace optimization
}  // namespace visual_odometry
