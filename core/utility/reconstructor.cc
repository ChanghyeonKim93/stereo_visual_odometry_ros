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

#include <vector>

#include "core/utility/reconstructor.h"

namespace visual_odometry {

Point Reconstructor::ReconstructDLT(const Pixel& pt0, const Pixel& pt1,
                                    const Pose& pose0, const Pose& pose1) {
  Point world_point{Point::Zero()};

  return world_point;
}

std::vector<Point> Reconstructor::ReconstructDLT(const std::vector<Pixel>& pts0,
                                                 const std::vector<Pixel>& pts1,
                                                 const Pose& pose0,
                                                 const Pose& pose1) {
  if (pts0.size() != pts1.size())
    throw std::runtime_error("pts0.size() != pts1.size()");

  const size_t n_pts = pts0.size();
  std::vector<Point> world_points(n_pts);
  for (size_t index = 0; index < n_pts; ++index) {
    // TODO(ChanghyeonKim): implement DLT
    world_points[index].setZero();
  }

  return world_points;
}

}  // namespace visual_odometry
