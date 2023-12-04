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

#include "core/orb_extractor.h"

#include <algorithm>
#include <list>
#include <vector>

#include "core/types.h"
#include "eigen3/Eigen/Dense"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/imgproc.hpp"

namespace visual_odometry {

class OrbExtractorNode {
 public:
  OrbExtractorNode() : flag_no_more(false) {}

  void DivideNode(OrbExtractorNode* n1, OrbExtractorNode* n2,
                  OrbExtractorNode* n3, OrbExtractorNode* n4);

  std::vector<cv::KeyPoint> keypoints;
  cv::Point2i UL, UR, BL, BR;
  std::list<OrbExtractorNode>::iterator node_list_iterator;
  bool flag_no_more;
};

void OrbExtractorNode::DivideNode(OrbExtractorNode* n1, OrbExtractorNode* n2,
                                  OrbExtractorNode* n3, OrbExtractorNode* n4) {
  const int halfX = std::ceil(static_cast<double>(UR.x - UL.x) * 0.5);
  const int halfY = std::ceil(static_cast<double>(BR.y - UL.y) * 0.5);

  // Define boundaries of childs
  n1->UL = UL;
  n1->UR = cv::Point2i(UL.x + halfX, UL.y);
  n1->BL = cv::Point2i(UL.x, UL.y + halfY);
  n1->BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
  n1->keypoints.reserve(keypoints.size());

  n2->UL = n1->UR;
  n2->UR = UR;
  n2->BL = n1->BR;
  n2->BR = cv::Point2i(UR.x, UL.y + halfY);
  n2->keypoints.reserve(keypoints.size());

  n3->UL = n1->BL;
  n3->UR = n1->BR;
  n3->BL = BL;
  n3->BR = cv::Point2i(n1->BR.x, BL.y);
  n3->keypoints.reserve(keypoints.size());

  n4->UL = n3->UR;
  n4->UR = n2->BR;
  n4->BL = n3->BR;
  n4->BR = BR;
  n4->keypoints.reserve(keypoints.size());

  // Associate points to childs
  for (size_t index = 0; index < keypoints.size(); index++) {
    const auto& kp = keypoints[index];
    if (kp.pt.x < n1->UR.x) {
      if (kp.pt.y < n1->BR.y)
        n1->keypoints.push_back(kp);
      else
        n3->keypoints.push_back(kp);
    } else if (kp.pt.y < n1->BR.y) {
      n2->keypoints.push_back(kp);
    } else {
      n4->keypoints.push_back(kp);
    }
  }

  if (n1->keypoints.size() == 1) n1->flag_no_more = true;
  if (n2->keypoints.size() == 1) n2->flag_no_more = true;
  if (n3->keypoints.size() == 1) n3->flag_no_more = true;
  if (n4->keypoints.size() == 1) n4->flag_no_more = true;
}

static bool compare_nodes(const std::pair<int, OrbExtractorNode*>& e1,
                          const std::pair<int, OrbExtractorNode*>& e2) {
  if (e1.first < e2.first) {
    return true;
  } else if (e1.first > e2.first) {
    return false;
  } else {
    if (e1.second->UL.x < e2.second->UL.x)
      return true;
    else
      return false;
  }
}

std::vector<cv::KeyPoint> OrbExtractor::DistributeOctTree(
    const std::vector<cv::KeyPoint>& keypoints_to_distribute, const int minX,
    const int maxX, const int minY, const int maxY, const int N) {
  // Compute how many initial nodes
  const int num_initial_nodes =
      std::round(static_cast<double>(maxX - minX) / (maxY - minY));
  const double initial_node_width =
      static_cast<double>(maxX - minX) / num_initial_nodes;
  const double inverse_initial_node_width = 1.0 / initial_node_width;

  std::list<OrbExtractorNode> node_list;
  std::vector<OrbExtractorNode*> initial_node_ptr_list;
  initial_node_ptr_list.resize(num_initial_nodes);
  for (int i = 0; i < num_initial_nodes; i++) {
    OrbExtractorNode ni;
    ni.UL = cv::Point2i(initial_node_width * (i), 0);
    ni.UR = cv::Point2i(initial_node_width * (i + 1), 0);
    ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
    ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
    ni.keypoints.reserve(keypoints_to_distribute.size());

    node_list.push_back(ni);
    initial_node_ptr_list[i] = &node_list.back();
  }

  // Associate points to childs
  for (size_t index = 0; index < keypoints_to_distribute.size(); index++) {
    const auto& kpts = keypoints_to_distribute[index];
    initial_node_ptr_list[kpts.pt.x * inverse_initial_node_width]
        ->keypoints.push_back(kpts);
  }

  std::list<OrbExtractorNode>::iterator node_list_itr = node_list.begin();
  while (node_list_itr != node_list.end()) {
    if (node_list_itr->keypoints.size() == 1) {
      node_list_itr->flag_no_more = true;
      node_list_itr++;
    } else if (node_list_itr->keypoints.empty()) {
      node_list_itr = node_list.erase(node_list_itr);
    } else {
      node_list_itr++;
    }
  }

  std::vector<std::pair<int, OrbExtractorNode*>> vSizeAndPointerToNode;
  vSizeAndPointerToNode.reserve(node_list.size() * 4);

  bool flag_finished = false;
  int iteration = 0;
  while (!flag_finished) {
    iteration++;

    int prevSize = node_list.size();
    int num_nodes_to_expand = 0;
    vSizeAndPointerToNode.clear();
    node_list_itr = node_list.begin();
    while (node_list_itr != node_list.end()) {
      if (node_list_itr->flag_no_more) {
        // If node only contains one point do not subdivide and continue
        node_list_itr++;
        continue;
      } else {
        // If more than one point, subdivide
        OrbExtractorNode n1, n2, n3, n4;
        node_list_itr->DivideNode(&n1, &n2, &n3, &n4);

        // Add childs if they contain points
        if (n1.keypoints.size() > 0) {
          node_list.push_front(n1);
          if (n1.keypoints.size() > 1) {
            num_nodes_to_expand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n1.keypoints.size(), &node_list.front()));
            node_list.front().node_list_iterator = node_list.begin();
          }
        }
        if (n2.keypoints.size() > 0) {
          node_list.push_front(n2);
          if (n2.keypoints.size() > 1) {
            num_nodes_to_expand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n2.keypoints.size(), &node_list.front()));
            node_list.front().node_list_iterator = node_list.begin();
          }
        }
        if (n3.keypoints.size() > 0) {
          node_list.push_front(n3);
          if (n3.keypoints.size() > 1) {
            num_nodes_to_expand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n3.keypoints.size(), &node_list.front()));
            node_list.front().node_list_iterator = node_list.begin();
          }
        }
        if (n4.keypoints.size() > 0) {
          node_list.push_front(n4);
          if (n4.keypoints.size() > 1) {
            num_nodes_to_expand++;
            vSizeAndPointerToNode.push_back(
                std::make_pair(n4.keypoints.size(), &node_list.front()));
            node_list.front().node_list_iterator = node_list.begin();
          }
        }

        node_list_itr = node_list.erase(node_list_itr);
        continue;
      }
    }

    // Finish if there are more nodes than required features
    // or all nodes contain just one point
    if (static_cast<int>(node_list.size()) >= N ||
        static_cast<int>(node_list.size()) == prevSize) {
      flag_finished = true;
    } else if ((static_cast<int>(node_list.size()) + num_nodes_to_expand * 3) >
               N) {
      while (!flag_finished) {
        prevSize = node_list.size();

        std::vector<std::pair<int, OrbExtractorNode*>>
            vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
        vSizeAndPointerToNode.clear();

        std::sort(vPrevSizeAndPointerToNode.begin(),
                  vPrevSizeAndPointerToNode.end(), compare_nodes);
        for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--) {
          OrbExtractorNode n1, n2, n3, n4;
          vPrevSizeAndPointerToNode[j].second->DivideNode(&n1, &n2, &n3, &n4);

          // Add childs if they contain points
          if (n1.keypoints.size() > 0) {
            node_list.push_front(n1);
            if (n1.keypoints.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n1.keypoints.size(), &node_list.front()));
              node_list.front().node_list_iterator = node_list.begin();
            }
          }
          if (n2.keypoints.size() > 0) {
            node_list.push_front(n2);
            if (n2.keypoints.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n2.keypoints.size(), &node_list.front()));
              node_list.front().node_list_iterator = node_list.begin();
            }
          }
          if (n3.keypoints.size() > 0) {
            node_list.push_front(n3);
            if (n3.keypoints.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n3.keypoints.size(), &node_list.front()));
              node_list.front().node_list_iterator = node_list.begin();
            }
          }
          if (n4.keypoints.size() > 0) {
            node_list.push_front(n4);
            if (n4.keypoints.size() > 1) {
              vSizeAndPointerToNode.push_back(
                  std::make_pair(n4.keypoints.size(), &node_list.front()));
              node_list.front().node_list_iterator = node_list.begin();
            }
          }

          node_list.erase(
              vPrevSizeAndPointerToNode[j].second->node_list_iterator);

          if (static_cast<int>(node_list.size()) >= N) break;
        }

        if (static_cast<int>(node_list.size()) >= N ||
            static_cast<int>(node_list.size()) == prevSize)
          flag_finished = true;
      }
    }
  }

  // Retain the best point in each node
  std::vector<cv::KeyPoint> resulting_keypoints;
  resulting_keypoints.reserve(keypoints_to_distribute.size());
  for (auto it = node_list.begin(); it != node_list.end(); it++) {
    std::vector<cv::KeyPoint>& keypoints = it->keypoints;
    cv::KeyPoint* keypoint_ptr = &keypoints[0];
    float maxResponse = keypoint_ptr->response;

    for (size_t k = 1; k < keypoints.size(); k++) {
      if (keypoints[k].response > maxResponse) {
        keypoint_ptr = &keypoints[k];
        maxResponse = keypoints[k].response;
      }
    }

    resulting_keypoints.push_back(*keypoint_ptr);
  }

  return resulting_keypoints;
}

OrbExtractor::OrbExtractor() { angle_patch_u_max_list_ = PrecomputeUmaxList(); }

std::vector<cv::Mat> OrbExtractor::ComputeImagePyramid(
    const cv::Mat& image, const int num_scale_levels,
    const double scale_factor) {
  static constexpr int kEdgeThreshold{19};

  std::vector<cv::Mat> image_pyramid(num_scale_levels);

  std::vector<double> inverse_scale_factor_list(num_scale_levels);
  inverse_scale_factor_list[0] = 1.0 / scale_factor;
  for (int level = 1; level < num_scale_levels; ++level)
    inverse_scale_factor_list[level] =
        inverse_scale_factor_list[level - 1] / scale_factor;

  const int image_height = image.rows;
  const int image_width = image.cols;

  for (int level = 0; level < num_scale_levels; ++level) {
    const double inverse_scale = inverse_scale_factor_list[level];
    cv::Size image_size(std::round(image_width * inverse_scale),
                        std::round(image_height * inverse_scale));
    cv::Size wholeSize(image_size.width + kEdgeThreshold * 2,
                       image_size.height + kEdgeThreshold * 2);
    cv::Mat temp(wholeSize, image.type()), masktemp;
    image_pyramid[level] = temp(cv::Rect(kEdgeThreshold, kEdgeThreshold,
                                         image_size.width, image_size.height));
    // Compute the resized image
    if (level != 0) {
      cv::resize(image_pyramid[level - 1], image_pyramid[level], image_size, 0,
                 0, cv::INTER_LINEAR);
      cv::copyMakeBorder(image_pyramid[level], temp, kEdgeThreshold,
                         kEdgeThreshold, kEdgeThreshold, kEdgeThreshold,
                         cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
    } else {
      cv::copyMakeBorder(image, temp, kEdgeThreshold, kEdgeThreshold,
                         kEdgeThreshold, kEdgeThreshold,
                         cv::BORDER_REFLECT_101);
    }
  }

  return image_pyramid;
}

std::vector<Feature> OrbExtractor::ExtractAndCompute(
    const std::vector<cv::Mat>& image_pyramid, const int num_max_features,
    const int num_scale_levels, const double scale_factor,
    const int fast_threshold) {
  if (static_cast<int>(image_pyramid.size()) != num_scale_levels)
    throw std::runtime_error("image_pyramid.size() != num_scale_levels");

  const auto num_features_per_level = ComputeNumFeaturesPerLevel(
      num_max_features, num_scale_levels, scale_factor);

  std::vector<Feature> all_features;

  (void)image_pyramid;
  (void)num_max_features;
  (void)fast_threshold;

  return all_features;
}

std::vector<int> OrbExtractor::PrecomputeUmaxList() {
  static constexpr int kHalfPatchSize{15};

  std::vector<int> u_max_list;
  u_max_list.resize(kHalfPatchSize + 1);

  const int v_max = std::floor(kHalfPatchSize * std::sqrt(2.0) * 0.5 + 1.0);
  const int v_min = std::ceil(kHalfPatchSize * std::sqrt(2.0) * 0.5);
  for (int v = 0; v <= v_max; ++v)
    u_max_list[v] =
        std::round(std::sqrt(kHalfPatchSize * kHalfPatchSize - v * v));

  // Make sure we are symmetric
  int v0 = 0;
  for (int v = kHalfPatchSize; v >= v_min; --v) {
    while (u_max_list[v0] == u_max_list[v0 + 1]) ++v0;
    u_max_list[v] = v0;
    ++v0;
  }

  for (int v = 0; v <= kHalfPatchSize; ++v) {
    std::cerr << v << ", " << u_max_list[v] << std::endl;
  }

  return u_max_list;
}

std::vector<int> OrbExtractor::ComputeNumFeaturesPerLevel(
    const int num_max_features, const int num_scale_levels,
    const double scale_factor) {
  std::vector<int> num_features_per_level(num_scale_levels);

  const double inverse_scale_factor = 1.0 / scale_factor;
  double num_desired_features_per_level =
      num_max_features * (1.0 - inverse_scale_factor) /
      (1.0 - std::pow(static_cast<double>(inverse_scale_factor),
                      static_cast<double>(num_scale_levels)));

  int accumulated_sum_of_num_features = 0;
  for (int level = 0; level < num_scale_levels - 1; level++) {
    num_features_per_level[level] = std::round(num_desired_features_per_level);
    accumulated_sum_of_num_features += num_features_per_level[level];
    num_desired_features_per_level *= inverse_scale_factor;
  }
  num_features_per_level[num_scale_levels - 1] =
      std::max(num_max_features - accumulated_sum_of_num_features, 0);

  return num_features_per_level;
}

std::vector<Feature> OrbExtractor::ExtractFastFeatures(
    const cv::Mat& image, const int num_features, const int level,
    const double scale_factor, const int fast_threshold) {
  static constexpr int kPatchSize{31};
  static constexpr int kEdgeThreshold{19};
  static constexpr double kCellSize{35.0};
  static constexpr bool kUseNonMaxSuppression{true};

  const int image_height = image.rows;
  const int image_width = image.cols;

  std::vector<Feature> features;
  const int minBorderX = kEdgeThreshold - 3;
  const int minBorderY = minBorderX;
  const int maxBorderX = image_width - kEdgeThreshold + 3;
  const int maxBorderY = image_height - kEdgeThreshold + 3;

  std::vector<cv::KeyPoint> keypoints_to_distribute;
  keypoints_to_distribute.reserve(num_features * 10);

  const double reduced_image_width = (maxBorderX - minBorderX);
  const double reduced_image_height = (maxBorderY - minBorderY);
  const int num_cells_u = reduced_image_width / kCellSize;
  const int num_cells_v = reduced_image_height / kCellSize;
  const int cell_width = std::ceil(reduced_image_width / num_cells_u);
  const int cell_height = std::ceil(reduced_image_height / num_cells_v);

  for (int row = 0; row < num_cells_v; row++) {
    const float left_top_v = minBorderY + row * cell_height;
    float right_bottom_v = left_top_v + cell_height + 6;

    if (left_top_v >= maxBorderY - 3) continue;
    if (right_bottom_v > maxBorderY) right_bottom_v = maxBorderY;

    for (int col = 0; col < num_cells_u; col++) {
      const float left_top_u = minBorderX + col * cell_width;
      float right_bottom_u = left_top_u + cell_width + 6;
      if (left_top_u >= maxBorderX - 6) continue;
      if (right_bottom_u > maxBorderX) right_bottom_u = maxBorderX;

      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image.rowRange(left_top_v, right_bottom_v)
                   .colRange(left_top_u, right_bottom_u),
               keypoints, fast_threshold, kUseNonMaxSuppression);

      if (keypoints.empty()) {
        const int fast_threshold_low = static_cast<int>(fast_threshold * 0.33);
        cv::FAST(image.rowRange(left_top_v, right_bottom_v)
                     .colRange(left_top_u, right_bottom_u),
                 keypoints, fast_threshold_low, kUseNonMaxSuppression);
      }
      if (keypoints.empty()) continue;

      const int u_offset = col * cell_width;
      const int v_offset = row * cell_height;
      for (auto it = keypoints.begin(); it != keypoints.end(); it++) {
        (*it).pt.x += u_offset;
        (*it).pt.y += v_offset;
        keypoints_to_distribute.push_back(*it);
      }
    }
  }

  std::vector<cv::KeyPoint> distributed_keypoints;
  distributed_keypoints =
      DistributeOctTree(keypoints_to_distribute, minBorderX, maxBorderX,
                        minBorderY, maxBorderY, num_features);

  const int scaled_patch_size = kPatchSize * std::pow(scale_factor, level - 1);

  // Add border to coordinates and scale information
  const int nkps = distributed_keypoints.size();
  for (int index = 0; index < nkps; ++index) {
    distributed_keypoints[index].pt.x += minBorderX;
    distributed_keypoints[index].pt.y += minBorderY;
    distributed_keypoints[index].octave = level;
    distributed_keypoints[index].size = scaled_patch_size;
  }

  std::vector<Feature> all_features;
  return all_features;
}

std::vector<float> OrbExtractor::CalculateFeatureAngle(
    const cv::Mat& image, const std::vector<Pixel>& pts) {
  static constexpr int kHalfPatchSize{15};

  const size_t n_pts = pts.size();
  std::vector<float> angle_list;
  angle_list.reserve(n_pts);

  for (size_t index = 0; index < n_pts; ++index) {
    const auto& pt = pts[index];
    const auto round_u = std::round(pt.x());
    const auto round_v = std::round(pt.y());
    const uint8_t* center_ptr = &image.at<uint8_t>(round_v, round_u);
    int centroid_u = 0;
    int centroid_v = 0;
    for (int u = -kHalfPatchSize; u <= kHalfPatchSize; ++u)
      centroid_u += u * center_ptr[u];

    const int image_width = image.cols;
    for (int v = 1; v <= kHalfPatchSize; ++v) {
      int v_sum = 0;
      const int u_max = angle_patch_u_max_list_[v];
      for (int u = -u_max; u <= u_max; ++u) {
        const int v_image_width = v * image_width;
        const int val_plus = center_ptr[u + v_image_width];
        const int val_minus = center_ptr[u - v_image_width];
        v_sum += (val_plus - val_minus);
        centroid_u += u * (val_plus + val_minus);
      }
      centroid_v += v * v_sum;
    }

    angle_list[index] = std::atan2(static_cast<double>(centroid_v),
                                   static_cast<double>(centroid_u));
  }

  return angle_list;
}

}  // namespace visual_odometry
