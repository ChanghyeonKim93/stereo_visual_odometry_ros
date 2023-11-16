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

#ifndef VISUAL_ODOMETRY_TYPES_H_
#define VISUAL_ODOMETRY_TYPES_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace visual_odometry {

using Pose = Eigen::Transform<float, 3, 1>;
using Point = Eigen::Matrix<float, 3, 1>;
using Pixel = Eigen::Matrix<float, 2, 1>;
using Position = Eigen::Matrix<float, 3, 1>;
using Rotation = Eigen::Matrix<float, 3, 3>;
using Quaternion = Eigen::Quaternion<float>;
using Descriptor = std::vector<uint8_t>;

class Camera;
class Frame;
class BodyFrame;
class Landmark;

using CameraPtr = std::shared_ptr<Camera>;
using FramePtr = std::shared_ptr<Frame>;
using BodyFramePtr = std::shared_ptr<BodyFrame>;
using LandmarkPtr = std::shared_ptr<Landmark>;

struct Feature {
  Pixel pixel{Pixel::Zero()};
  Descriptor descriptor{};
  int octave{0};
  float angle{0.0f};
};

class Camera {
 public:
  Camera()
      : image_height_(0),
        image_width_(0),
        fx_(0.0f),
        fy_(0.0f),
        cx_(0.0f),
        cy_(0.0f),
        inv_fx_(0.0f),
        inv_fy_(0.0f),
        T_b2c_(Pose::Identity()),
        T_c2b_(Pose::Identity()) {}
  Camera(const int image_height, const int image_width, const float fx,
         const float fy, const float cx, const float cy,
         const Pose& body_frame_to_camera_pose)
      : image_height_(image_height),
        image_width_(image_width),
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy) {
    static constexpr float kMinFloatNumber = 0.001f;
    if (fx_ <= kMinFloatNumber) throw std::runtime_error("fx_ is negative");
    if (fy_ <= kMinFloatNumber) throw std::runtime_error("fy_ is negative");
    inv_fx_ = 1.0f / fx_;
    inv_fy_ = 1.0f / fy_;
    T_b2c_ = body_frame_to_camera_pose;
    T_c2b_ = T_b2c_.inverse();
  }
  Camera(const Camera& rhs)
      : image_height_(rhs.image_height_),
        image_width_(rhs.image_width_),
        fx_(rhs.fx_),
        fy_(rhs.fy_),
        cx_(rhs.cx_),
        cy_(rhs.cy_),
        inv_fx_(rhs.inv_fx_),
        inv_fy_(rhs.inv_fy_),
        T_b2c_(rhs.T_b2c_),
        T_c2b_(rhs.T_c2b_) {}

 public:  // Getters
  int GetImageHeight() const { return image_height_; }
  int GetImageWidth() const { return image_width_; }
  float GetFx() const { return fx_; }
  float GetFy() const { return fy_; }
  float GetCx() const { return cx_; }
  float GetCy() const { return cy_; }
  float GetInverseFx() const { return inv_fx_; }
  float GetInverseFy() const { return inv_fy_; }

 private:
  int image_height_;
  int image_width_;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  float inv_fx_;
  float inv_fy_;
  Pose T_b2c_;  // body frame to camera
  Pose T_c2b_;  // camera to body frame
};

class Frame {
 public:
  inline static int id_counter_{0};

 public:
  explicit Frame(const BodyFramePtr& parent_body_frame_ptr,
                 const CameraPtr& related_camera_ptr)
      : id_(id_counter_++),
        parent_body_frame_ptr_(parent_body_frame_ptr),
        related_camera_ptr_(related_camera_ptr) {}

 public:  // getters
  const BodyFramePtr& GetParentBodyFrame() const {
    return parent_body_frame_ptr_;
  }
  const CameraPtr& GetRelatedCamera() const { return related_camera_ptr_; }
  const std::unordered_set<LandmarkPtr>& GetObservedLandmarkSet() const {
    return observed_landmark_set_;
  }

 public:  // setters
  void AddObservedLandmark(const LandmarkPtr& landmark) {
    observed_landmark_set_.insert(landmark);
  }

 private:
  int id_;
  BodyFramePtr parent_body_frame_ptr_{nullptr};
  CameraPtr related_camera_ptr_{nullptr};
  std::unordered_set<LandmarkPtr> observed_landmark_set_;
};

class BodyFrame {
 public:
  inline static int id_counter_{0};
  explicit BodyFrame(const double timestamp)
      : id_(id_counter_++),
        timestamp_(timestamp),
        left_frame_(nullptr),
        right_frame_(nullptr),
        world_to_body_frame_pose_(Pose::Identity()) {}

 public:  // getters
  int GetId() const { return id_; }
  double GetTimestamp() const { return timestamp_; }
  const FramePtr& GetLeftFrame() const { return left_frame_; }
  const FramePtr& GetRightFrame() const { return right_frame_; }
  const Pose& GetWorldToBodyFramePose() const {
    return world_to_body_frame_pose_;
  }

 public:  // setters
  void SetLeftFrame(const FramePtr& left_frame) { left_frame_ = left_frame; }
  void SetRightFrame(const FramePtr& right_frame) {
    right_frame_ = right_frame;
  }
  void SetWorldToBodyFramePose(const Pose& world_to_body_frame_pose) {
    world_to_body_frame_pose_ = world_to_body_frame_pose;
  }

 private:
  int id_;
  double timestamp_;
  FramePtr left_frame_{nullptr};
  FramePtr right_frame_{nullptr};
  Pose world_to_body_frame_pose_;
};

class Landmark {
 public:
  inline static int id_counter_{0};

 public:  // getters
  int GetId() const { return id_; }
  const Point& GetWorldPoint() const { return world_point_; }
  const std::unordered_set<FramePtr>& GetRelatedFrameSet() const {
    return related_frame_set_;
  }

 public:  // setters
  void AddRelatedFrame(const FramePtr& frame) {
    related_frame_set_.insert(frame);
  }
  void SetWorldPoint(const Point& world_point) { world_point_ = world_point; }

 private:
  int id_;
  Point world_point_;  // world point
  std::unordered_set<FramePtr> related_frame_set_;
};

}  // namespace visual_odometry

#endif  // VISUAL_ODOMETRY_TYPES_H_
