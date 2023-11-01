/*
  Copyright 2023 Changhyeon Kim
  e-mail: hyun91015@gmail.com
*/

#ifndef CORE_TYPES_H_
#define CORE_TYPES_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace visual_odometry {

using Pose = Eigen::Isometry3f;
using Point = Eigen::Vector3f;
using Pixel = Eigen::Vector2f;
using Position = Eigen::Vector3f;
using Rotation = Eigen::Matrix3f;
using Quaternion = Eigen::Quaternionf;

class Camera;
class Frame;
class BodyFrame;
class Landmark;

using CameraPtr = std::shared_ptr<Camera>;
using FramePtr = std::shared_ptr<Frame>;
using BodyFramePtr = std::shared_ptr<BodyFrame>;
using LandmarkPtr = std::shared_ptr<Landmark>;

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
  const int GetImageHeight() const { return image_height_; }
  const int GetImageWidth() const { return image_width_; }
  const float GetFx() const { return fx_; }
  const float GetFy() const { return fy_; }
  const float GetCx() const { return cx_; }
  const float GetCy() const { return cy_; }
  const float GetInverseFx() const { return inv_fx_; }
  const float GetInverseFx() const { return inv_fy_; }

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
 private:
  int id_;
  CameraPtr related_camera_ptr_{nullptr};
};

class BodyFrame {
 private:
  int id_;
  FramePtr left_frame_{nullptr};
  FramePtr right_frame_{nullptr};
};

class Landmark {
 private:
  int id_;
  std::vector<FramePtr> related_frame_list_;
};

}  // namespace visual_odometry

#endif  // CORE_TYPES_H_
