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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "core/stereo_visual_odometry.h"
#include "core/types.h"

#include "yaml-cpp/yaml.h"

void PrintMessageImpl(const std::string& message,
                      const std::string& function_name) {
  std::stringstream ss;
  ss << "[Info](" << function_name << "): " << message << "\n";
  std::cerr << ss.str();
}
#define PrintMessage(message) PrintMessageImpl((message), std::string(__func__))

void ThrowMessageImpl(const std::string& message,
                      const std::string& function_name) {
  std::stringstream ss;
  ss << "[Error](" << function_name << "): " << message << "\n";
  throw std::runtime_error(ss.str());
}
#define ThrowMessage(message) ThrowMessageImpl((message), std::string(__func__))

namespace visual_odometry {

StereoVisualOdometry::StereoVisualOdometry() {
  const std::string config_file_directory(
      "/home/kch/ros2_ws/src/stereo_visual_odometry_ros2/config/stereo/d435/");
  const std::string user_parameters_file_path(config_file_directory +
                                              "user_parameters.yaml");
  const std::string intrinsic_parameters_file_path(config_file_directory +
                                                   "intrinsics.yaml");
  const std::string extrinsic_parameters_file_path(config_file_directory +
                                                   "extrinsics.yaml");

  StereoVisualOdometry::Parameters user_parameters;
  if (!LoadUserParameters(user_parameters_file_path, &user_parameters))
    ThrowMessage("Loading user parameters is failed.");
  else
    PrintMessage("Loading user parameters is successfully done!");

  if (!LoadIntrinsicsAndExtrinsicsParameters(intrinsic_parameters_file_path,
                                             extrinsic_parameters_file_path))
    ThrowMessage("Loading intrinsic and extrinsic parameters is failed.");
  else
    PrintMessage(
        "Loading intrinsic and extrinsic parameters is successfully done!");

  // Initialize orb extractor
  orb_extractor_ = std::make_unique<OrbExtractor>();
}

bool StereoVisualOdometry::TrackStereoImages(const double timestamp,
                                             const cv::Mat& left_image,
                                             const cv::Mat& right_image) {
  bool is_track_succeeded = true;
  (void)timestamp;
  (void)left_image;
  (void)right_image;

  return is_track_succeeded;
}

bool StereoVisualOdometry::LoadIntrinsicsAndExtrinsicsParameters(
    const std::string& intrinsic_file_path,
    const std::string& extrinsic_file_path) {
  try {
    YAML::Node intrinsic_config = YAML::LoadFile(intrinsic_file_path);
    YAML::Node extrinsic_config = YAML::LoadFile(extrinsic_file_path);

    int image_height{0};
    int image_width{0};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};

    const auto left_params = intrinsic_config["left"];
    image_height = left_params["image_height"].as<int>();
    image_width = left_params["image_width"].as<int>();
    fx = left_params["projection"]["fx"].as<double>();
    fy = left_params["projection"]["fy"].as<double>();
    cx = left_params["projection"]["cx"].as<double>();
    cy = left_params["projection"]["cy"].as<double>();

    const auto left_extrinsic = extrinsic_config["body_to_left"];
    Position trans_body_to_left{Position::Zero()};
    trans_body_to_left.x() = left_extrinsic["position"]["x"].as<double>();
    trans_body_to_left.y() = left_extrinsic["position"]["y"].as<double>();
    trans_body_to_left.z() = left_extrinsic["position"]["z"].as<double>();
    Quaternion quat_body_to_left{Quaternion::Identity()};
    quat_body_to_left.w() = left_extrinsic["orientation"]["w"].as<double>();
    quat_body_to_left.x() = left_extrinsic["orientation"]["x"].as<double>();
    quat_body_to_left.y() = left_extrinsic["orientation"]["y"].as<double>();
    quat_body_to_left.z() = left_extrinsic["orientation"]["z"].as<double>();
    Pose T_body_to_left_cam{Pose::Identity()};
    T_body_to_left_cam.translation() = trans_body_to_left.cast<float>();
    T_body_to_left_cam.linear() =
        quat_body_to_left.toRotationMatrix().cast<float>();

    left_cam_ = std::make_shared<Camera>(image_height, image_width, fx, fy, cx,
                                         cy, T_body_to_left_cam);

    const auto right_params = intrinsic_config["right"];
    image_height = right_params["image_height"].as<int>();
    image_width = right_params["image_width"].as<int>();
    fx = right_params["projection"]["fx"].as<double>();
    fy = right_params["projection"]["fy"].as<double>();
    cx = right_params["projection"]["cx"].as<double>();
    cy = right_params["projection"]["cy"].as<double>();

    const auto right_extrinsic = extrinsic_config["body_to_right"];
    Position trans_body_to_right{Position::Zero()};
    trans_body_to_right.x() = right_extrinsic["position"]["x"].as<double>();
    trans_body_to_right.y() = right_extrinsic["position"]["y"].as<double>();
    trans_body_to_right.z() = right_extrinsic["position"]["z"].as<double>();
    Quaternion quat_body_to_right{Quaternion::Identity()};
    quat_body_to_right.w() = right_extrinsic["orientation"]["w"].as<double>();
    quat_body_to_right.x() = right_extrinsic["orientation"]["x"].as<double>();
    quat_body_to_right.y() = right_extrinsic["orientation"]["y"].as<double>();
    quat_body_to_right.z() = right_extrinsic["orientation"]["z"].as<double>();
    Pose T_body_to_right_cam{Pose::Identity()};
    T_body_to_right_cam.translation() = trans_body_to_right.cast<float>();
    T_body_to_right_cam.linear() =
        quat_body_to_right.toRotationMatrix().cast<float>();

    right_cam_ = std::make_shared<Camera>(image_height, image_width, fx, fy, cx,
                                          cy, T_body_to_right_cam);
  } catch (const YAML::BadFile& e) {
    throw std::runtime_error("BadFile: " + e.msg);
  } catch (const YAML::ParserException& e) {
    throw std::runtime_error("ParserException: " + e.msg);
  }

  return true;
}

// bool StereoVisualOdometry::LoadExtrinsicParameters(
//     const std::string& file_path) {}

bool StereoVisualOdometry::LoadUserParameters(const std::string& file_path,
                                              Parameters* parameters) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);

    // feature
    const auto feature_params = config["feature"];

    // feature.index_grid
    const auto index_grid_params = feature_params["index_grid"];
    parameters->feature.index_grid.cell_height =
        index_grid_params["cell_height"].as<int>();
    parameters->feature.index_grid.cell_height =
        index_grid_params["cell_width"].as<int>();
    PrintMessage("feature.index_grid.cell_height:" +
                 std::to_string(parameters->feature.index_grid.cell_height));
    PrintMessage("feature.index_grid.cell_width:" +
                 std::to_string(parameters->feature.index_grid.cell_width));

    // feature.image_pyramid
    const auto image_pyramid_params = feature_params["image_pyramid"];
    parameters->feature.image_pyramid.num_scale_levels =
        image_pyramid_params["num_scale_levels"].as<int>();
    parameters->feature.image_pyramid.scale_factor =
        image_pyramid_params["scale_factor"].as<double>();
    PrintMessage(
        "feature.image_pyramid.num_scale_levels:" +
        std::to_string(parameters->feature.image_pyramid.num_scale_levels));
    PrintMessage(
        "feature.image_pyramid.scale_factor:" +
        std::to_string(parameters->feature.image_pyramid.scale_factor));

    // feature.extractor
    const auto extractor_params = feature_params["extractor"];
    parameters->feature.extractor.num_features =
        extractor_params["num_features"].as<int>();
    parameters->feature.extractor.fast_threshold_high =
        extractor_params["fast_threshold_high"].as<int>();
    parameters->feature.extractor.fast_threshold_low =
        extractor_params["fast_threshold_low"].as<int>();
    parameters->feature.extractor.feature_selection.cell_height =
        extractor_params["feature_selection"]["cell_height"].as<int>();
    parameters->feature.extractor.feature_selection.cell_width =
        extractor_params["feature_selection"]["cell_width"].as<int>();

    // feature.matcher
    const auto matcher_params = feature_params["matcher"];
    parameters->feature.matcher.max_discriptor_distance =
        matcher_params["max_discriptor_distance"].as<int>();

    // optimization
    const auto optimization_params = config["optimization"];
    parameters->optimization.threshold_huber =
        optimization_params["threshold_huber"].as<double>();
  } catch (const YAML::BadFile& e) {
    throw std::runtime_error("BadFile: " + e.msg);
  } catch (const YAML::ParserException& e) {
    throw std::runtime_error("ParserException: " + e.msg);
  }

  return true;
}

}  // namespace visual_odometry
