// Copyright 2022 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 创建测量矩阵和测量协方差矩阵
#include "ekf_localizer/measurement.hpp"

#include "ekf_localizer/state_index.hpp"
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

/*----------------------- create -----------------------------------*/
//创建并返回一个 3x6 的矩阵 C，用于位姿测量的更新。
//矩阵 C 映射了 EKF 状态向量的位置和偏航角部分（x, y, yaw）到测量空间。
//used in ekf_module.cpp
Eigen::Matrix<double, 3, 6> poseMeasurementMatrix()
{
  Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw
  return C;
}

//创建并返回一个 2x6 的矩阵 C，用于速度（Twist）测量的更新。
//矩阵 C 映射了 EKF 状态向量的速度部分（vx, wz）到测量空间
//used in ekf_module.cpp
Eigen::Matrix<double, 2, 6> twistMeasurementMatrix()
{
  Eigen::Matrix<double, 2, 6> C = Eigen::Matrix<double, 2, 6>::Zero();
  C(0, IDX::VX) = 1.0;  // for vx
  C(1, IDX::WZ) = 1.0;  // for wz
  return C;
}
/*------------------------------ extract ---------------------------------*/
//    创建并返回一个 3x3 的协方差矩阵 R，用于位姿测量的协方差。
//    从提供的 36 元素协方差数组中提取位姿相关的协方差值，考虑平滑步数（smoothing_step）。
//used in ekf_module.cpp
Eigen::Matrix3d poseMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix3d R;
  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  R << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_Y), covariance.at(COV_IDX::X_YAW),
    covariance.at(COV_IDX::Y_X), covariance.at(COV_IDX::Y_Y), covariance.at(COV_IDX::Y_YAW),
    covariance.at(COV_IDX::YAW_X), covariance.at(COV_IDX::YAW_Y), covariance.at(COV_IDX::YAW_YAW);
  return R * static_cast<double>(smoothing_step);
}

//    创建并返回一个 2x2 的协方差矩阵 R，用于速度测量的协方差。
//    从提供的 36 元素协方差数组中提取速度相关的协方差值，同样考虑平滑步数。
//used in ekf_module.cpp
Eigen::Matrix2d twistMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix2d R;
  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  R << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_YAW), covariance.at(COV_IDX::YAW_X),
    covariance.at(COV_IDX::YAW_YAW);
  return R * static_cast<double>(smoothing_step);
}
