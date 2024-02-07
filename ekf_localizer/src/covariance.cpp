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

//将扩展卡尔曼滤波器（EKF）的协方差矩阵转换为适用于 ROS 消息的格式

#include "ekf_localizer/covariance.hpp"

#include "ekf_localizer/state_index.hpp"
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

//  这个函数接受一个 6x6 的协方差矩阵 P（通常是 EKF 的状态协方差矩阵）。
//  它将这个矩阵转换成一个 36 元素的数组，这个数组代表 ROS 中 PoseWithCovariance 消息的协方差部分。
//  函数选择 P 矩阵中与位姿相关的元素（位置 x, y 和偏航角 yaw）并将它们放置在正确的位置上，以符合 ROS 消息的格式。
//  used: ekf_module.cpp EKFModule::getCurrentPoseCovariance() 
std::array<double, 36> ekfCovarianceToPoseMessageCovariance(const Matrix6d & P)
{
  std::array<double, 36> covariance;
  covariance.fill(0.);

  covariance[COV_IDX::X_X] = P(IDX::X, IDX::X);
  covariance[COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  covariance[COV_IDX::X_YAW] = P(IDX::X, IDX::YAW);
  covariance[COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  covariance[COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  covariance[COV_IDX::Y_YAW] = P(IDX::Y, IDX::YAW);
  covariance[COV_IDX::YAW_X] = P(IDX::YAW, IDX::X);
  covariance[COV_IDX::YAW_Y] = P(IDX::YAW, IDX::Y);
  covariance[COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);

  return covariance;
}

//  这个函数同样接受一个 6x6 的协方差矩阵 P。
//  它转换这个矩阵为一个代表 ROS 中 TwistWithCovariance 消息的协方差部分的 36 元素数组。
//  这里主要关注的是与速度（线性速度 vx 和角速度 wz）相关的协方差元素。
//  used: ekf_module.cpp EKFModule::getCurrentTwistCovariance()
std::array<double, 36> ekfCovarianceToTwistMessageCovariance(const Matrix6d & P)
{
  std::array<double, 36> covariance;
  covariance.fill(0.);

  covariance[COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  covariance[COV_IDX::X_YAW] = P(IDX::VX, IDX::WZ);
  covariance[COV_IDX::YAW_X] = P(IDX::WZ, IDX::VX);
  covariance[COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);

  return covariance;
}
