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

//用于计算两个向量之间的马氏距离（Mahalanobis Distance）

#include "ekf_localizer/mahalanobis.hpp"


//计算向量 x 和 y 之间的平方马氏距离。
//Eigen::VectorXd & x，Eigen::VectorXd & y（两个向量），Eigen::MatrixXd & C（协方差矩阵的逆）。
//计算方法是首先计算差向量 d，然后使用协方差矩阵的逆 C.inverse() 来计算 d 在这个变换下的长度的平方。
double squaredMahalanobis(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C)
{
  const Eigen::VectorXd d = x - y;
  return d.dot(C.inverse() * d);
}

//获取 squaredMahalanobis 函数的结果并取其平方根来实现
//used in ekf_module.cpp 
double mahalanobis(const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C)
{
  return std::sqrt(squaredMahalanobis(x, y, C));
}
