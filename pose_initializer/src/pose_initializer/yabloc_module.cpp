// Copyright 2022 The Autoware Contributors
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

#include "yabloc_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

YabLocModule::YabLocModule(rclcpp::Node * node) : logger_(node->get_logger())
{
  cli_align_ = node->create_client<RequestPoseAlignment>("yabloc_align");
}

//  创建一个服务请求，包含需要对齐的位姿。
//  检查YabLoc对齐服务是否就绪，如果服务未就绪，则抛出一个ServiceUnready异常。
//  向YabLoc对齐服务发送请求，并等待响应。
//  如果服务调用成功，返回对齐后的位姿。如果失败，记录错误信息并抛出一个ServiceException异常，指明YabLoc对齐服务失败。
PoseWithCovarianceStamped YabLocModule::align_pose(const PoseWithCovarianceStamped & pose)
{
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  req->pose_with_covariance = pose;

  if (!cli_align_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("YabLoc align server is not ready.");
  }

  RCLCPP_INFO(logger_, "Call YabLoc align server.");
  const auto res = cli_align_->async_send_request(req).get();
  if (!res->success) {
    RCLCPP_INFO(logger_, "YabLoc align server failed.");
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "YabLoc align server failed.");
  }
  RCLCPP_INFO(logger_, "YabLoc align server succeeded.");

  // Overwrite the covariance.
  return res->pose_with_covariance;
}
