// Copyright 2018-2019 Autoware Foundation
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

#include "ekf_localizer/ekf_localizer.hpp"

#include "ekf_localizer/diagnostics.hpp"
#include "ekf_localizer/string.hpp"
#include "ekf_localizer/warning_message.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/msg_covariance.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}}
// clang-format on

using std::placeholders::_1;

// EKFLocalizer类的构造函数，它在Autoware Universe框架中实现了扩展卡尔曼滤波器（EKF）节点的初始化和配置。
EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(std::make_shared<Warning>(this)),
  params_(this),
  ekf_dt_(params_.ekf_dt),
  pose_queue_(params_.pose_smoothing_steps),
  twist_queue_(params_.twist_smoothing_steps)
{
  /* convert to continuous to discrete */
  //将过程噪声的标准偏差从连续时间转换为离散时间，这对于扩展卡尔曼滤波的准确性至关重要
  proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0); 
  proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);

  is_activated_ = false;

  /* initialize ros system */
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(ekf_dt_),
    std::bind(&EKFLocalizer::timerCallback, this));

  if (params_.publish_tf_) {
    timer_tf_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(params_.tf_rate_).period(),
      std::bind(&EKFLocalizer::timerTFCallback, this));
  }

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_biased_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_biased_pose", 1);
  pub_biased_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_biased_pose_with_covariance", 1);
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizer::callbackInitialPose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1, std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1, std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, _1));
  service_trigger_node_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizer::serviceTriggerNode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  ekf_module_ = std::make_unique<EKFModule>(warning_, params_);
  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);

  z_filter_.set_proc_dev(params_.z_filter_proc_dev);
  roll_filter_.set_proc_dev(params_.roll_filter_proc_dev);
  pitch_filter_.set_proc_dev(params_.pitch_filter_proc_dev);
}

/*
 * updatePredictFrequency
 */
void EKFLocalizer::updatePredictFrequency(const rclcpp::Time & current_time)
{
  if (last_predict_time_) {
    if (current_time < *last_predict_time_) {
      warning_->warn("Detected jump back in time");
    } else {
      /* Measure dt */
      ekf_dt_ = (current_time - *last_predict_time_).seconds();
      DEBUG_INFO(
        get_logger(), "[EKF] update ekf_dt_ to %f seconds (= %f hz)", ekf_dt_, 1 / ekf_dt_);

      if (ekf_dt_ > 10.0) {
        ekf_dt_ = 10.0;
        RCLCPP_WARN(
          get_logger(), "Large ekf_dt_ detected!! (%f sec) Capped to 10.0 seconds", ekf_dt_);
      } else if (ekf_dt_ > params_.pose_smoothing_steps / params_.ekf_rate) {
        RCLCPP_WARN(
          get_logger(), "EKF period may be too slow to finish pose smoothing!! (%f sec) ", ekf_dt_);
      }

      /* Register dt and accumulate time delay */
      ekf_module_->accumulate_delay_time(ekf_dt_);

      /* Update discrete proc_cov*/
      proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0);
      proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
      proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(current_time);
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  const rclcpp::Time current_time = this->now();

  if (!is_activated_) {
    warning_->warnThrottle(
      "The node is not activated. Provide initial pose to pose_initializer", 2000);
    publishDiagnostics(current_time);
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  updatePredictFrequency(current_time);

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  ekf_module_->predictWithDelay(ekf_dt_);
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  pose_diag_info_.queue_size = pose_queue_.size();
  pose_diag_info_.is_passed_delay_gate = true;
  pose_diag_info_.delay_time = 0.0;
  pose_diag_info_.delay_time_threshold = 0.0;
  pose_diag_info_.is_passed_mahalanobis_gate = true;
  pose_diag_info_.mahalanobis_distance = 0.0;

  bool pose_is_updated = false;

  if (!pose_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const auto t_curr = current_time;
    const size_t n = pose_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurementUpdatePose(*pose, t_curr, pose_diag_info_);
      if (is_updated) {
        pose_is_updated = true;

        // Update Simple 1D filter with considering change of z value due to measurement pose delay
        const double delay_time =
          (t_curr - pose->header.stamp).seconds() + params_.pose_additional_delay;
        const auto pose_with_z_delay = ekf_module_->compensatePoseWithZDelay(*pose, delay_time);
        updateSimple1DFilters(pose_with_z_delay, params_.pose_smoothing_steps);
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }
  pose_diag_info_.no_update_count = pose_is_updated ? 0 : (pose_diag_info_.no_update_count + 1);

  /* twist measurement update */
  twist_diag_info_.queue_size = twist_queue_.size();
  twist_diag_info_.is_passed_delay_gate = true;
  twist_diag_info_.delay_time = 0.0;
  twist_diag_info_.delay_time_threshold = 0.0;
  twist_diag_info_.is_passed_mahalanobis_gate = true;
  twist_diag_info_.mahalanobis_distance = 0.0;

  bool twist_is_updated = false;

  if (!twist_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const auto t_curr = current_time;
    const size_t n = twist_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurementUpdateTwist(*twist, t_curr, twist_diag_info_);
      if (is_updated) {
        twist_is_updated = true;
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }
  twist_diag_info_.no_update_count = twist_is_updated ? 0 : (twist_diag_info_.no_update_count + 1);

  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const geometry_msgs::msg::PoseStamped current_ekf_pose =
    ekf_module_->getCurrentPose(current_time, z, roll, pitch, false);
  const geometry_msgs::msg::PoseStamped current_biased_ekf_pose =
    ekf_module_->getCurrentPose(current_time, z, roll, pitch, true);
  const geometry_msgs::msg::TwistStamped current_ekf_twist =
    ekf_module_->getCurrentTwist(current_time);

  /* publish ekf result */
  publishEstimateResult(current_ekf_pose, current_biased_ekf_pose, current_ekf_twist);
  publishDiagnostics(current_time);
}

/*
 * timerTFCallback
 */
 // 将从扩展卡尔曼滤波器（EKF）获得的车辆的估计位置和方向转换为TF消息，并发布到ROS系统中
 // publish: geometry_msgs::msg::TransformStamped
void EKFLocalizer::timerTFCallback()
{
  if (!is_activated_) {
    return;
  }

  if (params_.pose_frame_id == "") {
    return;
  }

  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();

  const rclcpp::Time current_time = this->now();

  //创建一个geometry_msgs::msg::TransformStamped消息,包含了当前估计的位置和方向信息
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped = tier4_autoware_utils::pose2transform(
    ekf_module_->getCurrentPose(current_time, z, roll, pitch, false), "base_link");
  transform_stamped.header.stamp = current_time;
  
  //使用 tf_br_（TF广播器）发布TF消息
  tf_br_->sendTransform(transform_stamped);
}

/*
 * getTransformFromTF 查询是否有两个特定坐标帧之间的变换
 */

bool EKFLocalizer::getTransformFromTF(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer); // 监听TF变换，并使用 tf2::BufferCore 作为缓冲区来存储这些变换。
  rclcpp::sleep_for(std::chrono::milliseconds(100)); //给TF监听器一些时间来接收和处理TF消息。

  parent_frame = eraseLeadingSlash(parent_frame);
  child_frame = eraseLeadingSlash(child_frame);

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      warning_->warn(ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}

/*
 * callbackInitialPose 收到新的初始位置姿势数据时被触发
 */
void EKFLocalizer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!getTransformFromTF(params_.pose_frame_id, initialpose->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params_.pose_frame_id.c_str(), initialpose->header.frame_id.c_str());
  }
  ekf_module_->initialize(*initialpose, transform);
  initSimple1DFilters(*initialpose);
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!is_activated_) {
    return;
  }

  pose_queue_.push(msg);
}

/*
 * callbackTwistWithCovariance
 接收并处理带协方差的 Twist 消息。
 如果检测到消息中的速度很小（可能意味着不准确或不可靠），函数会增加该速度分量的协方差。
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Ignore twist if velocity is too small.
  // Note that this inequality must not include "equal".
  if (std::abs(msg->twist.twist.linear.x) < params_.threshold_observable_velocity_mps) {
    msg->twist.covariance[0 * 6 + 0] = 10000.0;
  }
  twist_queue_.push(msg);
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::publishEstimateResult(
  const geometry_msgs::msg::PoseStamped & current_ekf_pose,
  const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
  const geometry_msgs::msg::TwistStamped & current_ekf_twist)
{
  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose);
  pub_biased_pose_->publish(current_biased_ekf_pose);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_ekf_pose.header.stamp;
  pose_cov.header.frame_id = current_ekf_pose.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose.pose;
  pose_cov.pose.covariance = ekf_module_->getCurrentPoseCovariance();
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_cov = pose_cov;
  biased_pose_cov.pose.pose = current_biased_ekf_pose.pose;
  pub_biased_pose_cov_->publish(biased_pose_cov);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_ekf_twist.header.stamp;
  twist_cov.header.frame_id = current_ekf_twist.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist.twist;
  twist_cov.twist.covariance = ekf_module_->getCurrentTwistCovariance();
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  tier4_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_ekf_twist.header.stamp;
  yawb.data = ekf_module_->getYawBias();
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_ekf_pose.header.stamp;
  odometry.header.frame_id = current_ekf_pose.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);
}

//发布有关 EKFLocalizer 节点当前状态的诊断信息。
//这些信息对于监控节点的健康状况和性能非常重要。
//diag_staus_array -> diag_merged_status -> diag_msg -> publish to "/diagnostics"
//uses: diagnostics.cpp checkProcessActivated()_ 检查一个进程是否被激活，并生成相应的诊断状态。如果进程未激活，诊断状态将被设置为警告（WARN）
//uses: diagnostics.cpp checkMeasurementUpdated() 检查特定类型的测量是否已更新，并根据未更新的次数生成诊断状态。
//uses: diagnostics.cpp checkMeasurementQueueSize() 检查特定测量的队列大小，并生成相应的诊断状态。
//uses: diagnostics.cpp checkMeasurementDelayGate() 检查测量数据是否通过了延迟门控检查，并生成相应的诊断状态。如果测量数据的延迟超过了阈值，诊断状态将被设置为警告（WARN）。
//uses: diagnostics.cpp checkMeasurementMahalanobisGate() 检查测量数据是否通过了马氏距离门控检查，并生成相应的诊断状态。如果马氏距离超过了设定阈值，诊断状态将被设置为警告（WARN）。
//uses: diagnostics.cpp mergeDiagnosticStatus() 合并多个诊断状态信息，生成一个综合的诊断状态。
void EKFLocalizer::publishDiagnostics(const rclcpp::Time & current_time)
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;

  diag_status_array.push_back(checkProcessActivated(is_activated_));

  if (is_activated_) {
    diag_status_array.push_back(checkMeasurementUpdated(
      "pose", pose_diag_info_.no_update_count, params_.pose_no_update_count_threshold_warn,
      params_.pose_no_update_count_threshold_error));
    diag_status_array.push_back(checkMeasurementQueueSize("pose", pose_diag_info_.queue_size));
    diag_status_array.push_back(checkMeasurementDelayGate(
      "pose", pose_diag_info_.is_passed_delay_gate, pose_diag_info_.delay_time,
      pose_diag_info_.delay_time_threshold));
    diag_status_array.push_back(checkMeasurementMahalanobisGate(
      "pose", pose_diag_info_.is_passed_mahalanobis_gate, pose_diag_info_.mahalanobis_distance,
      params_.pose_gate_dist));

    diag_status_array.push_back(checkMeasurementUpdated(
      "twist", twist_diag_info_.no_update_count, params_.twist_no_update_count_threshold_warn,
      params_.twist_no_update_count_threshold_error));
    diag_status_array.push_back(checkMeasurementQueueSize("twist", twist_diag_info_.queue_size));
    diag_status_array.push_back(checkMeasurementDelayGate(
      "twist", twist_diag_info_.is_passed_delay_gate, twist_diag_info_.delay_time,
      twist_diag_info_.delay_time_threshold));
    diag_status_array.push_back(checkMeasurementMahalanobisGate(
      "twist", twist_diag_info_.is_passed_mahalanobis_gate, twist_diag_info_.mahalanobis_distance,
      params_.twist_gate_dist));
  }

  diagnostic_msgs::msg::DiagnosticStatus diag_merged_status;
  diag_merged_status = mergeDiagnosticStatus(diag_status_array);
  diag_merged_status.name = "localization: " + std::string(this->get_name());
  diag_merged_status.hardware_id = this->get_name();

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = current_time;
  diag_msg.status.push_back(diag_merged_status);
  pub_diag_->publish(diag_msg);
}

//更新几个简单的一维滤波器
//这些滤波器用于平滑和处理位置（pose）数据中的高度（z）、横滚（roll）和俯仰（pitch）分量
void EKFLocalizer::updateSimple1DFilters(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose, const size_t smoothing_step)
{
  double z = pose.pose.pose.position.z;

  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_dev = pose.pose.covariance[COV_IDX::Z_Z] * static_cast<double>(smoothing_step);
  double roll_dev = pose.pose.covariance[COV_IDX::ROLL_ROLL] * static_cast<double>(smoothing_step);
  double pitch_dev =
    pose.pose.covariance[COV_IDX::PITCH_PITCH] * static_cast<double>(smoothing_step);

  z_filter_.update(z, z_dev, pose.header.stamp);
  roll_filter_.update(rpy.x, roll_dev, pose.header.stamp);
  pitch_filter_.update(rpy.y, pitch_dev, pose.header.stamp);
}
//一维滤波器 initialize
void EKFLocalizer::initSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  double z = pose.pose.pose.position.z;

  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_dev = pose.pose.covariance[COV_IDX::Z_Z];
  double roll_dev = pose.pose.covariance[COV_IDX::ROLL_ROLL];
  double pitch_dev = pose.pose.covariance[COV_IDX::PITCH_PITCH];

  z_filter_.init(z, z_dev, pose.header.stamp);
  roll_filter_.init(rpy.x, roll_dev, pose.header.stamp);
  pitch_filter_.init(rpy.y, pitch_dev, pose.header.stamp);
}

/**
 * @brief trigger node
 */
void EKFLocalizer::serviceTriggerNode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    pose_queue_.clear();
    twist_queue_.clear();
    is_activated_ = true;
  } else {
    is_activated_ = false;
  }
  res->success = true;
  return;
}
