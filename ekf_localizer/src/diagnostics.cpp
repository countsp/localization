// Copyright 2023 Autoware Foundation
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

//生成诊断信息，这些信息通常用于监控和评估系统的运行状态

#include "ekf_localizer/diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <vector>

//检查一个进程是否被激活，并生成相应的诊断状态。如果进程未激活，诊断状态将被设置为警告（WARN）
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()
diagnostic_msgs::msg::DiagnosticStatus checkProcessActivated(const bool is_activated)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "is_activated";
  key_value.value = is_activated ? "True" : "False";
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (!is_activated) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "[WARN]process is not activated";
  }

  return stat;
}

//检查特定类型的测量是否已更新，并根据未更新的次数生成诊断状态。
//如果未更新的次数超过设定的阈值，诊断状态将被设置为警告（WARN）或错误（ERROR）。
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementUpdated(
  const std::string & measurement_type, const size_t no_update_count,
  const size_t no_update_count_threshold_warn, const size_t no_update_count_threshold_error)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = measurement_type + "_no_update_count";
  key_value.value = std::to_string(no_update_count);
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_no_update_count_threshold_warn";
  key_value.value = std::to_string(no_update_count_threshold_warn);
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_no_update_count_threshold_error";
  key_value.value = std::to_string(no_update_count_threshold_error);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (no_update_count >= no_update_count_threshold_warn) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "[WARN]" + measurement_type + " is not updated";
  }
  if (no_update_count >= no_update_count_threshold_error) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.message = "[ERROR]" + measurement_type + " is not updated";
  }

  return stat;
}

//检查特定测量的队列大小，并生成相应的诊断状态。
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementQueueSize(
  const std::string & measurement_type, const size_t queue_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = measurement_type + "_queue_size";
  key_value.value = std::to_string(queue_size);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";

  return stat;
}

//检查测量数据是否通过了延迟门控检查，并生成相应的诊断状态。如果测量数据的延迟超过了阈值，诊断状态将被设置为警告（WARN）。
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementDelayGate(
  const std::string & measurement_type, const bool is_passed_delay_gate, const double delay_time,
  const double delay_time_threshold)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = measurement_type + "_is_passed_delay_gate";
  key_value.value = is_passed_delay_gate ? "True" : "False";
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_delay_time";
  key_value.value = std::to_string(delay_time);
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_delay_time_threshold";
  key_value.value = std::to_string(delay_time_threshold);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (!is_passed_delay_gate) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "[WARN]" + measurement_type + " topic is delay";
  }

  return stat;
}

//检查测量数据是否通过了马氏距离门控检查，并生成相应的诊断状态。
//如果马氏距离超过了设定阈值，诊断状态将被设置为警告（WARN）
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()。
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementMahalanobisGate(
  const std::string & measurement_type, const bool is_passed_mahalanobis_gate,
  const double mahalanobis_distance, const double mahalanobis_distance_threshold)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = measurement_type + "_is_passed_mahalanobis_gate";
  key_value.value = is_passed_mahalanobis_gate ? "True" : "False";
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_mahalanobis_distance";
  key_value.value = std::to_string(mahalanobis_distance);
  stat.values.push_back(key_value);
  key_value.key = measurement_type + "_mahalanobis_distance_threshold";
  key_value.value = std::to_string(mahalanobis_distance_threshold);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (!is_passed_mahalanobis_gate) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "[WARN]mahalanobis distance of " + measurement_type + " topic is large";
  }

  return stat;
}

// The highest level within the stat_array will be reflected in the merged_stat.
// When all stat_array entries are 'OK,' the message of merged_stat will be "OK"
//合并多个诊断状态信息，生成一个综合的诊断状态。
//该函数考虑了各个诊断状态的严重程度，并将最严重的状态反映在合并后的诊断状态中。
//used in ekf_module.cpp EKFLocalizer::publishDiagnostics()
diagnostic_msgs::msg::DiagnosticStatus mergeDiagnosticStatus(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & stat_array)
{
  diagnostic_msgs::msg::DiagnosticStatus merged_stat;

  for (const auto & stat : stat_array) {
    if ((stat.level > diagnostic_msgs::msg::DiagnosticStatus::OK)) {
      if (!merged_stat.message.empty()) {
        merged_stat.message += "; ";
      }
      merged_stat.message += stat.message;
    }
    if (stat.level > merged_stat.level) {
      merged_stat.level = stat.level;
    }
    for (const auto & value : stat.values) {
      merged_stat.values.push_back(value);
    }
  }

  if (merged_stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    merged_stat.message = "OK";
  }

  return merged_stat;
}
