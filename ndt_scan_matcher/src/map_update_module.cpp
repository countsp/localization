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

#include "ndt_scan_matcher/map_update_module.hpp"

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
  std::shared_ptr<NormalDistributionsTransform> ndt_ptr)
: ndt_ptr_(std::move(ndt_ptr)),
  ndt_ptr_mutex_(ndt_ptr_mutex),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  dynamic_map_loading_update_distance_(
    node->declare_parameter<double>("dynamic_map_loading_update_distance")),
  dynamic_map_loading_map_radius_(
    node->declare_parameter<double>("dynamic_map_loading_map_radius")),
  lidar_radius_(node->declare_parameter<double>("lidar_radius"))
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");
}

//判断是否需要更新地图。基于车辆当前位置与上次更新位置之间的距离
//如果这个距离超过了预设的更新距离阈值 (dynamic_map_loading_update_distance_)，则认为需要更新地图
bool MapUpdateModule::should_update_map(const geometry_msgs::msg::Point & position) const
{
  if (last_update_position_ == std::nullopt) {
    return false;
  }
  const double dx = position.x - last_update_position_.value().x;
  const double dy = position.y - last_update_position_.value().y;
  const double distance = std::hypot(dx, dy);
  if (distance + lidar_radius_ > dynamic_map_loading_map_radius_) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, "Dynamic map loading is not keeping up.");
  }
  return distance > dynamic_map_loading_update_distance_;
}
//执行地图更新操作。该函数向地图加载服务发送请求，请求新的点云数据来更新当前地图。
void MapUpdateModule::update_map(const geometry_msgs::msg::Point & position)
{
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(dynamic_map_loading_map_radius_);
  request->cached_ids = ndt_ptr_->getCurrentMapIDs();

  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Waiting for pcd loader service. Check the pointcloud_map_loader.");
  }

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(logger_, "waiting response");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }
  update_ndt(result.get()->new_pointcloud_with_ids, result.get()->ids_to_remove);
  last_update_position_ = position;
}

//更新NDT算法的目标地图。
//该函数首先将要添加的点云数据从ROS消息格式转换为PCL格式，然后将这些点云数据添加到NDT算法的目标地图中，并移除旧的地图数据。
void MapUpdateModule::update_ndt(
  const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & maps_to_add,
  const std::vector<std::string> & map_ids_to_remove)
{
  RCLCPP_INFO(
    logger_, "Update map (Add: %lu, Remove: %lu)", maps_to_add.size(), map_ids_to_remove.size());
  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    RCLCPP_INFO(logger_, "Skip map update");
    return;
  }
  const auto exe_start_time = std::chrono::system_clock::now();

  const size_t add_size = maps_to_add.size();

  // Perform heavy processing outside of the lock scope
  std::vector<pcl::shared_ptr<pcl::PointCloud<PointTarget>>> points_pcl(add_size);
  for (size_t i = 0; i < add_size; i++) {
    points_pcl[i] = pcl::make_shared<pcl::PointCloud<PointTarget>>();
    pcl::fromROSMsg(maps_to_add[i].pointcloud, *points_pcl[i]);
  }

  (*ndt_ptr_mutex_).lock();

  // Add pcd
  for (size_t i = 0; i < add_size; i++) {
    ndt_ptr_->addTarget(points_pcl[i], maps_to_add[i].cell_id);
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    ndt_ptr_->removeTarget(map_id_to_remove);
  }

  ndt_ptr_->createVoxelKdtree();

  (*ndt_ptr_mutex_).unlock();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  RCLCPP_INFO(logger_, "Time duration for creating new ndt_ptr: %lf [ms]", exe_time);

  publish_partial_pcd_map();
}

//发布更新后的部分点云地图。
//该函数获取NDT算法目标地图中的点云数据，并将这些数据发布到一个ROS主题上，以便其他模块或节点可以订阅并使用这些数据。
void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl = ndt_ptr_->getVoxelPCD();

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}
