## Ros2
#### 创建 service ,
```
auto service = create_service<ServiceType>(
    "service_name",
    std::bind(&ClassName::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
```
#### example: /home/chopin/autoware/autoware.universe/localization/ekf_localizer/src/ekf_localizer.cpp

```
service_trigger_node_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizer::serviceTriggerNode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());
```
std::placeholders::_1 和 std::placeholders::_2 分别代表该成员函数serviceTriggerNode()的第一个和第二个参数。这些占位符在服务被实际调用时会被替换为实际的请求和响应对象。
