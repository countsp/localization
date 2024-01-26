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

#### geometry_msgs::msg::PoseStamped
geometry_msgs::msg::PoseStamped 是 ROS 2（Robot Operating System 2）中的一种消息类型，它用于表示一个带时间戳的位姿（pose）。这个消息类型在 ROS 2 中广泛用于各种应用，尤其是在需要传输或处理位置和方向信息的场合，如机器人导航、自动驾驶汽车等。

头部（Header）:
    
    时间戳 stamp 表示位姿数据的时间点
    
    frame_id 用于指定位姿数据相对于哪个坐标系

位姿（Pose）:
    
    位置（position）使用 geometry_msgs::msg::Point 类型表示，包含 x、y 和 z 三个坐标值。
    
    方向（orientation）使用四元数（geometry_msgs::msg::Quaternion）表示，包含 x、y、z、w 四个分量，用于描述物体的朝向。

#### geometry_msgs::msg::TwistStamped

头部（Header）:

    时间戳 stamp 表示消息中的数据是何时采集的
    
    frame_id 用于指定速度数据相对于哪个坐标系。

速度（Twist）:

    线性速度（linear）包含沿三个轴（x、y、z）的速度分量，通常用于描述直线运动。
    
    角速度（angular）包含围绕三个轴（x、y、z）的旋转速度分量，通常用于描述旋转运动。
