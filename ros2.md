## Ros2

### 创建 service ,
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

---

### geometry_msgs::msg::PoseStamped
geometry_msgs::msg::PoseStamped 是 ROS 2（Robot Operating System 2）中的一种消息类型，它用于表示一个带时间戳的位姿（pose）。这个消息类型在 ROS 2 中广泛用于各种应用，尤其是在需要传输或处理位置和方向信息的场合，如机器人导航、自动驾驶汽车等。

头部（Header）:
    
    时间戳 stamp 表示位姿数据的时间点
    
    frame_id 用于指定位姿数据相对于哪个坐标系

位姿（Pose）:
    
    位置（position）使用 geometry_msgs::msg::Point 类型表示，包含 x、y 和 z 三个坐标值。
    
    方向（orientation）使用四元数（geometry_msgs::msg::Quaternion）表示，包含 x、y、z、w 四个分量，用于描述物体的朝向。
    


### geometry_msgs::msg::TwistStamped

头部（Header）:

    时间戳 stamp 表示消息中的数据是何时采集的
    
    frame_id 用于指定速度数据相对于哪个坐标系。

速度（Twist）:

    线性速度（linear）包含沿三个轴（x、y、z）的速度分量，通常用于描述直线运动。
    
    角速度（angular）包含围绕三个轴（x、y、z）的旋转速度分量，通常用于描述旋转运动。

--- 

### declare_parameter
在ROS 2中，参数是一种节点可以使用的配置选项。这些参数可以在节点运行时动态地设置和更改。declare_parameter 函数不仅声明了一个新的参数，还可以用来获取参数的值。如果参数已经在参数服务器上设置，它将返回该值；如果没有设置，它将返回你提供的默认值。

this->declare_parameter<Type>("parameter_name", default_value) 中
Type：参数的数据类型（例如，int64_t, double, std::string等）。
"parameter_name"：参数的名称。
default_value：如果参数未在参数服务器上设置，则使用的默认值。

### create_callback_group
example: /home/chopin/autoware.universe.read/localization/ndt_scan_matcher/src/ndt_scan_matcher_core.cpp
```
  //在同一时间内只有一个回调可以被执行。
  //这有助于避免多个回调同时执行时可能出现的资源竞争或数据不一致的问题
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```
