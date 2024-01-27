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

### static_cast<double>
static_cast<double> () 是一个类型转换表达式，它将变量从它的原始类型转换为 double 类型。

static_cast 是 C++ 中四种类型转换运算符之一，用于执行非多态类型的转换。这意味着你可以用它来转换基本数据类型（如整数和浮点数），以及向上或向下转换非多态类的对象。

##### 示例和用途

```
size_t smoothing_step = 5;
double result = static_cast<double>(smoothing_step) / 2.0;
```
在这个例子中，如果没有 static_cast<double>(smoothing_step)，则 smoothing_step / 2.0 的结果会首先执行整数除法，然后将结果转换为 double，这可能不是预期的行为。通过使用 static_cast<double>，确保 smoothing_step 以 double 类型参与运算，从而获得正确的浮点数除法结果。

---
#### ()[]=""
```
state_ptr_(new std::map<std::string, std::string>)
(*state_ptr_)["state"] = "Initializing";
```

这段代码是C++中的一种特殊语法，用于访问和修改一个指针指向的对象中的元素。在这个特定的例子中，state_ptr_ 是指向 std::map<std::string, std::string> 类型对象的指针。

(*state_ptr_)：首先，使用 * 运算符来解引用 state_ptr_ 指针。这意味着它访问指针指向的实际 std::map 对象。

["state"] = "Initializing";：然后，使用 std::map 的 [] 运算符来访问键为 "state" 的元素。如果这个键在map中不存在，它将自动创建一个新的键值对。接着，将字符串 "Initializing" 赋值给这个键对应的值。


