### ::ros::NodeHandle的成员函数

`::ros::NodeHandle` 类是 ROS（Robot Operating System）中的一个核心类，用于交互操作和节点的管理。以下是一些较为常用的成员函数：

#### 订阅和发布
- **subscribe()**：订阅一个话题。
- **advertise()**：在一个话题上发布消息。

#### 服务
- **serviceClient()**：创建一个服务客户端，用于调用服务。
- **advertiseService()**：创建一个服务服务器，提供服务。

#### 参数服务器
- **getParam()**：从参数服务器获取一个参数。
- **setParam()**：在参数服务器上设置一个参数。
- **deleteParam()**：删除参数服务器上的一个参数。
- **param()**：获取参数值，如果参数不存在，则返回一个默认值。

#### 命名空间和节点信息
- **getNamespace()**：获取节点的命名空间。
- **resolveName()**：解析一个名称，考虑命名空间和相对名称。

#### 定时器
- **createTimer()**：创建一个定时器，用于定期执行回调。

#### 其他
- **ok()**：检查 ROS 是否仍在正常运行。
- **shutdown()**：关闭节点句柄。

---

### ::ros::NodeHandle::advertise()和ros::Publisher的区别

在 ROS（Robot Operating System）中，`advertise()` 方法和 `ros::Publisher` 类在发布消息时发挥着不同的作用。它们之间的区别主要在于它们的职责和如何在代码中使用。

#### `advertise()` 方法
- `advertise()` 是 `ros::NodeHandle` 类的一个成员方法，用于创建一个消息发布者（`ros::Publisher`）。
- 当你调用 `advertise()` 时，你需要指定话题名称、队列大小和（可选的）回调函数。
- 这个方法返回一个 `ros::Publisher` 对象，你可以使用这个对象来发布消息。
- `advertise()` 方法主要用于设置发布者的参数并初始化发布者。

  示例：
  ```cpp
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);

---
# createwalltimer()函数功能
创建一个定时器（timer），它按照设定的时间间隔周期性地调用一个回调函数。这个功能在需要定期执行任务（如定期发送消息、更新状态或执行周期性检查）的场景中非常有用。
```
#include <ros/ros.h>

void timerCallback(const ros::WallTimerEvent& event) {
    // 执行定时任务
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    // 创建一个定时器，每秒钟触发一次
    ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(1.0), timerCallback);

    ros::spin();

    return 0;
}
