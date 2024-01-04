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
  
  ```
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ```

---

### ::ros::NodeHandle::subscribe()

```
template<class M, class T>
ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, 
                          const ros::TransportHints& transport_hints = ros::TransportHints());
```

M 是消息类型，例如 std_msgs::String、sensor_msgs::Image 等。

topic 是你想订阅的话题名称。

queue_size 是消息队列的大小。这个参数用于定义在处理消息之前可以缓存多少条消息。

fp 是一个指向成员函数的指针，该函数会被调用来处理接收到的消息。这个函数的类型必须匹配消息类型。

obj 是一个指向成员函数所属对象的指针。

transport_hints 是可选的，用于配置底层的消息传输，如 TCP、UDP。


例子

```
#include <ros/ros.h>
#include <std_msgs/String.h>

class MyNode {
public:
    void stringCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received: %s", msg->data.c_str());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    MyNode myNode;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, &MyNode::stringCallback, &myNode);

    ros::spin();

    return 0;
}
```
---

### advertiseService()
在 ROS (Robot Operating System) 中，advertiseService() 方法用于创建一个服务服务器（service server）。这个方法使你的节点能夠响应来自客户端（service clients）的服务请求。
以下是 advertiseService() 方法的基本语法：

```
ros::ServiceServer ros::NodeHandle::advertiseService(
  const std::string& service, 
  bool(ServiceClass::*srv_func)(RequestType&, ResponseType&),
  ServiceClass* obj
);
```

> service: 服务的名称。
> 
> srv_func: 服务处理函数，当服务请求到达时，这个函数被调用。
> 
> obj: 服务处理函数所属对象的指针，通常是 this

**服务处理函数**

服务处理函数需要有特定的签名，它接受一个请求（RequestType）和一个响应（ResponseType），并返回一个布尔值。如果处理成功，返回 true；如果失败，则返回 false。

```
#include "ros/ros.h"
#include "std_srvs/Empty.h"

class MyServiceClass
{
public:
  bool myServiceCallback(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
  {
    ROS_INFO("Service called");
    // 这里处理服务请求
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_service_node");
  ros::NodeHandle nh;

  MyServiceClass my_service_obj;

  ros::ServiceServer service = nh.advertiseService("my_service", &MyServiceClass::myServiceCallback, &my_service_obj);

  ros::spin();

  return 0;
}

```
---

### createwalltimer()
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
```

---
### Rigid
**Rigid2：**

    使用 Eigen::Matrix<FloatType, 2, 1> 类型来表示二维空间中的平移向量。
    
    Rigid2 类使用 Eigen::Rotation2D<FloatType> 来表示二维空间中的旋转。
    
    这实际上是一个二维的向量，用于存储在二维空间（如平面）中的 x 和 y 坐标。
    
**Rigid3：**
    使用 Eigen::Matrix<FloatType, 3, 1> 类型来表示三维空间中的平移向量。这是一个三维的向量，用于存储在三维空间中的 x、y 和 z 坐标。
    
    Rigid3 类使用 Eigen::Quaternion<FloatType> 来表示三维空间中的旋转。
    
    四元数是一种常用于三维旋转的数学工具，因为它避免了万向节锁的问题，并且计算效率较高。
