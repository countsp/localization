## Ros2
#### 创建 service ,
```
auto service = create_service<ServiceType>(
    "service_name",
    std::bind(&ClassName::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
```
    ServiceType：
        指定服务的类型。这个类型定义了服务的请求和响应消息的结构。例如，std_srvs::srv::SetBool是一个标准的服务类型，其请求包含一个布尔值，响应也包含一个布尔值。

    "service_name"：
        服务的名称。这是其他节点用来调用此服务的唯一标识符。

    回调方法：
        当服务接收到请求时，将调用的方法。这个方法需要与服务类型的请求和响应匹配。使用std::bind可以将类的成员函数作为回调函数绑定。std::placeholders::_1 和 _2 分别代表请求和响应的占位符。

    rmw_qos_profile (可选)：
        定义服务的质量（QoS）配置。这个参数是可选的，用于指定如何处理服务通信的可靠性、持久性等特性。
