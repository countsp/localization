## Ros2
#### 创建 service
```auto service = this->create_service<ServiceType>(
    "service_name",
    std::bind(&ClassName::serviceCallback, this, _1, _2));
```
