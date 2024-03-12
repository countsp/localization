# 新建一个imu发布者
ros2 pkg create my_imu_publisher --build-type ament_cmake --dependencies rclcpp sensor_msgs

cd my_imu_publisher/src/

gedit imu_publisher.cpp

```
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher() : Node("imu_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            [this]() { publishIMUData(); });
    }

private:
    void publishIMUData()
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_frame";

        // 假设一些示例数据
        imu_msg.orientation.x = 1.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 0.0;

        imu_msg.angular_velocity.x = 0.1;
        imu_msg.angular_velocity.y = 0.2;
        imu_msg.angular_velocity.z = 0.3;

        imu_msg.linear_acceleration.x = 0.4;
        imu_msg.linear_acceleration.y = 0.5;
        imu_msg.linear_acceleration.z = 0.6;

        publisher_->publish(imu_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```

修改CMakeList.txt

```
cmake_minimum_required(VERSION 3.8)
project(my_imu_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare a C++ executable
add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)

# Install the executable
install(TARGETS
  imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

cd ..

colcon build

source install/setup.bash

ros2 run my_imu_publisher imu_publisher

# 新建一个imu接受者
在my_imu_publisher/src/目录下创建一个新的C++文件，例如命名为imu_subscriber.cpp
```
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUSubscriber : public rclcpp::Node
{
public:
    IMUSubscriber() : Node("imu_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&IMUSubscriber::imuCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data_modified", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 修改IMU数据
        auto modified_msg = *msg;
        modified_msg.orientation.x += 1.0;
        modified_msg.orientation.y += 1.0;
        modified_msg.orientation.z += 1.0;
        modified_msg.orientation.w += 1.0;

        // 可以在此处增加其他字段的修改

        // 发布修改后的IMU数据
        publisher_->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
修改CMakeList.txt
```
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUSubscriber : public rclcpp::Node
{
public:
    IMUSubscriber() : Node("imu_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&IMUSubscriber::imuCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data_modified", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 修改IMU数据
        auto modified_msg = *msg;
        modified_msg.orientation.x += 1.0;
        modified_msg.orientation.y += 1.0;
        modified_msg.orientation.z += 1.0;
        modified_msg.orientation.w += 1.0;

        // 可以在此处增加其他字段的修改

        // 发布修改后的IMU数据
        publisher_->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
colcon build --packages-select my_imu_publisher或者colcon build 重新编译

# launch文件同时调起两个
cd my_imu_publisher 

mkdir launch

gedit imu_launch.py

```
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='my_imu_publisher',
            executable='imu_publisher',
            name='imu_publisher'
        ),
        Node(
            package='my_imu_publisher',
            executable='imu_subscriber',
            name='imu_subscriber'
        )
    ])
```

打开你的my_imu_publisher包的CMakeLists.txt文件，并添加或确认以下行存在，以确保launch文件夹和其内容被安装到正确的位置：

```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```
重新colcon build

ros2 launch my_imu_publisher imu_launch.py
