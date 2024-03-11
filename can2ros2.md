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

cd ..

colcon build

source install/setup.bash

ros2 run my_imu_publisher imu_publisher
