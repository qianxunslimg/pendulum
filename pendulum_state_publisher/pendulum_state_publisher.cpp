// 引入必要的头文件
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pendulum2_msgs/msg/joint_state.hpp"

// 自定义类 PendulumStatePublisher 继承自 rclcpp::Node
class PendulumStatePublisher : public rclcpp::Node
{
public:
  PendulumStatePublisher()
  : Node("pendulum_state_publisher")
  {
    // 初始化关节状态消息的初始值
    state_message_.name.push_back("cart_base_joint");
    state_message_.position.push_back(0.0);
    state_message_.velocity.push_back(0.0);
    state_message_.effort.push_back(0.0);
    state_message_.name.push_back("pole_joint");
    state_message_.position.push_back(0.0);
    state_message_.velocity.push_back(0.0);
    state_message_.effort.push_back(0.0);

    // 创建发布器，发布 sensor_msgs::msg::JointState 类型的消息到 "joint_states" 主题
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 创建订阅器，订阅 pendulum2_msgs::msg::JointState 类型的消息从 "pendulum_joint_states" 主题
    // 当接收到消息时，回调函数会更新关节状态消息并发布出去
    subscription_ = this->create_subscription<pendulum2_msgs::msg::JointState>(
      "pendulum_joint_states",
      10,
      [this](pendulum2_msgs::msg::JointState::UniquePtr msg) {
        state_message_.position[0] = msg->cart_position;
        state_message_.velocity[0] = msg->cart_velocity;
        state_message_.effort[0] = msg->cart_force;
        state_message_.position[1] = msg->pole_angle;
        state_message_.velocity[1] = msg->pole_velocity;
        state_message_.header.stamp = this->get_clock()->now();
        this->publisher_->publish(state_message_);
      });
  }

private:
  sensor_msgs::msg::JointState state_message_;  // 关节状态消息
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;  // 发布器
  rclcpp::Subscription<pendulum2_msgs::msg::JointState>::SharedPtr subscription_;  // 订阅器
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 创建 PendulumStatePublisher 对象并将其作为参数传递给 rclcpp::spin()
  // rclcpp::spin() 会开始事件循环，处理节点的回调函数，直到节点被关闭
  rclcpp::spin(std::make_shared<PendulumStatePublisher>());

  rclcpp::shutdown();  // 关闭 ROS 2
  return 0;
}
