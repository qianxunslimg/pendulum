// 这段代码实现了一个名为PendulumDriverNode的ROS 2节点，它的作用是控制一个摆杆的运动。具体来说，这个节点的功能包括：

// 发布摆杆的状态信息：节点创建了一个状态发布器(state_pub_)，定期发布摆杆的位置、速度、角度和角速度等状态信息。

// 接收控制命令：节点创建了一个控制命令订阅器(command_sub_)，用于接收外部发送的控制命令，例如期望摆杆的位置或施加的力。

// 接收干扰力信息：节点创建了一个干扰力订阅器(disturbance_sub_)，用于接收干扰力的信息，以模拟外部环境对摆杆的干扰。

// 实现生命周期管理：节点实现了生命周期接口，包括配置、激活、去激活、清理和关闭等状态的管理。这样可以在节点的不同生命周期阶段执行相应的操作，例如在激活状态时启动状态发布器，去激活状态时停止发布器并记录驱动器的状态。

// 总体而言，这段代码实现了一个摆杆驱动节点，用于控制摆杆的运动，并提供了状态发布、控制命令接收和干扰力接收等功能，以实现对摆杆的控制和监控。

#include <string>
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "pendulum_driver/pendulum_driver_node.hpp"

namespace pendulum
{
  namespace pendulum_driver
  {
    PendulumDriverNode::PendulumDriverNode(const rclcpp::NodeOptions &options)
        : PendulumDriverNode("pendulum_driver", options)
    {
    }

    PendulumDriverNode::PendulumDriverNode(
        const std::string &node_name,
        const rclcpp::NodeOptions &options)
        : LifecycleNode(node_name, options),
          state_topic_name_(declare_parameter("state_topic_name").get<std::string>()),
          command_topic_name_(declare_parameter("command_topic_name").get<std::string>()),
          disturbance_topic_name_(declare_parameter("disturbance_topic_name").get<std::string>()),
          cart_base_joint_name_(declare_parameter("cart_base_joint_name").get<std::string>()),
          pole_joint_name_(declare_parameter("pole_joint_name").get<std::string>()),
          state_publish_period_(std::chrono::microseconds{
              declare_parameter("state_publish_period_us").get<std::uint16_t>()}),
          enable_topic_stats_(declare_parameter("enable_topic_stats").get<bool>()),
          topic_stats_topic_name_{declare_parameter("topic_stats_topic_name").get<std::string>()},
          topic_stats_publish_period_{std::chrono::milliseconds{
              declare_parameter("topic_stats_publish_period_ms").get<std::uint16_t>()}},
          deadline_duration_{std::chrono::milliseconds{
              declare_parameter("deadline_duration_ms").get<std::uint16_t>()}},
          driver_(
              PendulumDriver::Config(
                  declare_parameter("driver.pendulum_mass").get<double>(),
                  declare_parameter("driver.cart_mass").get<double>(),
                  declare_parameter("driver.pendulum_length").get<double>(),
                  declare_parameter("driver.damping_coefficient").get<double>(),
                  declare_parameter("driver.gravity").get<double>(),
                  declare_parameter("driver.max_cart_force").get<double>(),
                  declare_parameter("driver.noise_level").get<double>(),
                  std::chrono::microseconds{state_publish_period_})),
          num_missed_deadlines_pub_{0U},
          num_missed_deadlines_sub_{0U}
    {
      // 初始化状态消息
      init_state_message();
      // 创建状态发布器
      create_state_publisher();
      // 创建控制命令订阅器
      create_command_subscription();
      // 创建干扰力订阅器
      create_disturbance_subscription();
      // 创建状态定时器回调
      create_state_timer_callback();
    }

    // 初始化状态消息
    void PendulumDriverNode::init_state_message()
    {
      state_message_.cart_position = 0.0;
      state_message_.cart_velocity = 0.0;
      state_message_.cart_force = 0.0;
      state_message_.pole_angle = 0.0;
      state_message_.pole_velocity = 0.0;
    }

    // 创建状态发布器
    void PendulumDriverNode::create_state_publisher()
    {
      rclcpp::PublisherOptions sensor_publisher_options;
      // 设置发布器的截止期回调函数，用于统计未满足截止期的消息数量
      sensor_publisher_options.event_callbacks.deadline_callback =
          [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
      {
        num_missed_deadlines_pub_++;
      };
      // 创建状态发布器，并设置截止期、选项和策略
      state_pub_ = this->create_publisher<pendulum2_msgs::msg::JointState>(
          state_topic_name_,
          rclcpp::QoS(10).deadline(deadline_duration_),
          sensor_publisher_options);
    }

    // 创建控制命令订阅器
    void PendulumDriverNode::create_command_subscription()
    {
      // 预先分配消息池中的消息
      using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
      using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
      auto command_msg_strategy =
          std::make_shared<MessagePoolMemoryStrategy<pendulum2_msgs::msg::JointCommand, 1>>();

      rclcpp::SubscriptionOptions command_subscription_options;
      // 设置订阅器的截止期回调函数，用于统计未满足截止期的消息数量
      command_subscription_options.event_callbacks.deadline_callback =
          [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
      {
        num_missed_deadlines_sub_++;
      };
      if (enable_topic_stats_)
      {
        // 启用主题统计信息
        command_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
        command_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
        command_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
      }
      // 设置控制命令接收回调函数
      auto on_command_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg)
      {
        driver_.set_controller_cart_force(msg->force);
      };
      // 创建控制命令订阅器，并设置截止期、选项、回调函数和策略
      command_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
          command_topic_name_,
          rclcpp::QoS(10).deadline(deadline_duration_),
          on_command_received,
          command_subscription_options,
          command_msg_strategy);
    }

    // 创建干扰力订阅器
    void PendulumDriverNode::create_disturbance_subscription()
    {
      // 设置干扰力接收回调函数
      auto on_disturbance_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg)
      {
        driver_.set_disturbance_force(msg->force);
      };
      // 创建干扰力订阅器
      disturbance_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
          disturbance_topic_name_, rclcpp::QoS(10), on_disturbance_received);
    }

    // 创建状态定时器回调
    void PendulumDriverNode::create_state_timer_callback()
    {
      auto state_timer_callback = [this]()
      {
        driver_.update();
        const auto state = driver_.get_state();
        state_message_.cart_position = state.cart_position;
        state_message_.cart_velocity = state.cart_velocity;
        state_message_.cart_force = state.cart_force;
        state_message_.pole_angle = state.pole_angle;
        state_message_.pole_velocity = state.pole_velocity;
        // 发布状态消息
        state_pub_->publish(state_message_);
      };
      // 创建状态定时器，并设置回调函数
      state_timer_ = this->create_wall_timer(state_publish_period_, state_timer_callback);
      // 立即取消定时器以防止在此状态下触发它
      state_timer_->cancel();
    }

    // 记录驱动器状态
    void PendulumDriverNode::log_driver_state()
    {
      const auto state = driver_.get_state();
      const auto disturbance_force = driver_.get_disturbance_force();
      const double controller_force_command = driver_.get_controller_cart_force();

      RCLCPP_INFO(get_logger(), "Cart position = %lf", state.cart_position);
      RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.cart_velocity);
      RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.pole_angle);
      RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
      RCLCPP_INFO(get_logger(), "Controller force command = %lf", controller_force_command);
      RCLCPP_INFO(get_logger(), "Disturbance force = %lf", disturbance_force);
      RCLCPP_INFO(get_logger(), "Publisher missed deadlines = %lu", num_missed_deadlines_pub_);
      RCLCPP_INFO(get_logger(), "Subscription missed deadlines = %lu", num_missed_deadlines_sub_);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PendulumDriverNode::on_configure(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(get_logger(), "Configuring");
      // reset internal state of the driver for a clean start
      driver_.reset();
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PendulumDriverNode::on_activate(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(get_logger(), "Activating");
      state_pub_->on_activate();
      state_timer_->reset();
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(get_logger(), "Deactivating");
      state_timer_->cancel();
      state_pub_->on_deactivate();
      // log the status to introspect the result
      log_driver_state();
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(get_logger(), "Cleaning up");
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PendulumDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(get_logger(), "Shutting down");
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
  } // namespace pendulum_driver
} // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::pendulum_driver::PendulumDriverNode)
