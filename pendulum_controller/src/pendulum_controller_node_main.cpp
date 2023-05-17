// Copyright 2020 Carlos San Vicente
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

// 定义主函数，接收命令行参数
int main(int argc, char *argv[])
{
  // 创建一个ProcessSettings对象，用于配置进程的实时设置
  pendulum::utils::ProcessSettings settings;
  // 调用init方法，根据命令行参数初始化设置，如果失败则返回退出码
  if (!settings.init(argc, argv))
  {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try
  {
    // 如果设置了configure_child_threads为true，则调用configure_process方法，
    // 配置进程的实时设置，这样进程创建的子线程（如ROS节点）会继承这些设置
    if (settings.configure_child_threads)
    {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }
    // 调用rclcpp::init方法，初始化ROS客户端库
    rclcpp::init(argc, argv);

    // 创建一个静态单线程执行器对象
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // 创建一个摆动控制器节点对象，传入节点名称为"pendulum_controller"
    using pendulum::pendulum_controller::PendulumControllerNode;
    const auto controller_node_ptr =
        std::make_shared<PendulumControllerNode>("pendulum_controller");

    // 调用add_node方法，将节点添加到执行器中
    exec.add_node(controller_node_ptr->get_node_base_interface());

    // 如果设置了configure_child_threads为false，则调用configure_process方法，
    // 配置进程的实时设置，这样进程创建的子线程（如ROS节点）不会继承这些设置
    if (!settings.configure_child_threads)
    {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    // 如果设置了auto_start_nodes为true，则调用autostart函数，
    // 将节点从Unconfigured状态转换到Active状态，开始执行功能
    if (settings.auto_start_nodes)
    {
      pendulum::utils::autostart(*controller_node_ptr);
    }

    // 调用spin方法，让执行器开始运行节点
    exec.spin();
    // 调用rclcpp::shutdown方法，关闭ROS客户端库
    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    // 捕获并打印任何标准异常，并返回错误码2
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), e.what());
    ret = 2;
  }
  catch (...)
  {
    // 捕获并打印任何未知异常，并返回错误码-1
    RCLCPP_INFO(
        rclcpp::get_logger("pendulum_demo"), "Unknown exception caught. "
                                             "Exiting...");
    ret = -1;
  }
  // 返回退出码
  return ret;
}
