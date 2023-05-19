// 这段代码实现了一个主函数，用于启动摆杆驱动节点。具体功能和实现方式如下：

// 创建一个ProcessSettings对象并进行初始化，该对象用于处理命令行参数。

// 如果初始化失败，返回EXIT_FAILURE。

// 配置进程的实时设置，如果设置为true，子线程将继承这些设置。

// 初始化ROS 2节点。

// 创建一个静态的单线程执行器(exec)。

// 使用PendulumDriverNode创建一个摆杆驱动节点对象(driver_node_ptr)。

// 将驱动节点添加到执行器中。

// 配置进程的实时设置，如果设置为false，子线程将不继承这些设置。

// 如果设置为自动启动节点(auto_start_nodes=true)，则调用pendulum::utils::autostart函数自动启动驱动节点。

// 执行器开始执行spin()，进入循环等待状态。

// 当spin()循环结束后，调用rclcpp::shutdown()关闭ROS 2节点。

// 如果捕获到std::exception异常，将异常信息记录到日志并将返回值ret设置为2。

// 如果捕获到其他未知异常，记录相应的错误信息到日志并将返回值ret设置为-1。

// 返回最终的返回值ret。

// 总体而言，这段代码实现了一个摆杆驱动节点的启动器，用于初始化并启动ROS 2节点，创建摆杆驱动节点对象，并通过执行器进行执行。代码通过使用ProcessSettings对象处理命令行参数，配置进程的实时设置，创建执行器和驱动节点对象，最后通过spin()进入循环等待状态，实现了摆杆驱动节点的启动和执行。

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

int main(int argc, char *argv[])
{
  pendulum::utils::ProcessSettings settings;
  if (!settings.init(argc, argv))
  {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try
  {
    // 配置进程的实时设置
    if (settings.configure_child_threads)
    {
      // 由ROS节点创建的子线程将继承这些设置
      settings.configure_process();
    }
    rclcpp::init(argc, argv);

    // 创建静态执行器
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // 创建摆杆仿真
    using pendulum::pendulum_driver::PendulumDriverNode;
    const auto driver_node_ptr = std::make_shared<PendulumDriverNode>("pendulum_driver");

    exec.add_node(driver_node_ptr->get_node_base_interface());

    // 配置进程的实时设置
    if (!settings.configure_child_threads)
    {
      // 由ROS节点创建的子线程将不继承这些设置
      settings.configure_process();
    }

    if (settings.auto_start_nodes)
    {
      pendulum::utils::autostart(*driver_node_ptr);
    }

    exec.spin();
    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_driver"), e.what());
    ret = 2;
  }
  catch (...)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("pendulum_driver"), "捕获到未知异常。"
                                               "正在退出...");
    ret = -1;
  }
  return ret;
}
