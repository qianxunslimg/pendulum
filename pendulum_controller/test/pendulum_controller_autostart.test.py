# Copyright 2020 Carlos San Vicente
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 导入os模块，用于操作系统相关的功能
import os
# 导入signal模块，用于处理信号
import signal

# 导入unittest模块，用于编写单元测试
import unittest

# 导入launch模块，用于启动ROS 2应用程序
import launch
# 导入launch_ros模块，用于启动ROS 2节点和事件
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

# 导入launch_ros.substitutions模块，用于获取ROS 2包的路径
from launch_ros.substitutions import FindPackageShare

# 导入launch_testing模块，用于编写集成测试
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

# 导入pytest模块，用于运行测试
import pytest


# 使用pytest装饰器，标记这个函数是一个rostest
@pytest.mark.rostest
# 定义一个函数，生成测试描述，返回一个LaunchDescription对象和一个字典对象
def generate_test_description():
    # 获取pendulum_controller包的路径，并拼接出参数文件的路径
    package_dir = FindPackageShare('pendulum_controller').find('pendulum_controller')
    param_file_path = os.path.join(package_dir, 'params', 'test.param.yaml')
    # 创建一个LaunchConfiguration对象，用于存储参数文件的路径
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    # 创建一个LifecycleNode对象，用于启动pendulum_controller节点，设置其包名、可执行文件名、节点名、参数、参数、额外的环境变量等属性
    controller_node = launch_ros.actions.LifecycleNode(
        package='pendulum_controller',
        executable='pendulum_controller_exe',
        name='pendulum_controller',
        parameters=[param_file],
        arguments=['--autostart', 'True'],
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    # 创建一个TimerAction对象，用于在5秒后发送一个SIGINT信号给pendulum_controller节点，让其关闭
    shutdown_timer = launch.actions.TimerAction(
        period=5.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc is controller_node
                )
            )
        ]
    )

    # 返回一个LaunchDescription对象，包含了启动pendulum_controller节点、定时关闭节点、保持进程活跃、准备测试的动作
    return (
        launch.LaunchDescription([
            controller_node,
            shutdown_timer,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]),
        # 返回一个字典对象，包含了pendulum_controller节点的引用，方便后续的测试使用
        {
            'controller_node': controller_node,
        }
    )


# 定义一个TestControllerNode类，继承自unittest.TestCase，用于测试pendulum_controller节点的启动和关闭
class TestControllerNode(unittest.TestCase):

    # 定义一个test_proc_starts方法，接收proc_info和controller_node作为参数，用于测试pendulum_controller节点是否成功启动
    def test_proc_starts(self, proc_info, controller_node):
        # 使用proc_info对象的assertWaitForStartup方法，等待pendulum_controller节点启动，设置超时时间为5秒
        proc_info.assertWaitForStartup(process=controller_node, timeout=5)

    # 定义一个test_proc_terminates方法，接收proc_info和controller_node作为参数，用于测试pendulum_controller节点是否成功关闭
    def test_proc_terminates(self, proc_info, controller_node):
        # 使用proc_info对象的assertWaitForShutdown方法，等待pendulum_controller节点关闭，设置超时时间为10秒
        proc_info.assertWaitForShutdown(controller_node, timeout=10)


# 使用launch_testing.post_shutdown_test装饰器，标记这个类是一个后置测试，即在所有进程关闭后运行
@launch_testing.post_shutdown_test()
# 定义一个TestShutdown类，继承自unittest.TestCase，用于测试pendulum_controller节点的优雅关闭
class TestShutdown(unittest.TestCase):

    # 定义一个test_node_graceful_shutdown方法，接收proc_info和controller_node作为参数，用于测试pendulum_controller节点是否优雅关闭
    def test_node_graceful_shutdown(self, proc_info, controller_node):
        """Test controller_node graceful shutdown."""
        # 使用launch_testing.asserts模块的assertExitCodes方法，断言pendulum_controller节点的退出码是否正常
        launch_testing.asserts.assertExitCodes(proc_info, process=controller_node)

