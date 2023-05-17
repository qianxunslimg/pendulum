# Copyright 2019 Carlos San Vicente
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

# 导入os模块，用于操作系统相关的功能，如文件路径
import os

# 导入launch模块，用于创建和管理ROS 2启动过程
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
# 导入launch_ros模块，用于创建和管理ROS 2节点
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 定义一个函数，用于生成启动描述对象，该对象包含了启动过程中需要执行的动作和条件
def generate_launch_description():
    # 获取pendulum_bringup包的目录路径，该包包含了摆动机器人的启动文件和参数
    bringup_dir = FindPackageShare('pendulum_bringup').find('pendulum_bringup')

    # 设置机器人描述参数，从urdf文件中读取机器人的模型和关节信息，并存储在robot_desc变量中
    urdf_file = os.path.join(bringup_dir, 'urdf', 'pendulum.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # 设置参数文件的路径，从param_file_path变量中获取参数文件的绝对路径，并存储在param_file变量中
    param_file_path = os.path.join(bringup_dir, 'params', 'pendulum.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    # 设置rviz配置文件的路径，从rviz_cfg_path变量中获取rviz配置文件的绝对路径
    rviz_cfg_path = os.path.join(bringup_dir, 'rviz/pendulum.rviz')

    # 创建一些启动配置变量，用于设置启动过程中的一些选项，如自动启动、优先级、CPU亲和性、内存锁定等
    # 创建一个名为autostart的启动配置变量，用于设置是否自动启动生命周期节点，其默认值为True，即自动启动
    autostart_param = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically start lifecycle nodes')
    # 创建一个名为priority的启动配置变量，用于设置进程的优先级，其默认值为0，即不设置优先级
    priority_param = DeclareLaunchArgument(
        name='priority',
        default_value='0',
        description='Set process priority')
    # 创建一个名为cpu-affinity的启动配置变量，用于设置进程的CPU亲和性，其默认值为0，即不设置CPU亲和性
    cpu_affinity_param = DeclareLaunchArgument(
        name='cpu-affinity',
        default_value='0',
        description='Set process CPU affinity')
    # 创建一个名为lock-memory的启动配置变量，用于设置是否锁定进程的内存，其默认值为False，即不锁定内存
    with_lock_memory_param = DeclareLaunchArgument(
        name='lock-memory',
        default_value='False',
        description='Lock the process memory')
    # 创建一个名为lock-memory-size的启动配置变量，用于设置锁定内存的大小，单位为MB，其默认值为0，即不设置锁定内存大小
    lock_memory_size_param = DeclareLaunchArgument(
        name='lock-memory-size',
        default_value='0',
        description='Set lock memory size in MB')
    # 创建一个名为config-child-threads的启动配置变量，用于设置是否配置进程的子线程（通常是DDS线程），其默认值为False，即不配置子线程
    config_child_threads_param = DeclareLaunchArgument(
        name='config-child-threads',
        default_value='False',
        description='Configure process child threads (typically DDS threads)')
    # 创建一个名为rviz的启动配置变量，用于设置是否在其他节点之外启动RVIZ2，其默认值为False，即不启动RVIZ2
    with_rviz_param = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RVIZ2 in addition to other nodes'
    )

    # Node definitions
    # 定义一个pendulum_demo节点运行器，用于创建和运行一个pendulum_demo节点
    pendulum_demo_runner = Node(
        # 指定节点所属的包名，这里是pendulum_demo
        package='pendulum_demo',
        # 指定节点的可执行文件名，这里是pendulum_demo
        executable='pendulum_demo',
        # 指定节点的输出方式，这里是在屏幕上显示
        output='screen',
        # 指定节点的参数文件，这里是从param_file变量中获取
        parameters=[param_file],
        # 指定节点的命令行参数，这里是从启动配置变量中获取一些选项，如自动启动、优先级、CPU亲和性等
        arguments=[
           '--autostart', LaunchConfiguration('autostart'),
           '--priority', LaunchConfiguration('priority'),
           '--cpu-affinity', LaunchConfiguration('cpu-affinity'),
           '--lock-memory', LaunchConfiguration('lock-memory'),
           '--lock-memory-size', LaunchConfiguration('lock-memory-size'),
           '--config-child-threads', LaunchConfiguration('config-child-threads')
           ]
    )

    # 定义一个robot_state_publisher节点运行器，用于创建和运行一个robot_state_publisher节点
    robot_state_publisher_runner = Node(
        # 指定节点所属的包名，这里是robot_state_publisher
        package='robot_state_publisher',
        # 指定节点的可执行文件名，这里是robot_state_publisher
        executable='robot_state_publisher',
        # 指定节点的输出方式，这里是在屏幕上显示
        output='screen',
        # 指定节点的参数，这里是从rsp_params变量中获取机器人描述参数
        parameters=[rsp_params],
        # 指定节点的启动条件，这里是根据rviz启动配置变量来判断是否启动该节点
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 定义一个rviz2节点运行器，用于创建和运行一个rviz2节点
    rviz_runner = Node(
        # 指定节点所属的包名，这里是rviz2
        package='rviz2',
        # 指定节点的可执行文件名，这里是rviz2
        executable='rviz2',
        # 指定节点的名称，这里是rviz2
        name='rviz2',
        # 指定节点的命令行参数，这里是指定rviz配置文件的路径
        arguments=['-d', str(rviz_cfg_path)],
        # 指定节点的启动条件，这里是根据rviz启动配置变量来判断是否启动该节点
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 定义一个pendulum_state_publisher节点运行器，用于创建和运行一个pendulum_state_publisher节点
    pendulum_state_publisher_runner = Node(
        # 指定节点所属的包名，这里是pendulum_state_publisher
        package='pendulum_state_publisher',
        # 指定节点的可执行文件名，这里是pendulum_state_publisher
        executable='pendulum_state_publisher',
        # 指定节点的启动条件，这里是根据rviz启动配置变量来判断是否启动该节点
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 创建一个启动描述对象，用于存储启动过程中需要执行的动作和条件
    ld = LaunchDescription()

    # 向启动描述对象中添加启动配置变量，这些变量可以在启动时通过命令行参数进行修改
    ld.add_action(autostart_param)
    ld.add_action(priority_param)
    ld.add_action(cpu_affinity_param)
    ld.add_action(with_lock_memory_param)
    ld.add_action(lock_memory_size_param)
    ld.add_action(config_child_threads_param)
    ld.add_action(with_rviz_param)
    # 向启动描述对象中添加节点运行器，这些运行器可以创建和管理ROS 2节点，如机器人状态发布器、摆动演示器、RVIZ2和摆动状态发布器
    ld.add_action(robot_state_publisher_runner)
    ld.add_action(pendulum_demo_runner)
    ld.add_action(rviz_runner)
    ld.add_action(pendulum_state_publisher_runner)

    # 返回启动描述对象，供其他模块使用
    return ld
