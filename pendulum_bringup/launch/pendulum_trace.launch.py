# 版权声明
# 该文件基于Apache License 2.0发布，您可以在符合许可协议的前提下使用该文件。
# 您可以在以下链接获取许可协议的副本：
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# 除非适用法律要求或书面同意，否则软件根据"AS IS"的方式分发，
# 不提供任何明示或暗示的担保或条件。
# 有关许可下的特定语言的管理权限，请参阅许可协议。

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from tracetools_launch.action import Trace


def generate_launch_description():
    """
    生成 LaunchDescription 对象，用于启动一组相关的节点。
    """
    # 创建 Trace 对象，用于追踪
    ros_tracing = Trace(
        session_name='pendulum',
        events_kernel=[]
    )

    # 获取 bringup 目录路径
    bringup_dir = FindPackageShare('pendulum_bringup').find('pendulum_bringup')
    # 包含另一个 launch 文件
    pendulum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/pendulum_bringup.launch.py'])
    )

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()
    # 添加追踪操作
    ld.add_action(ros_tracing)
    # 添加包含的 launch 文件
    ld.add_action(pendulum_launch)

    return ld
