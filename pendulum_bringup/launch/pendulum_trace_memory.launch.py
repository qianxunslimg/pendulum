# 导入所需的模块
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# 定义一个函数，用于生成启动描述对象，该对象包含了启动过程中需要执行的动作和条件
def generate_launch_description():
    # 创建一个Trace动作，用于设置追踪会话的名称和事件列表
    ros_tracing_memory_usage = Trace(
        session_name='pendulum-memory-usage',
        events_ust=[
            'lttng_ust_libc:malloc',
            'lttng_ust_libc:calloc',
            'lttng_ust_libc:realloc',
            'lttng_ust_libc:free',
            'lttng_ust_libc:memalign',
            'lttng_ust_libc:posix_memalign',
        ] + DEFAULT_EVENTS_ROS,
        events_kernel=[
            'kmem_mm_page_alloc',
            'kmem_mm_page_free',
        ]
    )

    # 获取pendulum_bringup包的目录路径
    bringup_dir = FindPackageShare('pendulum_bringup').find('pendulum_bringup')
    # 创建一个IncludeLaunchDescription动作，用于包含pendulum_bringup包中的启动文件
    pendulum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/pendulum_bringup.launch.py'])
    )

    # 创建一个启动描述对象，用于存储启动过程中需要执行的动作和条件
    ld = LaunchDescription()
    # 向启动描述对象中添加Trace动作，用于追踪内存使用情况
    ld.add_action(ros_tracing_memory_usage)
    # 向启动描述对象中添加IncludeLaunchDescription动作，用于包含pendulum_bringup包中的启动文件
    ld.add_action(pendulum_launch)

    # 返回启动描述对象，供其他模块使用
    return ld
