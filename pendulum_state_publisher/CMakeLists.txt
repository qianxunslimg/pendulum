cmake_minimum_required(VERSION 3.5)
project(pendulum_state_publisher)

# 默认使用C++14标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 如果使用GNU或Clang编译器，添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找所需的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pendulum2_msgs REQUIRED)

# 添加可执行文件
add_executable(pendulum_state_publisher pendulum_state_publisher.cpp)

# 添加目标文件的依赖关系
ament_target_dependencies(pendulum_state_publisher rclcpp pendulum2_msgs sensor_msgs)

# 安装目标文件到指定目录
install(TARGETS
  pendulum_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 如果构建测试，找到并添加测试依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成ament包
ament_package()
