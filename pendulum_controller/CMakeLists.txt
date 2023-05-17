cmake_minimum_required(VERSION 3.5)

project(pendulum_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pendulum2_msgs REQUIRED)
find_package(pendulum_utils REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    pendulum2_msgs
    pendulum_utils)

set(PENDULUM_CONTROLLER_LIB pendulum_controller)
add_library(${PENDULUM_CONTROLLER_LIB} SHARED
        src/pendulum_controller_node.cpp
        src/pendulum_controller.cpp)

target_include_directories(${PENDULUM_CONTROLLER_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PENDULUM_CONTROLLER_LIB} ${dependencies})

rclcpp_components_register_nodes(${PENDULUM_CONTROLLER_LIB} "pendulum::Controller")

# Generate standalone node executable
set(PENDULUM_CONTROLLER_EXE "${PENDULUM_CONTROLLER_LIB}_exe")
add_executable(${PENDULUM_CONTROLLER_EXE} src/pendulum_controller_node_main.cpp)
target_link_libraries(${PENDULUM_CONTROLLER_EXE} ${PENDULUM_CONTROLLER_LIB})

ament_export_targets(export_${PENDULUM_CONTROLLER_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# 判断是否开启了测试选项，如果是，则执行以下代码
if(BUILD_TESTING)
  # 寻找apex_test_tools软件包，并将其包含到当前项目中
  find_package(apex_test_tools REQUIRED)
  # 寻找ament_cmake_gtest软件包，并将其包含到当前项目中
  find_package(ament_cmake_gtest REQUIRED)
  # 寻找ament_lint_auto软件包，并将其包含到当前项目中
  find_package(ament_lint_auto REQUIRED)
  # 调用ament_lint_auto_find_test_dependencies函数，自动寻找测试所需的依赖包
  ament_lint_auto_find_test_dependencies()

  # 调用apex_test_tools_add_gtest函数，添加一个名为test_pendulum_controller的gtest测试目标，指定其源码文件为test/test_pendulum_controller.cpp
  apex_test_tools_add_gtest(test_pendulum_controller test/test_pendulum_controller.cpp)
  # 判断是否创建了test_pendulum_controller目标，如果是，则将PENDULUM_CONTROLLER_LIB库链接到该目标上
  if(TARGET test_pendulum_controller)
    target_link_libraries(test_pendulum_controller ${PENDULUM_CONTROLLER_LIB})
  endif()
  # 调用apex_test_tools_add_gtest函数，添加一个名为test_pendulum_controller_node的gtest测试目标，指定其源码文件为test/test_pendulum_controller_node.cpp
  apex_test_tools_add_gtest(test_pendulum_controller_node test/test_pendulum_controller_node.cpp)
  # 判断是否创建了test_pendulum_controller_node目标，如果是，则将PENDULUM_CONTROLLER_LIB库链接到该目标上
  if(TARGET test_pendulum_controller_node)
    target_link_libraries(test_pendulum_controller_node ${PENDULUM_CONTROLLER_LIB})
  endif()

  # 寻找launch_testing_ament_cmake软件包，并将其包含到当前项目中
  find_package(launch_testing_ament_cmake)
  # 调用add_launch_test函数，添加一个名为pendulum_controller_lifecycle.test的launch测试目标，指定其launch文件为test/pendulum_controller_lifecycle.test.py，并设置环境变量和超时时间
  add_launch_test(
      test/pendulum_controller_lifecycle.test.py
      TARGET pendulum_controller_lifecycle.test
      ENV
      TIMEOUT 30
  )
  # 调用add_launch_test函数，添加一个名为pendulum_controller_autostart.test的launch测试目标，指定其launch文件为test/pendulum_controller_autostart.test.py，并设置环境变量和超时时间
  add_launch_test(
      test/pendulum_controller_autostart.test.py
      TARGET pendulum_controller_autostart.test
      ENV
      TIMEOUT 30
  )
endif()

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS ${PENDULUM_CONTROLLER_LIB} ${PENDULUM_CONTROLLER_EXE}
    EXPORT export_${PENDULUM_CONTROLLER_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()
