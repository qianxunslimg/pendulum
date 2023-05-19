// 版权声明
// 本代码使用 Apache License, Version 2.0 授权许可；
// 除非符合许可证要求，否则不得使用本文件；
// 可在以下网址获取许可证的副本：
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 适用法律要求或书面同意，本软件分发基于“原样”提供，
// 不提供任何明示或暗示的保证或条件。
// 请参阅许可证了解限制详情。

#include "pendulum_controller/pendulum_controller.hpp"
#include <utility>
#include <vector>

namespace pendulum
{
  namespace pendulum_controller
  {
    // PendulumController 类的 Config 结构体构造函数
    PendulumController::Config::Config(std::vector<double> feedback_matrix)
        : feedback_matrix{std::move(feedback_matrix)} {}

    // 获取 Config 结构体中的反馈矩阵
    const std::vector<double> &
    PendulumController::Config::get_feedback_matrix() const
    {
      return feedback_matrix;
    }

    // PendulumController 类的构造函数
    PendulumController::PendulumController(const Config &config)
        : cfg_(config),
          state_{0.0, 0.0, M_PI, 0.0},
          reference_{0.0, 0.0, M_PI, 0.0}
    {
    }

    // 重置控制器状态为默认状态
    void PendulumController::reset()
    {
      // 将控制器状态重置为竖直向上摆动的位置
      set_state(0.0, 0.0, M_PI, 0.0);
      set_teleop(0.0, 0.0, M_PI, 0.0);
    }

    // 更新控制器状态
    void PendulumController::update()
    {
      // 设置力命令为计算结果
      set_force_command(calculate(state_, reference_));
    }

    // 设置手动控制的参考状态
    void PendulumController::set_teleop(
        double cart_pos, double cart_vel,
        double pole_pos, double pole_vel)
    {
      reference_[0] = cart_pos;
      reference_[1] = cart_vel;
      reference_[2] = pole_pos;
      reference_[3] = pole_vel;
    }

    // 设置手动控制的参考状态（不包括摆杆的状态）
    void PendulumController::set_teleop(double cart_pos, double cart_vel)
    {
      reference_[0] = cart_pos;
      reference_[1] = cart_vel;
    }

    // 设置系统状态
    void PendulumController::set_state(
        double cart_pos, double cart_vel,
        double pole_pos, double pole_vel)
    {
      state_ = {cart_pos, cart_vel, pole_pos, pole_vel};
    }

    // 设置力命令
    void PendulumController::set_force_command(double force)
    {
      force_command_ = force;
    }

    // 获取手动控制的参考状态
    const std::vector<double> &PendulumController::get_teleop() const
    {
      return reference_;
    }

    // 获取系统状态
    const std::vector<double> &PendulumController::get_state() const
    {
      return state_;
    }

    // 获取力命令
    double PendulumController::get_force_command() const
    {
      return force_command_;
    }

    // 计算力命令
    double PendulumController::calculate(
        const std::vector<double> &state,
        const std::vector<double> &reference) const
    {
      double controller_output = 0.0;
      size_t dim = state.size();
      if ((dim != reference.size()) &&
          (dim != cfg_.get_feedback_matrix().size()))
      {
        throw std::invalid_argument("wrong state size vector");
      }

      // 根据反馈矩阵和系统状态计算控制器输出
      for (size_t i = 0; i < dim; i++)
      {
        controller_output += -cfg_.get_feedback_matrix()[i] * (state[i] - reference[i]);
      }

      return controller_output;
    }
  } // namespace pendulum_controller
} // namespace pendulum
