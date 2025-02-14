#include "pendulum_driver/pendulum_driver.hpp"
#include <vector>
#include "rcppmath/clamp.hpp"

namespace pendulum
{
  namespace pendulum_driver
  {
    // PendulumDriver 类的构造函数
    PendulumDriver::PendulumDriver(const Config &config)
        : cfg_(config),
          ode_solver_(STATE_DIM),
          X_{0.0, 0.0, M_PI, 0.0},
          controller_force_{0.0},
          disturbance_force_{0.0},
          rand_gen_(rd()),
          noise_gen_(std::uniform_real_distribution<double>(
              -config.get_noise_level(), config.get_noise_level()))
    {
      // 计算控制器的时间步长（用于离散微分/积分）
      dt_ = cfg_.get_physics_update_period().count() / (1000.0 * 1000.0);
      if (std::isnan(dt_) || dt_ == 0)
      {
        throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
      }

      // 导数函数，用于计算状态的导数
      derivative_function_ = [this](const std::vector<double> &y,
                                    double u, size_t i) -> double
      {
        const double m = cfg_.get_pendulum_mass();
        const double M = cfg_.get_cart_mass();
        const double L = cfg_.get_pendulum_length();
        const double d = cfg_.get_damping_coefficient();
        const double g = cfg_.get_gravity();
        if (i == 0)
        {
          return y[1];
        }
        else if (i == 1)
        {
          double Sy = sin(y[2]);
          double Cy = cos(y[2]);
          double D = m * L * L * (M + m * (1 - Cy * Cy));
          return (1 / D) *
                     (-m * m * L * L * g * Cy * Sy + m * L * L * (m * L * y[3] * y[3] * Sy - d * y[1])) +
                 m * L * L * (1 / D) * u;
        }
        else if (i == 2)
        {
          return y[3];
        }
        else if (i == 3)
        {
          double Sy = sin(y[2]);
          double Cy = cos(y[2]);
          double D = m * L * L * (M + m * (1 - Cy * Cy));
          return (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * (m * L * y[3] * y[3] * Sy - d * y[1])) - m * L * Cy * (1 / D) * u + noise_gen_(rand_gen_);
        }
        else
        {
          throw std::invalid_argument("received wrong index");
        }
      };
    }

    // 设置控制器施加在小车上的力
    void PendulumDriver::set_controller_cart_force(double force)
    {
      controller_force_ = rcppmath::clamp(force, -cfg_.get_max_cart_force(), cfg_.get_max_cart_force());
    }

    // 设置系统的状态
    void PendulumDriver::set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel)
    {
      state_.cart_position = cart_pos;
      state_.cart_velocity = cart_vel;
      state_.pole_angle = pole_pos;
      state_.pole_velocity = pole_vel;
    }

    // 设置干扰力
    void PendulumDriver::set_disturbance_force(double force)
    {
      disturbance_force_ = force;
    }

    // 获取系统的状态
    const PendulumDriver::PendulumState &PendulumDriver::get_state() const
    {
      return state_;
    }

    // 获取控制器施加在小车上的力
    double PendulumDriver::get_controller_cart_force() const
    {
      return controller_force_;
    }

    // 获取干扰力
    double PendulumDriver::get_disturbance_force() const
    {
      return disturbance_force_;
    }

    // 更新系统状态
    void PendulumDriver::update()
    {
      double cart_force = disturbance_force_ + controller_force_;
      ode_solver_.step(derivative_function_, X_, dt_, cart_force);

      state_.cart_position = X_[0];
      state_.cart_velocity = X_[1];
      state_.cart_force = cart_force;
      state_.pole_angle = X_[2];
      state_.pole_velocity = X_[3];
    }

    // 重置系统状态
    void PendulumDriver::reset()
    {
      // 初始状态为竖直向上
      set_state(0.0, 0.0, M_PI, 0.0);
      set_disturbance_force(0.0);
      set_controller_cart_force(0.0);
      X_ = {0.0, 0.0, M_PI, 0.0};
    }

  } // namespace pendulum_driver
} // namespace pendulum
