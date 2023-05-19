#ifndef __COMWISE_CHASSIS__CHASSIS_MODEL__H__
#define __COMWISE_CHASSIS__CHASSIS_MODEL__H__

#include <tuple>
#include "common/algo.h"
#include "var/var.h"

namespace comwise {
namespace chassis {

class move_model
{
public:
  virtual vel_t get_move(double, double) = 0;
  virtual std::tuple<double, double> set_move(const vel_t &msg) = 0;

  virtual void update_wheelbase(double wheel_base) = 0;

  void set_config(const MoveModelControl &option) { control_option_ = option; }
  MoveModelControl get_config() { return control_option_; }

private:
  MoveModelControl control_option_;
};

class move_steer_model : public move_model
{
  struct MoveData {
    double v;
    double w;
  };

public:
  move_steer_model() = delete;
  explicit move_steer_model(const BicycleModelOption &option)
      : option_(option) {}

  /**
   *  @brief  自行车模型下 vd, theta 转化为 v, w
   *  [driver_velocity, steering_angle] --> [actual_velocity, actual_omega]
   *  @param  driver_velocity  舵轮速度 m/s
   *  @param  steering_angle   舵轮转角 弧度 (-1.57 ~ 1.57)
   */
  vel_t get_move(double driver_velocity, double steering_angle) {
    vel_t msg;
    double final_steering_angle = steering_angle + option_.eccentric_angle;
    msg.w = driver_velocity * option_.reduction_ratio *
            std::sin(final_steering_angle) / option_.wheel_base;
    msg.v = driver_velocity * option_.reduction_ratio *
        (std::cos(final_steering_angle) + option_.eccentric_distance *
              std::sin(final_steering_angle) / option_.wheel_base);
    msg.vv = driver_velocity * option_.reduction_ratio; // 供导航使用
    return msg;
  }

  /*
  (v, w) --> (driver_velocity, steering_angle)
  */
  std::tuple<double, double> set_move(const vel_t &msg) {
    double angle = 0.f;
    double speed = 0.f;
    angle = msg.w;
    double final_cmd_angle = common::Clamp<double>(angle - option_.eccentric_angle,
                            common::Deg2Rad(-90.), common::Deg2Rad(90.));

    double temp = common::NearZero(std::cos(final_cmd_angle))
                      ? 0.001 : std::cos(final_cmd_angle);
    // 限速 -3m/s ~ 3m/s
    speed = msg.v / option_.reduction_ratio;
    // }
#ifdef DEBUG_CHASSIS_MODEL
    printf("model: %f, %f, %f, %f\n", angle, final_cmd_angle, temp, speed);
#endif

    return std::make_tuple(speed, final_cmd_angle);
  }

  // 更新轴距 m
  void update_wheelbase(double wheelbase) { option_.wheel_base = wheelbase; }

private:
  BicycleModelOption option_;
  MoveData cmd_fdbk;
  MoveData last_cmd;
};

class move_diff_model : public move_model
{
public:
  move_diff_model() /*= delete*/ {};

  explicit move_diff_model(const DiffModelOption &option)
      : option_(option) {}

  /**
   *  @brief  差速轮模型下 v_l, v_r 转化为 v, w
   *  [left_rpm, right_rpm] --> [actual_velocity, actual_omega]
   *  @param  left_rpm  左轮转速
   *  @param  right_rpm  右轮转速
   */
  vel_t get_move(double left_rpm, double right_rpm) {
    vel_t msg;

    auto left_vel = common::Sign(option_.left_motor_direction) *
                    common::RPMToWheelVelocity(left_rpm, option_.wheel_diameter,
                                               option_.left_motor_reduce_ratio);
    auto right_vel =
        common::Sign(option_.right_motor_direction) *
        common::RPMToWheelVelocity(right_rpm, option_.wheel_diameter,
                                   option_.right_motor_reduce_ratio);

    msg.v = common::WheelVelToVelocity(left_vel, right_vel);
    // 顶升车 virtual_velocity = actual_velocity
    msg.vv = common::WheelVelToVelocity(left_vel, right_vel);
    msg.w = common::WheelVelToOmega(left_vel, right_vel, option_.wheel_track);
    
    return msg;
  }

  /*
  (v, w) --> (left_rpm, right_rpm)
  */
  std::tuple<double, double> set_move(const vel_t &msg) {
    auto left_velocity = common::CalcLeftVelocity(
        msg.v, msg.w, option_.wheel_track);
    auto right_velocity = common::CalcRightVelocity(
        msg.v, msg.w, option_.wheel_track);

    auto left_rpm =
        common::Sign(option_.left_motor_direction) *
        common::WheelVelocityToRPM(left_velocity, option_.wheel_diameter,
                                   option_.left_motor_reduce_ratio);
    auto right_rpm =
        common::Sign(option_.right_motor_direction) *
        common::WheelVelocityToRPM(right_velocity, option_.wheel_diameter,
                                   option_.right_motor_reduce_ratio);

    return std::make_tuple(left_rpm, right_rpm);
  }

  void update_wheelbase(double wheelbase) {}

private:
  DiffModelOption option_;
};

class move_ssmr_model : public move_model
{
public:
  move_ssmr_model() = delete;

  explicit move_ssmr_model(const SSMRModelOption &option)
      : option_(option) {}

  /**
   *  @brief  独轮车模型下 v_l, v_r 转化为 v, w
   *  [left_rpm, right_rpm] --> [actual_velocity, actual_omega]
   *  @param  left_rpm  左轮转速
   *  @param  right_rpm  右轮转速
   */
  vel_t get_move(double left_rpm, double right_rpm) {
    vel_t msg;

    auto left_vel = (common::Sign(get_config().feedback_left_sign)) *
                    common::RPMToWheelVelocity(left_rpm, option_.wheel_diameter,
                                               option_.move_motor_reduce_ratio);
    auto right_vel =
        (common::Sign(get_config().feedback_right_sign)) *
        common::RPMToWheelVelocity(right_rpm, option_.wheel_diameter,
                                   option_.move_motor_reduce_ratio);

    msg.v = common::WheelVelToVelocity(left_vel, right_vel);
    // 顶升车 virtual_velocity = actual_velocity
    msg.vv = common::WheelVelToVelocity(left_vel, right_vel);
    msg.w =
        common::WheelVelToOmega(left_vel, right_vel, option_.wheel_track);
    return msg;
  }

  /*
  (v, w) --> (left_rpm, right_rpm)
  */
  std::tuple<double, double> set_move(const vel_t &msg) {
    auto left_velocity = common::CalcLeftVelocity(
        msg.v, msg.w, option_.wheel_track);
    auto right_velocity = common::CalcRightVelocity(
        msg.v, msg.w, option_.wheel_track);

    auto left_rpm =
        (common::Sign(get_config().set_left_sign)) *
        common::WheelVelocityToRPM(left_velocity, option_.wheel_diameter,
                                   option_.move_motor_reduce_ratio);
    auto right_rpm =
        (common::Sign(get_config().set_right_sign)) *
        common::WheelVelocityToRPM(right_velocity, option_.wheel_diameter,
                                   option_.move_motor_reduce_ratio);

    return std::make_tuple(left_rpm, right_rpm);
  }

  void update_wheelbase(double wheelbase) {}

private:
  SSMRModelOption option_;
};

// 设置运动模型 (bicycle, unicycle) --> 用来解算导航数据
class move_model_factory final
{
public:
  static std::shared_ptr<move_model> create_model(
      uint32_t model_type, const MoveModelOption &option) {
    std::shared_ptr<move_model> move_model_{nullptr};
    switch (model_type)
    {
    case chassis_type_t::kChassisSteer:
    {
      move_model_ = std::make_shared<move_steer_model>(option.steer_model_option);
      break;
    }
    case chassis_type_t::kChassisDiff:
    {
      move_model_ = std::make_shared<move_diff_model>(option.diff_model_option);
      break;
    }
    case chassis_type_t::kChassisSMSR:
    {
      move_model_ = std::make_shared<move_ssmr_model>(option.ssmr_model_option);
      break;
    }
    case chassis_type_t::kChassisMecanum:
    default:
    {
      move_model_ = nullptr;
      break;
    }
    }
    if (move_model_) {
      move_model_->set_config(option.movemode_control_option);
    }
    return move_model_;
  }
};

} // namespace chassis
} // namespace comwise

#endif // __COMWISE_CHASSIS__CHASSIS_MODEL__H__
