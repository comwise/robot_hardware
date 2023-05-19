/**
 * Copyright (c) 2023 comwise. All rights reserved.
 */
#ifndef __COMWISE__CHASSIS_DATA__H__
#define __COMWISE__CHASSIS_DATA__H__

#include <string>
#include <vector>
#include <map>

namespace comwise {

struct MotorData {
  uint32_t time_stamp{0};
  int32_t current{0};
  int32_t speed{0};
  int32_t position{0};
  uint16_t status_code{0};
};

struct BicycleModelOption {
  double wheel_base{0.0};         // 轴距 m
  double max_wheel_base{0.0};     // 最大轴距
  double min_wheel_base{0.0};     // 最小轴距
  double eccentric_distance{0.0}; // 舵轮距过后轮中心垂线的距离(偏心距) m
  double eccentric_angle{0.0};    // 舵轮(偏心角度) 弧度
  double reduction_ratio{0.0};    // 辅助减速比
};

struct DiffModelOption {
  double wheel_track{0.0};                // 轮间距 m
  double wheel_diameter{0.0};             // 轮子直径 m
  double left_motor_reduce_ratio{1.0};    // 左电机减速比
  double right_motor_reduce_ratio{1.0};   // 右电机减速比
  int left_motor_direction{1};            // 左电机方向
  int right_motor_direction{1};           // 右电机方向

  // the following variable to be improved
  double lift_motor_reduce_ratio{0.0};    // 顶升电机减速比
  double lift_screw_lead{0.0};            // 顶升结构导程
  double lift_max_height{0.0};            // 顶升电机最大顶升高度
  int lift_mechanism{0};                  // 升降机构:1-丝杆 2-剪刀叉 3-IO气缸
  bool use_lift{false};                   // 启用顶升
  bool lift_motor_mode{false};            // 顶升电机工作模式：0-速度 1:绝对位置

  double rotate_motor_reduce_ratio{0.0};  // 旋转电机减速比
  bool use_rotate{false};                 // 启用旋转
  
  bool use_up_camera{false};              // 启用向上电机
  bool use_down_camera{false};            // 启用向下电机
  
  bool use_io_laser_avoid{false};         // 启用激光IO雷达避障
};

struct SSMRModelOption {
  double wheel_track{0.0};                // 轮间距 m
  double wheel_diameter{0.0};             // 轮子直径 m
  double move_motor_reduce_ratio{1.0};    // 移动电机减速比 系数
  double lift_motor_reduce_ratio{0.0};    // 顶升电机减速比
  double lift_screw_lead{0.0};            // 顶升结构导程
  double lift_max_height{0.0};            // 顶升电机最大顶升高度
  double rotate_motor_reduce_ratio{0.0};  // 旋转电机减速比
  double gama{0.0};                       // 求解虚拟轮间距的相关系数
  double virtual_wheel_track{0.0};        // 虚拟轮间距
  int lift_mechanism{0};                  // 升降机构:1-丝杆 2-剪刀叉 3-IO气缸
  bool use_lift{false};                   // 启用顶升
  bool lift_motor_mode{false};            // 顶升电机工作模式：0-速度 1:绝对位置
  bool use_rotate{false};                 // 启用旋转
  bool use_up_camera{false};              // 启用向上电机
  bool use_down_camera{false};            // 启用向下电机
  bool use_io_laser_avoid{false};         // 启用激光IO雷达避障
};

struct MoveModelControl {
  // 传感器屏蔽相关参数
  bool use_fork_avod{0.0};
  double fork_avoid_dist{0.0};
  // 惯性补偿相关参数
  bool use_inertia_compensation{0.0};
  double velocity_coefficient{0.0};
  double omega_coefficient{0.0};
  // 手动控制相关参数
  double pallet_up_speed{0.0};
  double pallet_down_speed{0.0};
  double pallet_rotate_speed{0.0};
  double forward_speed{0.0};
  double backward_speed{0.0};
  double rotate_left_speed{0.0};
  double rotate_right_speed{0.0};
  uint8_t led_type{0};
  uint8_t audio_type{0};
  bool enter_low_power{false};
  bool enable_charge{false};
  //电机选装方向定义
  double set_left_sign{0.0};
  double set_right_sign{0.0};
  double feedback_left_sign{0.0};
  double feedback_right_sign{0.0};
  double rotate_motor_sign{0.0};
  double lift_motor_sign{0.0};
};

// 移动模型参数
struct MoveModelOption {
  uint32_t model_type{0};
  BicycleModelOption steer_model_option;
  DiffModelOption diff_model_option;
  SSMRModelOption ssmr_model_option;
  MoveModelControl movemode_control_option;
};


} // namespace comwise

#endif  // __COMWISE__CHASSIS_DATA__H__
