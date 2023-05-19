/**
 * Copyright (c) 2023 comwise. All rights reserved.
 */
#ifndef __COMWISE_COMMON__ALGO__H__
#define __COMWISE_COMMON__ALGO__H__

//#include <angles/angles.h>
#include <chrono>
#include <cmath>
#include "log/log.h"

namespace common {

#define PI_MATH 3.1415926

template <typename T>
int Sign(T data) {
  if (data > 0) {
    return 1;
  } else if (data < 0) {
    return -1;
  } else {
    return 0;
  }
}

template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

template <typename T>
bool NearZero(const T& val, double bound = 0.0001) {
  return std::fabs(static_cast<double>(val)) <= bound;
}



// 1 (m/s) = 60/(d * pi) (rpm)
// 轮子速度---> rpm
template <typename T>
T WheelVelocityToRPM(T velocity, T diameter, T radio) {
  return velocity / M_PI / diameter * 60.0 * radio;
}

template <typename T>
T RPMToWheelVelocity(T rpm, T diameter, T radio) {
  return rpm * M_PI * diameter / 60.0 / radio;
}

// 1 (rad/s) = 60 / (2 * pi) (rpm)
// 旋转电机角速度 --> rpm
template <typename T>
T RadPSToRPM(T radps, T radio) {
  T ret = 60.0 * radio * radps / (2 * M_PI);
  if (std::fabs(ret) > 3000) {
    LOGP_ERROR("[RadPSToRPM] over 3000 rpm");
    ret = Sign(radps)* 3000;
  }
  return ret;
}

template <typename T>
T RPMToRadPS(T radps, T radio) {
  return 60.0 * radio * radps / (2 * M_PI);
}

template <typename T>
T LiftVPSToRPM(T lift_v, T radio, T screw) {
  T ret = 60.0 * lift_v * radio / screw;
  if (std::fabs(ret) > 3000) {
    LOGP_ERROR("[RadPSToRPM] over 3000 rpm");
    ret = Sign(lift_v) * 3000;
  }
  LOG_DEBUG("[RadPSToRPM] ret:%f", ret);
  return ret;
}

template <typename T>
T RPMToLiftVPS(T radps, T radio, T screw) {
  return radps * screw / 60.f / radio;
}

// 模型速度 --> 左右轮子速度 according to the formula:
// Vl = V - W * L / 2
// Vr = V + W * L / 2
template <typename T>
T CalcLeftVelocity(T v, T w, T wheel_track) {
  return v - w * wheel_track / 2.;
}

template <typename T>
T CalcRightVelocity(T v, T w, T wheel_track) {
  return v + w * wheel_track / 2.;
}

// 左右轮子速度 --> 模型线速度速度和角速度
template <typename T>
T WheelVelToVelocity(T left_vel, T right_vel) {
  return (left_vel + right_vel) / 2.;
}

template <typename T>
T WheelVelToOmega(T left_vel, T right_vel, T wheel_track) {
  return (right_vel - left_vel) / wheel_track;
}

// 大小端转化
template <typename T>
T GetHtons(const uint8_t* byArr) {
  return ((static_cast<T>(byArr[0]) << 8) & 0xFF00) | byArr[1];
}

enum class Direction {
  None = 0,
  XPositive = 0x1100,  // on X Axis positive direction
  XNegative = 0x1300,  // on X Axis negative direction
  YPositive = 0x1400,  // on Y Axis positive direction
  YNegative = 0x1200,  // on Y Axis negative direction
  XPosYPos = 0x1500,
  XPosYNeg = 0x1600,
  XNegYPos = 0x1700,
  XNegYNeg = 0x1800,
};

// wheel omega to motor rotate speed
template <typename T>
T rads2rmin(T rad_persec, int reduceRatio) {
  return 60 * reduceRatio * rad_persec / (2 * PI_MATH);
}

template <typename T1, typename T2>
T1 Omega2PalletRotater(T1 omega, T2 reduce_ratio) {
  return rads2rmin(omega, reduce_ratio);
}

// motor rotate speed to wheel omega
template <typename T>
T rmin2rads(T r_permin, T reduceRatio) {
  return r_permin * 2.0f * PI_MATH / 60.0f / reduceRatio;
}

// degree conver to rad
template <typename T>
T Deg2Rad(T degree) {
  return degree * 2.0f * PI_MATH / 360.0f;
}

// rad conver to degree
template <typename T>
T Rad2Deg(T rad) {
  return rad * 360.0f / (2.0f * PI_MATH);
}

//
template <typename T>
T normalize_angle_positive(T angle) {
  return static_cast<T>(
      fmod(fmod(angle, 2.0f * PI_MATH) + 2.0f * PI_MATH, 2.0f * PI_MATH));
}

template <typename T>
T normalize_angle(T angle) {
  T a = normalize_angle_positive(angle);
  if (a > PI_MATH) a -= 2.0f * PI_MATH;
  return a;
}

template <typename T>
T normal_theta(T theta) {
    T angle = theta;
    while (angle < -2*M_PI) {
        angle += 2*M_PI;
    }

    while (angle > 2*M_PI) {
        angle -= 2*M_PI;
    }

    return angle;
}

#if 0
template <typename T>
T AngleSubtract(T param1, T param2) {
  return angles::normalize_angle(param1 - param2);
}
#endif

template <typename T>
T AngleAddition(T param1, T param2) {
  T angle = param1 + param2;
  if (angle > PI_MATH)
    return fmod(fmod(angle, 2 * PI_MATH) - 2 * PI_MATH, 2 * PI_MATH);
  else if (angle < -PI_MATH)
    return fmod(fmod(angle, 2 * PI_MATH) + 2 * PI_MATH, 2 * PI_MATH);
  else
    return angle;
}

template <typename T>
bool fNearby(T left, T right, T diff) {
  return fabs(left - right) <= diff ? true : false;
}

template <typename T>
Direction Angle2Direction(T current_angle) {
  if (fNearby(current_angle, static_cast<T>(0), static_cast<T>(PI_MATH / 4)))
    return Direction::XPositive;
  if (fNearby(current_angle, static_cast<T>(PI_MATH / 2),
              static_cast<T>(PI_MATH / 4)))
    return Direction::YPositive;
  if (fNearby(current_angle, static_cast<T>(-PI_MATH / 2),
              static_cast<T>(PI_MATH / 4)))
    return Direction::YNegative;
  return Direction::XNegative;
}

//
template <typename T>
double Direction2Angle(T direction) {
  switch (direction) {
    case Direction::XPositive:
      return 0.00f;
    case Direction::XNegative:
      return PI_MATH;
    case Direction::YPositive:
      return PI_MATH / 2;
    case Direction::YNegative:
      return -PI_MATH / 2;
    case Direction::XPosYPos:
      return PI_MATH / 4;
    case Direction::XPosYNeg:
      return -PI_MATH / 4;
    case Direction::XNegYPos:
      return PI_MATH * 3 / 4;
    case Direction::XNegYNeg:
      return -PI_MATH * 3 / 4;
    default:
      return -1;
  }
}
template <typename T>
float fbound(T raw, T rmin, T rmax) {
  return fmin(fmax(raw, rmin), rmax);
}

}  // namespace common

#endif  // comwise_COMMON_INCLUDE_comwise_COMMON_COMMON_H_
