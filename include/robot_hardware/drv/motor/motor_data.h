/**
 * Copyright (c) 2022 comwise Inc. All rights reserved.
 */
#ifndef __COMWISE_MOTOR__MOTOR_DATA__H__
#define __COMWISE_MOTOR__MOTOR_DATA__H__

#include <cstdint>
#include <string>
#include <memory>
#include "var/var_common.h"
#include "drv/can/can_data.h"

namespace comwise {
namespace motor {

enum motor_mode_t
{
  kMotorModeNone        = 0,
  kMotorModePos         = 1,
  kMotorModeAbsolutePos = kMotorModePos,
  kMotorModeRelativePos = 2,
  kMotorModeSpeed       = 3,
};

struct motor_data_t
{
  uint16_t tag{0};
  uint32_t time_stamp{0};
  int32_t  voltage{0};
  int32_t  current{0};
  int32_t  speed{0};
  int32_t  position{0};
  int16_t  status{0};
};

struct motor_param_t : public can::node_param_t
{
  uint32_t position{0};
};

} // namespace motor
} // namespace comwise

#endif  // __COMWISE_MOTOR__MOTOR_DATA__H__
