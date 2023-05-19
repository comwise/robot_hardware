/**
 * Copyright (c) 2022 comwise Inc. All rights reserved.
 */
#ifndef __COMWISE_MOTOR__MOTOR_DRIVER__H__
#define __COMWISE_MOTOR__MOTOR_DRIVER__H__

#include <chrono>
#include "motor_data.h"
#include "core/driver.h"

namespace comwise {
namespace can {
  class msg_publisher;
  class canopen;
}
namespace motor {

class motor_driver : public core::driver_event<motor_param_t, void *, void *>
{
protected:
  using can_frame_t = can::can_frame_t;
  using msg_publisher_t = can::msg_publisher;
  using canopen_t = can::canopen;

public:
  motor_driver(const std::string &id, uint32_t type = 0)
    : core::driver_event<motor_param_t, void *, void *>(id, type) { }
  virtual ~motor_driver() { }

  virtual int init(const motor_param_t &param) override { 
    param_ = param;
    data_.tag = param.position;
    set_state(kStateReady);
    return 0;
  }
  virtual int deinit() override { set_state(kStateIdle); return 0; }

  virtual int start() override { set_state(kStateRunning); return 0; }
  virtual int stop() override { set_state(kStatePause); return 0; }

  virtual int get_pos() { return data_.tag; }
  virtual int set_pos(int pos) { data_.tag = pos; }

  virtual void set_control(double speed) { }
  virtual void get_data(void *data) { }

  virtual void SetControl(float velocity, float angle, uint16_t action) { }
  virtual void SetControlWithRatio(float velocity, float angle, 
                  uint16_t mv_rpm, uint16_t action, bool manual_mode = false) { }

  virtual void SetConfig(uint16_t accel, uint16_t decel, uint16_t steering_limit) { }
  virtual void SetReset(bool reset) {}
  virtual void SetAccelAndDecelControl(double time1, double time2) { }
  virtual void SetHeartBeat() { }
  
  virtual bool is_ready() { 
    auto now = std::chrono::steady_clock::now();
    auto delta_time = std::chrono::duration_cast<
            std::chrono::milliseconds>(now - time_).count();
    if (delta_time > 500) {
      set_error(0, kLevelTimeout);
    } else {
      clear();
    }
    return get_state() == kStateRunning && get_level() == kLevelOK;
  }

protected:
  motor_param_t param_;
  motor_data_t data_;
  std::shared_ptr<canopen_t> canopen_{nullptr};

  std::chrono::steady_clock::time_point time_;

};

} // namespace motor
} // namespace comwise

#endif  // __COMWISE_MOTOR__MOTOR_DRIVER__H__
