#ifndef __COMWISE_MOTOR__MOTOR_FACTORY__H__
#define __COMWISE_MOTOR__MOTOR_FACTORY__H__

#include <string>
#include <map>
#include "common/singleton.h"
#include "motor_driver.h"

namespace comwise {
namespace motor {

enum kMotorPosition {
  kMotorFrontLeft   = 0,
  kMotorFrontRight  = 1,
  kMotorBackRight   = 2,
  kMotorBackLeft    = 3,
  kMotorLift        = 10,
  kMotorRotate      = 11,
  kMotorCurtis      = kMotorFrontLeft,
  kMotorLeft        = kMotorFrontLeft,
  kMotorRight       = kMotorFrontRight,
};

class motor_factory final : public common::singleton<motor_factory>
{
public:
  using motor_driver_t = std::shared_ptr<motor_driver>;
  using motor_id_t = std::string;
  using motor_pos_t = uint32_t;

public:
  motor_driver_t create(const motor_id_t &id, uint32_t type) {
    auto cit = drv_list_.find(id);
    if (cit != drv_list_.end()) {
        return cit->second;
    } else {
      motor_driver_t obj = create_object(id, type);
      drv_list_[id] = obj;
      return obj;
    }
  }

  motor_driver_t find(const motor_id_t &id) {
    auto cit = drv_list_.find(id);
    return cit != drv_list_.end()? cit->second : nullptr;
  }

  motor_driver_t find(motor_pos_t type) {
    for (auto &cit : drv_list_) {
      if(cit.second && cit.second->get_pos() == type) {
        return cit.second;
      }
    }
    return nullptr;
  }

private:
  motor_driver_t create_object(const std::string &id, uint32_t type) {
    switch (type) {
      default:
        return nullptr;
    }
  }

private:
  std::map<motor_id_t, motor_driver_t> drv_list_;

};

} // namespace motor
} // namespace comwise

#define MOTOR_FACTORY comwise::motor::motor_factory::instance()

#endif // __COMWISE_MOTOR__MOTOR_FACTORY__H__
