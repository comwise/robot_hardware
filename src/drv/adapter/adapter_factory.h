#include <cstdint>
#include "core/driver.h"
#include "var/var.h"
#include "etc/etc_default.h"

namespace comwise {
namespace drv {

class adapter_factory
{
public:
  using drv_ptr_t = std::shared_ptr<core::driver_core<sensor_param_t>>;

public:
  static drv_ptr_t create(const std::string &id, uint16_t major, const std::string &minor = "")
  {
    drv_ptr_t drv {nullptr};
    switch (major)
    {
    case kHwBattery:
      break;
    case kHwUltrasonic:
      break;
    case kHwIMU:
      break;
    default:
      break;
    };
    return drv;
  }
};

} // drv
} // comwise