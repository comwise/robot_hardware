#ifndef __COMWISE__CHASSIS_BASE__H__
#define __COMWISE__CHASSIS_BASE__H__

#include <cstdint>
#include <memory>
#include <string>
#include "core/driver.h"
#include "var/var.h"

namespace comwise {
namespace chassis {

class chassis_base : public core::driver_event<
        std::shared_ptr<chassis_obj_t>, void*, void *>
{
public:
    using chassis_id_t = std::string;
    using chassis_type_t = uint32_t;
    using chassis_param_t = std::shared_ptr<chassis_obj_t>;
    using pos_t = pose_t;

public:
    explicit chassis_base(const chassis_id_t &id, chassis_type_t type)
        : core::driver_event<chassis_param_t, void*, void *>(id, type) { }
    virtual ~chassis_base() { }

    //!> set/get speed
    virtual int set_speed(const vel_t &vel) = 0;
    virtual int get_speed(vel_t &vel) = 0;
};

} // namespace chassis
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_BASE__H__
