#ifndef __COMWISE_DRV__DRV_FACTORY__H__
#define __COMWISE_DRV__DRV_FACTORY__H__

#include <cstdint>
#include <memory>
#include "drv_io.h"
#include "adapter/drv_joystick.h"
#include "adapter/drv_chassis.h"
#include "adapter/drv_can.h"
#include "adapter/drv_network.h"
#include "adapter/drv_motor.h"
#include "adapter/drv_sensor.h"

namespace comwise {
namespace drv {

class drv_factory final
{
public:
    using drv_ptr_t = std::shared_ptr<drv_base>;
    using drv_type_t = drv_base::drv_type_t;
    using drv_id_t = drv_base::drv_id_t;

public:
    static drv_ptr_t create_object(drv_type_t type, drv_id_t obj_id)
    {
        drv_ptr_t obj {nullptr};
        switch (type)
        {
        case kHwLaser:
        case kHwModbus:
        case kHwCamera:
            obj = std::make_shared<drv::drv_io>(obj_id, type); break;
        case kHwMotor:
            obj = std::make_shared<drv::drv_motor>(obj_id, type); break;
        case kHwChassis:
            obj = std::make_shared<drv::drv_chassis>(obj_id, type); break;
        case kHwJoystick:
            obj = std::make_shared<drv::drv_joystick>(obj_id, type); break;
        case kHwBattery:
        case kHwIMU:
        case kHwUltrasonic:
            obj = std::make_shared<drv::drv_sensor>(obj_id, type); break;
        case kHwCan:
            obj = std::make_shared<drv::drv_can>(obj_id, type); break;
        case kHwNetwork:
            obj = std::make_shared<drv::drv_network>(obj_id, type); break;
        default:
            obj = std::make_shared<drv::drv_impl>(obj_id, type);
            break;
        }
        return std::move(obj);
    }
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_FACTORY__H__
