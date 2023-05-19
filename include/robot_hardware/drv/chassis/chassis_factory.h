#ifndef __COMWISE__CHASSIS_FACTORY__H__
#define __COMWISE__CHASSIS_FACTORY__H__

#include <cstdint>
#include <memory>
#include <string>
#include "chassis_steer.h"
#include "chassis_diff.h"
#include "chassis_smsr.h"

namespace comwise {
namespace chassis {

class chassis_factory final
{
public:
    using chassis_ptr_t = std::shared_ptr<chassis_base>;
    using chassis_id_t = chassis_base::chassis_id_t;

public:
    static chassis_ptr_t create_object(
        const chassis_id_t &id, uint32_t type) {
        chassis_ptr_t obj {nullptr};
        switch (type) {
        case kChassisNone:
            obj = std::make_shared<chassis_impl>(id); break;
        case kChassisSteer:
            obj = std::make_shared<chassis_steer>(id); break;
        case kChassisDiff:
            obj = std::make_shared<chassis_diff>(id); break;
        case kChassisSMSR:
            obj = std::make_shared<chassis_smsr>(id); break;
        default:
            obj = nullptr; break;
        }
        return std::move(obj);
    }
};

} // namespace chassiss
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_FACTORY__H__
