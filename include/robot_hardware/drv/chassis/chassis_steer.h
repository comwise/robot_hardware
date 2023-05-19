#ifndef __COMWISE__CHASSIS_STEER__H__
#define __COMWISE__CHASSIS_STEER__H__

#include "chassis_impl.h"

namespace comwise {
namespace curtis {
    class curtis_driver;
}
namespace chassis {

class chassis_steer : public chassis_impl
{
    using curtis_driver_t = curtis::curtis_driver;
    using curtis_driver_ptr = std::shared_ptr<curtis_driver_t>;
public:
    explicit chassis_steer(const chassis_id_t &id, 
                        chassis_type_t type = kChassisSteer);
    virtual ~chassis_steer();

private:
    virtual void work_handler() override;

    int move_cmd(const vel_t &vel);
    int move_feedback(vel_t &vel);
    int move_odom(const vel_t &vel, double time);
};

} // namespace chassis
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_STEER__H__
