#ifndef __COMWISE__CHASSIS_SMSR__H__
#define __COMWISE__CHASSIS_SMSR__H__

#include "chassis_impl.h"

namespace comwise {
namespace chassis {

class chassis_smsr : public chassis_impl
{
public:
    explicit chassis_smsr(const chassis_id_t &id, 
                        chassis_type_t type = kChassisSMSR);
    virtual ~chassis_smsr();

private:
    virtual void work_handler() override;

    int move_cmd(const vel_t &vel);
    int move_feedback(vel_t &vel);
    int move_odom(const vel_t &vel, double time);

protected:
    bool manual_mode_{false};
    bool last_manual_mode_{true};
    uint32_t power_mode_{0};
};

} // namespace chassis
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_SMSR__H__
