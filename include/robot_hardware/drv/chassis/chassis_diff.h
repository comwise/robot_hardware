#ifndef __COMWISE__CHASSIS_DIFF__H__
#define __COMWISE__CHASSIS_DIFF__H__

#include "chassis_impl.h"

namespace comwise {
namespace chassis {

class chassis_diff : public chassis_impl 
{
public:
    explicit chassis_diff(const chassis_id_t &id, 
                        chassis_type_t type = kChassisDiff);
    virtual ~chassis_diff();

private:
    virtual void work_handler() override;

    int move_cmd(const vel_t &vel);
    int move_feedback(vel_t &vel);
    int move_odom(const vel_t &vel, double delta_time);

protected:
    bool manual_mode_{false};
    bool last_manual_mode_{true};
};

} // namespace chassis
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_DIFF__H__
