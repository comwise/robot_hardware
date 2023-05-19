#ifndef __COMWISE_DRV__DRV_MOTOR__H__
#define __COMWISE_DRV__DRV_MOTOR__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace motor {
class motor_driver;
}
namespace drv {

class drv_motor : public drv_impl
{
    using motor_driver_t = std::shared_ptr<motor::motor_driver>;

public:
    drv_motor(drv_id_t id, drv_type_t type = comwise::kHwMotor);
    virtual ~drv_motor() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    motor_driver_t drv_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_MOTOR__H__
