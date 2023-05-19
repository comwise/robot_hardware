#ifndef __COMWISE_DRV__DRV_SENSOR__H__
#define __COMWISE_DRV__DRV_SENSOR__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace drv {

class drv_sensor : public drv_impl
{
    using driver_ptr = std::shared_ptr<core::driver_core<sensor_param_t>>;
public:
    drv_sensor(drv_id_t id, drv_type_t type = comwise::kHwUltrasonic);
    virtual ~drv_sensor() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    driver_ptr drv_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_SERIAL__H__
