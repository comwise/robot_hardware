#ifndef __COMWISE_DRV__DRV_CAN__H__
#define __COMWISE_DRV__DRV_CAN__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace can {
class can_center;
}
namespace drv {

class drv_can : public drv_impl
{
    using can_service_t = can::can_center;

public:
    drv_can(drv_id_t id, drv_type_t type = comwise::kHwCan);
    virtual ~drv_can() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    can_service_t* can_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_CAN__H__
