#ifndef __COMWISE_DRV__DRV_CHASSIS__H__
#define __COMWISE_DRV__DRV_CHASSIS__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace chassis {
    class chassis_base;
}
namespace drv {

class drv_chassis : public drv_impl
{
    using chassis_ptr_t = std::shared_ptr<chassis::chassis_base>;

public:
    drv_chassis(drv_id_t id, drv_type_t type = comwise::kHwChassis);
    virtual ~drv_chassis() override;

    //! read and write data
    virtual int read(const drv_request_t &, drv_reply_t) override;
    virtual int write(const drv_request_t &, drv_reply_t) override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    chassis_ptr_t chassis_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_CHASSIS__H__
