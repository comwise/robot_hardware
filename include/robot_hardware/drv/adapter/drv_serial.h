#ifndef __COMWISE_DRV__DRV_SERIAL__H__
#define __COMWISE_DRV__DRV_SERIAL__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace drv {

class drv_serial : public drv_impl
{
public:
    drv_serial(drv_id_t id, drv_type_t type = comwise::kHwSerial);
    virtual ~drv_serial() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    std::string dev_;
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_SERIAL__H__
