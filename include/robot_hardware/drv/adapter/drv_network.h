#ifndef __COMWISE_DRV__DRV_NETWORK__H__
#define __COMWISE_DRV__DRV_NETWORK__H__

#include "drv/drv_impl.h"

namespace comwise {
namespace net {
    class net_plugin;
}
namespace drv {

class drv_network : public drv_impl
{
    using net_ptr_t = std::shared_ptr<net::net_plugin>;

public:
    drv_network(drv_id_t id, drv_type_t type = comwise::kHwNetwork);
    virtual ~drv_network() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    net_ptr_t drv_ptr_;
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_NETWORK__H__
