#ifndef __COMWISE_DRV__DRV_BASE__H__
#define __COMWISE_DRV__DRV_BASE__H__

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <functional>
#include "core/driver.h"
#include "var/var.h"

namespace comwise {
namespace drv {

using drv_var_t = std::shared_ptr<var_obj_t>;
using drv_param_t = std::shared_ptr<etc_obj_t>;
using drv_request_t = drv_var_t;
using drv_reply_t = drv_var_t;
class drv_base : public core::driver_event<
                     drv_param_t, drv_request_t, drv_reply_t>
{
public:
    using drv_type_t = hw_type_t;
    using drv_id_t = hw_id_t;
    using status_data_t = status_obj_t;

public:
    explicit drv_base(drv_id_t id, drv_type_t type) : driver_event(id, type) { }
    virtual ~drv_base() { }

    //! enable/disable driver
    virtual int set_config(const drv_param_t &cfg) = 0;
    virtual int get_config(drv_param_t &cfg) = 0;
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_BASE__H__
