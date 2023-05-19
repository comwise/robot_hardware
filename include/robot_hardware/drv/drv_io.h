#ifndef __COMWISE_DRV__DRV_IO__H__
#define __COMWISE_DRV__DRV_IO__H__

#include <atomic>
#include <vector>
#include "drv_subp.h"

namespace std {
    class thread;
}

namespace comwise {
namespace process {
    class ros_ctrler;
}
namespace net{
    class net_plugin;
}
namespace drv {

class drv_io : public drv_subp
{
    using ros_ctrler_t = comwise::process::ros_ctrler;
    using net_plugin = comwise::net::net_plugin;

public:
    drv_io(drv_id_t id, drv_type_t type = comwise::kHwIO);
    virtual ~drv_io() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

private:
    bool init_drvparam(const drv_param_t &cfg);
    bool init_subparam(const drv_param_t &cfg);

    //! init ros
    int init_ros_args();
    int init_ros_service();

    int start_subp();
    int start_rosp();

protected:
    std::shared_ptr<ros_ctrler_t> ros_ctrler_ { nullptr };
    std::map<std::string, std::string> ros_args_;

    std::atomic<bool> is_ros_ {false};

    std::shared_ptr<net_plugin> net_plugin_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_IO__H__
