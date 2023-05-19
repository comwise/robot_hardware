#ifndef __COMWISE_JOY__JOY_CTL_BASE__H__
#define __COMWISE_JOY__JOY_CTL_BASE__H__

#include <memory>
#include <atomic>
#include <mutex>
#include <string>
#include <functional>
#include "joy_data.h"
#include "core/driver.h"

namespace std {
    class thread;
}

namespace comwise {
namespace joy {

using msg_id_t = std::string;
using msg_cfg_t = std::string;
using msg_data_t = void*;

class joy_ctl_base : public core::driver_event<msg_cfg_t, msg_data_t, msg_data_t>
{
public:
    explicit joy_ctl_base(const msg_id_t &name, uint32_t type = 0);
    virtual ~joy_ctl_base();

    //!> init/deinit app
    virtual int init(const msg_cfg_t &cfg = "") override;
    virtual int deinit() override;

    //!> start/stop app
    virtual int start() override;
    virtual int stop() override;

    //!> read/write data
    virtual int read(const msg_data_t &request, msg_data_t reply) override;
    virtual int write(const msg_data_t &request, msg_data_t reply) override;

    //!> set joy callback data
    virtual void set_joy_data(const joy_data &data);

protected:
    //!> sub app will override these function
    virtual int _start();
    virtual int _stop();

protected:
    msg_cfg_t cfg_;

    std::atomic_bool is_loop_{true};
    std::shared_ptr<std::thread> work_thr_ {nullptr};
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_CTL_BASE__H__
