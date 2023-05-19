#ifndef __COMWISE_DRV__DRV_IMPL__H__
#define __COMWISE_DRV__DRV_IMPL__H__

#include <atomic>
#include <vector>
#include "drv_base.h"

namespace std {
    class thread;
}

namespace comwise {
namespace drv {

class drv_impl : public drv_base
{
protected:
    using status_ptr_t = std::shared_ptr<status_obj_t>;

public:
    drv_impl(drv_id_t id, drv_type_t type = comwise::kHwIO);
    virtual ~drv_impl() override;

    //! init/deinit driver
    virtual int init(const drv_param_t &param= nullptr) override;
    virtual int deinit() override;

    //! start/stop driver
    virtual int start() override;
    virtual int stop() override;

    //! enable/disable driver
    virtual int set_config(const drv_param_t &cfg) override;
    virtual int get_config(drv_param_t &cfg) override;

    //! read and write data
    virtual int read(const drv_request_t &, drv_reply_t) override;
    virtual int write(const drv_request_t &, drv_reply_t) override;

protected:
    //! start/stop driver
    virtual int _start();
    virtual int _stop();

    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true);

    //! publish health message
    //virtual int publish(const status_data_t &data);
    virtual int publish(const std::string &name, const std::string &msg,
        int level = 0, const std::string &suggest = "");

protected:
    drv_param_t drv_cfg_;

    std::unique_ptr<std::thread> drv_thr_;
    std::atomic<bool> is_loop_ {true};

    status_ptr_t status_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_IMPL__H__
