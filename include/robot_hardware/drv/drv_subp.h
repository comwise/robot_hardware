#ifndef __COMWISE_DRV__DRV_SUBP__H__
#define __COMWISE_DRV__DRV_SUBP__H__

#include <atomic>
#include <vector>
#include "drv_impl.h"

namespace std {
    class thread;
}

namespace comwise {
namespace process {
    class subp_ctrler;
}
namespace drv {

class drv_subp : public drv_impl
{
protected:
    using subp_ctrler_t = std::shared_ptr<comwise::process::subp_ctrler>;
    using status_ptr_t = std::shared_ptr<status_obj_t>;

public:
    drv_subp(drv_id_t id, drv_type_t type = comwise::kHwIO);
    virtual ~drv_subp() override;

    //! init/deinit driver
    virtual int init(const drv_param_t &param = nullptr) override;
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
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true);

    //! publish health message
    //virtual int publish(const status_data_t &data);
    virtual int publish(const std::string &name, const std::string &msg,
        int level = 0, const std::string &suggest = "");

    //! start/stop driver
    virtual int _start();
    virtual int _stop();

protected:
    subp_ctrler_t subp_ctrler_ { nullptr };
    std::vector<std::string> subp_args_;
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_SUBP__H__
