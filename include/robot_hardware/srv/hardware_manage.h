#ifndef __COMWISE_SRV__RUNNER_MANAGE__H__
#define __COMWISE_SRV__RUNNER_MANAGE__H__

#include <atomic>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <mutex>
#include <functional>
#include <condition_variable>
#include "var/var.h"
#include "etc/etc_observer.h"
#include "common/any.h"

namespace std {
    class thread;
}

namespace comwise {
namespace drv {
  class drv_base;
}
namespace srv {

class runner_service : public etc::etc_observer
{
public:
    // basic type
    using pos_t = pose_t;
    using etc_arg_ptr_t = std::shared_ptr<etc::arg_t>;

    // driver type
    using drv_id_t = std::string;
    using drv_param_t = std::shared_ptr<etc_obj_t>;
    using drv_base_t = drv::drv_base;
    using drv_ptr_t = std::shared_ptr<drv_base_t>;
    using drv_vector_t = std::vector<drv_ptr_t>;
    using drv_id_list_t = std::map<drv_id_t, drv_ptr_t>;
    using drv_type_list_t = std::map<uint32_t, drv_id_list_t>;

public:
    explicit runner_service();
    virtual ~runner_service();

    // service 
    int init();
    int deinit();

    int start(const std::string &id = kEmptyStr);
    int stop(const std::string &id = kEmptyStr);

    //! param update callback
    virtual bool update(etc_arg_ptr_t arg = nullptr) override;

protected:
    virtual bool register_msg(uint32_t type, drv_ptr_t drv) { return true; }
    virtual bool unregister_msg(uint32_t type) { return true; }

private:
    void start_thread();

    int update_param(etc_arg_ptr_t &arg);
    int update_local_param(const drv_id_t &id, const drv_param_t &cfg);
    int update_web_param(const drv_id_t &id, const drv_param_t &cfg);

protected:
    // driver list
    drv_id_list_t drv_ids_;
    drv_type_list_t drv_types_;
    std::mutex drv_mtx_;

    // runner thread (not currently in use)
    std::unique_ptr<std::thread> runner_thr_{nullptr};
    std::atomic_bool is_runner_loop_{false};

    // work thread (update param var)
    etc_arg_ptr_t drv_param_;
    std::unique_ptr<std::thread> worker_thr_{nullptr};
    std::atomic_bool is_loop_{false};
    std::mutex update_mtx_;
    std::condition_variable update_cv_;
};

class hardware_manage : public runner_service
{
public:
    using data_cb_t = std::function<void(const std::string &id, const any &data)>;
public:
    explicit hardware_manage();
    virtual ~hardware_manage();

    virtual bool register_msg(uint32_t type, drv_ptr_t drv) override;
    virtual bool unregister_msg(uint32_t type) override;

    void set_data_cb(const data_cb_t &cb) { data_cb_ = cb; }

    //!> find driver object by id
    drv_id_list_t get_drv(const std::string &id);
    //!> get driver object by type
    drv_id_list_t get_drv(int32_t type);
    //!> find all driver object
    drv_type_list_t get_drvs();

private:
    data_cb_t data_cb_{nullptr};
};

} // namespace srv
} // namespace comwise
 
#endif // __COMWISE_SRV__RUNNER_MANAGE__H__
