#ifndef __COMWISE_ETC__ETC_POOL__H__
#define __COMWISE_ETC__ETC_POOL__H__

#include <atomic>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include "common/singleton.h"
#include "etc_arg.h"

namespace comwise {
namespace etc {

class etc_pool final : public common::singleton<etc_pool>
{
public:
    using pool_key_t = std::string;
    using pool_value_t = std::string;
    using template_value_t = std::shared_ptr<comwise::template_obj_t>;
    using template_type_t = std::map<std::string, template_value_t>;
    using template_map_t = std::map<std::string, template_type_t>;

public:
    etc_pool();
    virtual ~etc_pool();

    //!> get/set all param
    etc_param_t& get_param();
    etc_param_t get_param() const;
    void set_param(const etc_param_t &param);

    //!> get/set param by id
    etc_value_t get_param(const etc_id_t &id) const;
    void set_param(const etc_id_t &id, etc_value_t param);
    void earse_param(const etc_id_t &id);

    //!> get/set ros topic/service name
    pool_value_t get_value(pool_key_t key) const;
    void set_value(pool_key_t key, const pool_value_t &value);
    void set_value(const std::map<pool_key_t, pool_value_t> &value);

    //!> get/set template value
    void set_template(const std::string &major_type,
        const std::string &minor_type, const template_value_t &value);
    bool get_template(const std::string &major_type, 
        const std::string &minor_type, template_value_t &value);
    //!> get default value
    etc_value_t get_default(const std::string &major,
        const std::string &minor = kEmptyStr);
    etc_param_t get_default(const std::string &major = kEmptyStr);

    //! get/set jagent is ready
    bool get_ready() const { return is_ready_; }
    void set_ready(bool is_ready) { is_ready_ = is_ready; }

private:
    //! param buffer object
    etc_param_t etc_param_;
    mutable std::mutex etc_mtx_;

    //! template buffer object
    template_map_t template_param_;
    mutable std::mutex template_mtx_;

    //! topic/servie name buffer
    std::map<pool_key_t, pool_value_t> topics_;
    mutable std::mutex topics_mtx_;

    //! load config is ready ?
    std::atomic_bool is_ready_{false};
};

} // namespace etc
} // namespace comwise

#define ETC_POOL comwise::etc::etc_pool::instance()

#endif // __COMWISE_ETC__ETC_POOL__H__
