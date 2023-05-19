#include "adapter/drv_network.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "net/net_plugin.h"

namespace comwise {
namespace drv {

drv_network::drv_network(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
{

}

drv_network::~drv_network()
{
    drv_ptr_ = nullptr;
}

int drv_network::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOGP_INFO("set driver object(%s) param ... ", get_id().c_str());

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<net_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            cfg_ptr = std::dynamic_pointer_cast<net_obj_t>(
                etc::etc_default::get_drv_default(get_type()));
            publish("update config", ss.str(), status_obj_t::WARN);
        }

        // 2\ save config
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") param is still nullptr, please check config";
            publish("update config", ss.str(), status_obj_t::ERROR);
            return RET_OBJECT_IS_NULL;
        }
        drv_cfg_ = cfg_ptr;

        // 3\ set status callback
        if (nullptr == drv_ptr_)
        drv_ptr_ = std::make_shared<net::net_plugin>(id_);
        if (drv_ptr_) {
            drv_ptr_->set_network(cfg_ptr->local_dev, cfg_ptr->local_addr);
            drv_ptr_->set_status_callback([&](int level, const std::string &err) {
                publish("set network", err, level);
            });
        }
    } catch (const std::exception &e) {
        ss.str("");
        ss << "parse driver(" << get_id() << ") config exception, " << e.what();
        publish("update config", ss.str(), status_obj_t::ERROR);
        ret = RET_PARSE_PARAM_EXCEPTION;
    }

    if (ret == RET_OK) {
        LOGP_INFO("set driver(%s) param ok", get_id().c_str());
    } else {
        LOGP_ERROR("set driver(%s) param error, return code = %d", get_id().c_str(), ret);
    }

    return ret;
}

int drv_network::_start()
{
    std::stringstream ss;

    // 1\if can running
    if (get_state() >= kStateRunning) {
        LOGP_WARN("driver(%s) is running", get_id().c_str());
        return RET_OK;
    }

    // 2\ set driver args
    if (nullptr == drv_ptr_) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error, driver object is nullptr";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }

    // 3\ start sub process
    if (RET_OK != drv_ptr_->execute()) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    return RET_OK;
}

int drv_network::_stop()
{
    LOGP_INFO("stop driver(%s) ok", get_id().c_str());
    
    return RET_OK;
}

} // namespace drv
} // namespace comwise

