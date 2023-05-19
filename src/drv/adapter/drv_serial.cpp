#include "adapter/drv_serial.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "serial/serial_bridge.h"

namespace comwise {
namespace drv {

drv_serial::drv_serial(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
{

}

drv_serial::~drv_serial()
{

}

int drv_serial::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOGP_INFO("set driver object(%s) param ... ", get_id().c_str());

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<serial_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            cfg_ptr = std::dynamic_pointer_cast<serial_obj_t>(
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
        dev_ = cfg_ptr->device;

        serial::serial_param_t sparam;
        if (!SERIAL_CENTER || !SERIAL_CENTER->init(sparam)) {
            ss.str("");
            ss << "driver(" << get_id() << ") init failed";
            publish("update config", ss.str(), status_obj_t::ERROR);
            return RET_OBJECT_IS_NOINIT;
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

int drv_serial::_start()
{
    std::stringstream ss;

    // 1\if can running
    if (get_state() >= kStateRunning) {
        LOGP_WARN("driver(%s) is running", get_id().c_str());
        return RET_OK;
    }

    // 2\ set driver args
    if (nullptr == SERIAL_CENTER) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error, driver object is nullptr";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }

    // 3\ start sub process
    if (!SERIAL_CENTER->start(dev_)) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    return RET_OK;
}

int drv_serial::_stop()
{
    bool ret = false;

    if (SERIAL_CENTER) {
        ret = SERIAL_CENTER->stop(dev_);
    }
    if (ret) {
        LOGP_INFO("stop driver(%s) ok", get_id().c_str());
    } else {
        std::stringstream ss;
        ss << "stop driver(" << get_id() << ") error";
        publish("stop driver", ss.str(), status_obj_t::WARN);
    }
    
    return ret? RET_OK : RET_ERROR;
}

} // namespace drv
} // namespace comwise

