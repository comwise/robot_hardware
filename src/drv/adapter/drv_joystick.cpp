#include "adapter/drv_joystick.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "joystick/joy_stick.h"

namespace comwise {
namespace drv {

drv_joystick::drv_joystick(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
{

}

drv_joystick::~drv_joystick()
{
    joy_stick_ = nullptr;
}

int drv_joystick::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOGP_INFO("set driver object(%s) param ... ", get_id().c_str());

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<joy_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            cfg_ptr = std::dynamic_pointer_cast<joy_obj_t>(
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

        // 3\ init sub process
        if(nullptr == joy_stick_) {
            joy_stick_ = std::move(std::make_shared<joy::joy_stick>(
                cfg_ptr->device, cfg_ptr->coalesce_interval, cfg_ptr->repeat_interval));
        } else {
            joy_stick_->set_config(cfg_ptr->device, cfg_ptr->coalesce_interval, cfg_ptr->repeat_interval);
        }

        joy_stick_->set_data_callback(std::bind(&drv_joystick::joy_data, this, std::placeholders::_1));

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

int drv_joystick::_start()
{
    std::stringstream ss;

    // 1\if can running
    if (get_state() >= kStateRunning) {
        LOGP_WARN("driver(%s) is running", get_id().c_str());
        return RET_OK;
    }

    // 2\ set driver args
    if (nullptr == joy_stick_) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error, driver object is nullptr";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }

    // 3\ start sub process
    if (!joy_stick_->start()) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    return RET_OK;
}

int drv_joystick::_stop()
{
    bool ret = false;
    if (joy_stick_) {
        ret = joy_stick_->stop();
    }
    return ret? RET_OK : RET_ERROR;
}

void drv_joystick::joy_data(const joy_data_t &data)
{
#ifdef DEBUG_JOY
    std::stringstream ss;
    ss << data.time;
    ss << " button:";
    for(auto &item : data.buttons) {
      ss << " " << item;
    }
    ss << " axes:";
    for(auto &item : data.axes) {
      ss << " " << item;
    }
    std::cout << ss.str() << std::endl;
#endif
    emit("joy_data", data);
}

} // namespace drv
} // namespace comwise

