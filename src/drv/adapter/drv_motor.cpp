#include "adapter/drv_motor.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "motor/motor_factory.h"

namespace comwise {
namespace drv {

drv_motor::drv_motor(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
    , drv_(nullptr)
{

}

drv_motor::~drv_motor()
{
}

int drv_motor::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOGP_INFO("set driver object(%s) param ... ", get_id().c_str());

        // 1\ parse parameter
        auto drv_param = std::dynamic_pointer_cast<motor_obj_t>(param);
        if (nullptr == drv_param) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            drv_param = std::dynamic_pointer_cast<motor_obj_t>(
                etc::etc_default::get_drv_default(get_type()));
            publish("update config", ss.str(), status_obj_t::WARN);
        }

        // 2\ save config
        if (nullptr == drv_param) {
            ss.str("");
            ss << "driver(" << get_id() << ") param is still nullptr, please check config";
            publish("update config", ss.str(), status_obj_t::ERROR);
            return RET_OBJECT_IS_NULL;
        }
        drv_cfg_ = drv_param;

        drv_ = MOTOR_FACTORY->create(drv_param->name, drv_param->minor_type);

        motor::motor_param_t motor_param;
        motor_param.channel  = drv_param->channel;
        motor_param.node_id  = drv_param->node_id;
        motor_param.mode     = drv_param->mode;
        motor_param.position = drv_param->position;
        if (!drv_ || 0 != drv_->init(motor_param)) {
            ss.str("");
            ss << "driver(" << get_id() << ") init can failed";
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
        LOGP_INFO("set driver object(%s) param ok", get_id().c_str());
    } else {
        LOGP_ERROR("set driver object(%s) param error, return code = %d", get_id().c_str(), ret);
    }

    return ret;
}

int drv_motor::_start()
{
    std::stringstream ss;

    // 1\if can running
    if (get_state() >= kStateRunning) {
        LOGP_WARN("driver(%s) is running", get_id().c_str());
        return RET_OK;
    }

    // 2\ set driver args
    if (nullptr == drv_) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error, driver object is nullptr";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }

    // 3\ start sub process
    if (RET_OK != drv_->start()) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    return RET_OK;
}

int drv_motor::_stop()
{
    int ret = false;

    if (drv_) {
        ret = drv_->stop();
    }
    if (RET_OK == ret) {
        LOGP_INFO("stop driver(%s) ok", get_id().c_str());
    } else {
        std::stringstream ss;
        ss << "stop driver(" << get_id() << ") error";
        publish("stop driver", ss.str(), status_obj_t::WARN);
    }
    
    return ret;
}

} // namespace drv
} // namespace comwise

