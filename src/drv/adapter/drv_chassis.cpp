#include "adapter/drv_chassis.h"
#include "common/make_thread.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "chassis/chassis_factory.h"

namespace comwise {
namespace drv {

drv_chassis::drv_chassis(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
{

}

drv_chassis::~drv_chassis()
{
    if (chassis_) {
        chassis_->deinit();
    }
    chassis_ = nullptr;
}

int drv_chassis::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOG_STREAM_INFO << "set driver object(" << get_id() << ") param ... ";

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<chassis_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            cfg_ptr = std::dynamic_pointer_cast<chassis_obj_t>(
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
        chassis_ = chassis::chassis_factory::create_object(id_, cfg_ptr->minor_type);
        if (chassis_) {
            int chassis_ret = chassis_->init(cfg_ptr);
            if(RET_OK != chassis_ret) {
                ret = RET_INIT_ERROR;
                ss.str("");
                ss << "init chassis driver(" << get_id() << ") error, ret = " << chassis_ret;
                publish("update config", ss.str(), status_obj_t::ERROR);
            }
        } else {
            ret = RET_OBJECT_IS_NULL;
        }

    } catch (const std::exception &e) {
        ss.str("");
        ss << "parse driver(" << get_id() << ") config exception, " << e.what();
        publish("update config", ss.str(), status_obj_t::ERROR);
        ret = RET_PARSE_PARAM_EXCEPTION;
    }

    if (ret == RET_OK) {
        LOG_STREAM_INFO << "set driver object(" << get_id() << ") param ok";
    } else {
        LOG_STREAM_ERROR << "set driver object(" << get_id() << ") param error, return code = " << ret;
    }

    return ret;
}

int drv_chassis::_start()
{
    std::stringstream ss;

    // 1\if can running
    if (get_state() >= kStateRunning) {
        LOGP_WARN("driver(%s) is running", get_id().c_str());
        return RET_OK;
    }

    // 2\ set driver args
    if (nullptr == chassis_) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error, driver object is nullptr";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }
    for (auto &id : listener()) {
        chassis_->on(id, [&](const common::any &data) {
            emit(id, data);
        });
    }

    // 3\ start sub process
    if (RET_OK != chassis_->start()) {
        ss.str("");
        ss << "start driver(" << get_id() << ") error";
        publish("start driver", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    return RET_OK;
}

int drv_chassis::_stop()
{
    bool ret = false;

    if (chassis_) {
        ret = chassis_->stop();
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

int drv_chassis::read(const drv_request_t &request, drv_reply_t response)
{
    switch (request->obj_type)
    {
    case kParamTypeReadSpeed:
    {
        auto resp = std::dynamic_pointer_cast<move_cmd_t>(response);
        if (!resp) {
            resp = std::make_shared<move_cmd_t>();
        }
        vel_t vel;
        if (RET_OK == chassis_->get_speed(vel)) {
            resp->velocity.emplace_back(vel);
        }
        break;
    }
    case kParamTypeReadData:
    {
        break;
    }
    }
    return 0;
}

int drv_chassis::write(const drv_request_t &request, drv_reply_t response)
{
    switch (request->obj_type)
    {
    case kParamTypeWriteSpeed:
    {
        auto move_cmd = std::dynamic_pointer_cast<move_cmd_t>(request);
        if (nullptr == move_cmd) {
            return 0;
        }
        switch (move_cmd->type)
        {
        case kChassisDiff:
        case kChassisSteer:
        {
            chassis_->set_speed(move_cmd->velocity.front());
            break;
        }
        default:
            break;
        }
        break;
    }
    case kParamTypeReadData:
    {
        break;
    }
    default:
        break;
    }
    return 0;
}

} // namespace drv
} // namespace comwise

