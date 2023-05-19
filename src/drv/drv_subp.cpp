#include "drv/drv_subp.h"
#include "common/make_thread.h"
#include "process/subp_ctrler.h"
#include "log/log.h"
#include "etc/etc_default.h"

namespace comwise {
namespace drv {

drv_subp::drv_subp(drv_id_t id, drv_type_t type)
    : drv_impl(id, type)
{

}

drv_subp::~drv_subp()
{
    deinit();
}

int drv_subp::init(const drv_param_t &param)
{
    int ret = RET_OK;

    // 1\ if driver is inited
    if (get_state() >= kStateReady) {
        LOG_STREAM_INFO << "driver object(" << get_id() << ") had inited";
        return RET_OK;
    }

    // 2\ set config
    LOG_STREAM_INFO << "init driver object(" << get_id() << ") ... ";
    publish("init driver", "uninitialized", status_obj_t::ERROR);
    ret = update(param);
    LOG_STREAM_INFO << "init driver object(" << get_id() << "), ret = " << ret;

    set_state(ret==RET_OK? kStateReady : kStateIdle);
    
    return ret;
}

int drv_subp::deinit()
{
    if (get_state() <= kStateIdle) {
        return RET_OK;
    }

    is_loop_ = false;

    int ret = stop();

    if (drv_thr_ && drv_thr_->joinable()) {
        drv_thr_->join();
    }
    drv_thr_ = nullptr;

    subp_ctrler_ = nullptr;

    set_state(kStateIdle);

    LOG_STREAM_INFO << "deinit driver object(" << get_id() << ") ok";

    return ret;
}

int drv_subp::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOG_STREAM_INFO << "set driver object(" << get_id() << ") config ... ";

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<etc_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default config";
            cfg_ptr = etc::etc_default::get_drv_default(get_type());
            publish("update config", ss.str(), status_obj_t::WARN);
        }

        // 2\ save config
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") param is still nullptr, please check type and config";
            publish("update config", ss.str(), status_obj_t::ERROR);
            return RET_OBJECT_IS_NULL;
        }
        drv_cfg_ = cfg_ptr;

        // 3\ init sub process
        std::string sub_name = drv_cfg_->name;
        std::string sub_cmd = "/bin/bash";
        std::string sub_arg = etc::etc_default::get_node_arg(cfg_ptr);
        subp_args_.clear();
        subp_args_.push_back("-c");
        subp_args_.push_back(sub_arg);
        LOG_STREAM_INFO << "subp args = " << sub_arg;

        if(nullptr == subp_ctrler_) {
            subp_ctrler_ = std::move(std::make_shared<process::subp_ctrler>(sub_name, sub_cmd));
        } else {
            subp_ctrler_->set_name(sub_name);
            subp_ctrler_->set_args(subp_args_);
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

int drv_subp::set_config(const drv_param_t &cfg)
{
    int ret = RET_OK;

    LOG_STREAM_INFO << "driver object(" << get_id() << "), before_status = " << get_state();
    ret = update(cfg, false);
    if(ret != RET_OK ) {
        LOG_STREAM_ERROR << "set driver object(" << get_id() << ") config error, ret = " << ret;
        return RET_SET_PARAM_ERROR;
    } else {
        set_state(kStateReady);
    }
    LOG_STREAM_INFO << "driver object(" << get_id() << "), after_status = " << get_state();

    return ret;
}

int drv_subp::get_config(drv_param_t &cfg)
{
    cfg = drv_cfg_;
    return RET_OK;
}

int drv_subp::start()
{
    int ret = RET_OK;

    if (nullptr != drv_thr_) {
        if(get_state() == kStatePause) {
            set_state(kStateReady);
        }
        LOG_STREAM_WARN << "thread(" << get_id() << ") is already running，status = " << get_state();
        return ret;
    }
    is_loop_ = true;
    drv_thr_ = std::move(common::make_thread(get_id(), [&]() {
    try {
        while (is_loop_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            switch (get_state()) {
            case kStateIdle:
            {
                LOG_STREAM_ERROR << "driver object(" << get_id() << ") is not init";
                ret = RET_OBJECT_IS_NOINIT;
                break;
            }
            case kStateReady:
            {
                int sub_ret = _start();
                LOG_STREAM_WARN << "exit subprocess(" << get_id() << "," << get_state() << ") return = " << sub_ret << ", check if restart?";
                break;
            }
            case kStateRunning:
            case kStatePause:
                break;
            case kStateExit:
            {
                ret = RET_OK;
                LOG_STREAM_INFO << "exit subprocess(" << get_id() << "," << get_state()<< ") normal";
                return ret;
            }
            default:
                LOG_STREAM_ERROR << "thread(" << get_id() << ") have unknown status: " << get_state();
                break;
            }
        }
        ret = RET_OK;
        LOG_STREAM_INFO << "thread(" << get_id() << ") quit loop normal";
    } catch (const std::exception &e) {
        std::stringstream ss;
        ss << "start thread(" << get_id() << ") exception, " << e.what();
        publish("start driver", ss.str(), status_obj_t::ERROR);
        ret = RET_RUN_THREAD_EXCEPTION;
    }
    }));

    LOG_STREAM_INFO << "start thread(" << get_id() << ") success";

    return ret;
}

int drv_subp::stop()
{
    return  _stop();
}

int drv_subp::_start()
{
    std::stringstream ss;

    // 1\if can running
    if(get_state() >= kStateRunning) {
        LOG_STREAM_WARN << "driver object(" << get_id() << ") is running";
        return RET_OK;
    }

    // 2\ set subp_ctrler args
    if(nullptr == subp_ctrler_) {
        ss.str("");
        ss << "start object(" << get_id() << ") error, subp_ctrler object is nullptr";
        publish("start subprocess", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }
    subp_ctrler_->set_args(subp_args_);

    // 3\ start sub process
    if (!subp_ctrler_->start()) {
        ss.str("");
        ss << "start sub process(" << get_id() << ") error";
        publish("start subprocess", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);
    publish("start driver", "start driver ok");

    // 4\ waiting for end
    LOG_STREAM_INFO << "waiting for sub process(" << get_id() << ") to end ...";
    int sub_ret = subp_ctrler_->wait_for_end(); // block here
    if(sub_ret != 0) {
        LOG_STREAM_ERROR << "exit sub process(" << get_id() << ") error, return = " << sub_ret;
    } else {
        LOG_STREAM_INFO << "exit sub process(" << get_id() << ") ok";
    }
    if((get_state() != kStatePause) && (get_state() != kStateIdle)) {
        set_state(kStateReady);
    }
    if(false == is_loop_) {
        LOG_STREAM_INFO << "exit sub process(" << get_id() << ", " << get_state() << "), set exit flag";
        set_state(kStateExit);
    } else {
        std::string info = std::string("exit subprocess(") + get_id() + "), please check it";
        publish("start driver", info, status_obj_t::WARN);    
    }
    return sub_ret != 0 ? RET_RUN_SUB_PROCESS_ERROR : RET_OK;
}

int drv_subp::_stop()
{
    bool ret = false;

    // 1\ if can stop
    if (get_state() == kStatePause || get_state() < kStateReady) {
        return RET_OK;
    }
    
    // 2\ pause thread, reset status
    if(get_state() >= kStateReady) {
        set_state(kStatePause);
    }

    // 3\ stop subp running
    if(subp_ctrler_) {
        ret = subp_ctrler_->stop();
    }
    if(ret) {
        LOG_STREAM_INFO << "stop driver object(" << get_id() << ") ok";
    } else {
        std::stringstream ss;
        ss << "stop driver object(" << get_id() << ") error";
        publish("stop driver", ss.str(), status_obj_t::WARN);
    }
    
    return ret? RET_OK : RET_ERROR;
}

int drv_subp::read(const drv_request_t &request, drv_reply_t reply)
{
    int ret = RET_ERROR;
    if (nullptr == request) {
        LOG_STREAM_ERROR << "driver(" << get_id() << ") read input param object is nullptr";
        return ret;
    }
    return ret;
}

int drv_subp::write(const drv_request_t &request, drv_reply_t reply)
{
    int ret = RET_ERROR;
    return ret;
}

int drv_subp::publish(const std::string &name, const std::string &msg, int level, const std::string &suggest)
{
    std::string drv_type = etc::etc_default::get_drv_type(get_type());

    status_data_t data;
    data.stamp = std::chrono::system_clock::now().time_since_epoch().count();
    data.name = get_id();
    data.message = msg;
    data.level = level;
    data.suggest = suggest;

    data.dev_id = drv_type;
    data.dev_type = drv_type;
    data.dev_status = get_state();

    if(!name.empty()) {
        data.values[name] = msg;
    }

    emit("status", data);

    switch (level) {
    case status_obj_t::WARN:
        LOG_STREAM_WARN << get_id() << ", " << msg;
        break;
    case status_obj_t::ERROR: 
        LOG_STREAM_ERROR << get_id() << ", " << msg;
        break;
    case status_obj_t::FATAL:
        LOG_STREAM_ERROR << get_id() << ", " << msg;
        break;
    default:
        //LOG_STREAM_INFO << get_id() << ", " << msg;
        break;
    }

    if(status_) {
        *status_ =  data;
        status_->dev_id = drv_cfg_? drv_cfg_->name : get_id();
        status_->name = "";
    }

    return RET_OK;
}

} // namespace drv
} // namespace comwise

