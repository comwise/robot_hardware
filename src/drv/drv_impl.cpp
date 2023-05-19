#include "drv/drv_impl.h"
#include "common/make_thread.h"
#include "log/log.h"
#include "etc/etc_default.h"

namespace comwise {
namespace drv {

drv_impl::drv_impl(drv_id_t id, drv_type_t type)
    : drv_base(id, type)
    , drv_thr_(nullptr)
    , status_(new status_obj_t())
{

}

drv_impl::~drv_impl()
{
    deinit();
}

int drv_impl::init(const drv_param_t &param)
{
    int ret = RET_OK;

    // 1\ if driver is inited
    if (get_state() >= kStateReady) {
        LOGP_INFO("driver(%s) had inited", get_id().c_str());
        return RET_OK;
    }

    // 2\ set config
    LOGP_INFO("init driver(%s) ...", get_id().c_str());
    publish("init driver", "uninitialized", status_obj_t::ERROR);
    ret = update(param);
    LOGP_INFO("init driver(%s), ret = %d", get_id().c_str(), ret);

    set_state((ret==RET_OK)? kStateReady : kStateIdle);
    
    return ret;
}

int drv_impl::deinit()
{
    if (get_state() <= kStateIdle || get_state() >= kStateExit) {
        return RET_OK;
    }

    int ret = stop();

    set_state(kStateExit);
    is_loop_ = false;
    if (drv_thr_ && drv_thr_->joinable()) {
        drv_thr_->join();
    }
    drv_thr_ = nullptr;
    //set_state(kStateIdle);
    LOGP_INFO("deinit driver(%s) ok", get_id().c_str());

    return ret;
}

int drv_impl::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;
    std::stringstream ss;

    try {

        LOGP_INFO("set driver(%s) param ... ", get_id().c_str());

        // 1\ parse parameter
        auto cfg_ptr = std::dynamic_pointer_cast<etc_obj_t>(param);
        if (nullptr == cfg_ptr) {
            ss.str("");
            ss << "driver(" << get_id() << ") input param object is nullptr, use default param";
            cfg_ptr = etc::etc_default::get_drv_default(get_type());
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

int drv_impl::set_config(const drv_param_t &cfg)
{
    int ret = RET_OK;

    LOGP_INFO("set driver(%s) config, before_status = %d", get_id().c_str(), get_state());
    ret = update(cfg, false);
    if (ret != RET_OK ) {
        LOGP_ERROR("set driver(%s) config error, ret = %d", get_id().c_str(), ret);
        return RET_SET_PARAM_ERROR;
    } else {
        set_state(kStateReady);
    }
    LOGP_INFO("set driver(%s) config, after_status = %d", get_id().c_str(), get_state());

    return ret;
}

int drv_impl::get_config(drv_param_t &cfg)
{
    cfg = drv_cfg_;
    return RET_OK;
}

int drv_impl::start()
{
    int ret = RET_OK;

    if (nullptr != drv_thr_) {
        if(get_state() == kStatePause) {
            set_state(kStateReady);
        }
        LOGP_WARN("thread(%s) is already running，status = %d",
            get_id().c_str(), get_state());
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
                LOGP_WARN("driver(%s) is not init", get_id().c_str());
                ret = init(drv_cfg_);
                if (ret == RET_OK) {
                    set_state(kStateReady);
                    set_code(RET_OK);
                } else {
                    set_code(RET_INIT_ERROR);
                }
                break;
            }
            case kStateReady:
            {
                int sub_ret = _start();
                if (RET_OK == sub_ret) {
                    set_state(kStateRunning);
                    set_code(RET_OK);
                } else {
                    set_code(RET_START_ERROR);
                }
                LOGP_WARN("start driver(%s) return %d, state = %d", 
                    get_id().c_str(), sub_ret, get_state());
                break;
            }
            case kStateRunning:
            case kStatePause:
                break;
            case kStateExit:
            {
                ret = RET_OK;
                LOGP_INFO("exit driver(%s) normal, state = %d", get_id().c_str(), get_state());
                return ret;
            }
            default:
                LOGP_ERROR("running driver(%s) have unknown state: %d", get_id().c_str(), get_state());
                break;
            }
        }
        ret = RET_OK;
        LOGP_INFO("thread(%s) quit loop normal", get_id().c_str());
    } catch (const std::exception &e) {
        std::stringstream ss;
        ss << "start thread(" << get_id() << ") exception, " << e.what();
        publish("start driver", ss.str(), status_obj_t::ERROR);
        ret = RET_RUN_THREAD_EXCEPTION;
    }
    }));

    LOGP_INFO("start thread(%s) success", get_id().c_str());

    return ret;
}

int drv_impl::stop()
{
    int ret = RET_OK;

    // 1\ if can stop
    if (get_state() == kStatePause || get_state() < kStateReady) {
        return RET_OK;
    }
    
    // 2\ pause thread, reset status
    if (get_state() >= kStateReady) {
        set_state(kStatePause);
    }

    // 3\ stop subp running
    ret = _stop();
    if (RET_OK == ret) {
        LOGP_INFO("stop driver(%s) ok", get_id().c_str());
    } else {
        std::stringstream ss;
        ss << "stop driver(" << get_id() << ") error";
        publish("stop driver", ss.str(), status_obj_t::WARN);
    }
    
    return ret;
}

int drv_impl::_start()
{
    return RET_OK;
}

int drv_impl::_stop()
{   
    return RET_OK;
}

int drv_impl::read(const drv_request_t &request, drv_reply_t reply)
{
    return RET_ERROR;
}

int drv_impl::write(const drv_request_t &request, drv_reply_t reply)
{
    return RET_ERROR;
}

int drv_impl::publish(const std::string &name, const std::string &msg,
                      int level, const std::string &suggest)
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

    if (!name.empty()) {
        data.values[name] = msg;
    }

    emit("status", data);

    switch (level) {
    case status_obj_t::WARN:
        LOGP_WARN("%s, %s", get_id().c_str(), msg.c_str());
        break;
    case status_obj_t::ERROR:
    case status_obj_t::FATAL:
        LOGP_ERROR("%s, %s", get_id().c_str(), msg.c_str());
        break;
    default:
        //LOGP_INFO(get_id() << ", " << msg;
        break;
    }

    if (status_) {
        *status_ =  data;
        status_->dev_id = drv_cfg_? drv_cfg_->name : get_id();
        status_->name = "";
    }

    return RET_OK;
}

} // namespace drv
} // namespace comwise

