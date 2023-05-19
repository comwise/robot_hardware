#include "srv/hardware_manage.h"
#include "log/log.h"
#include "common/make_thread.h"
#include "drv/drv_factory.h"

namespace comwise {
namespace srv {

runner_service::runner_service()
    : etc_observer("runner_service")
{
    init();
}

runner_service::~runner_service()
{
    deinit();
}

int runner_service::init()
{
    if (nullptr == worker_thr_) {
        start_thread();
    }
    return 0;
}

int runner_service::deinit()
{
    is_runner_loop_ = false;
    if (runner_thr_ && runner_thr_->joinable()) {
        runner_thr_->join();
    }
    runner_thr_ = nullptr;
    return 0;
}

int runner_service::start(const std::string &id)
{
    if (id.empty()) {
        for (auto &cit : drv_ids_) {
            auto &obj = cit.second;
            if (obj) {
                obj->start();
            }
        }
    } else {
        auto cit = drv_ids_.find(id);
        if (cit != drv_ids_.end()) {
            auto &obj = cit->second;
            if (obj) {
                obj->start();
            }
        }
    }
    return 0;
}

int runner_service::stop(const std::string &id)
{
    if (id.empty()) {
        for (auto &cit : drv_ids_) {
            auto &obj = cit.second;
            if (obj) {
                obj->stop();
            }
        }
    } else {
        auto cit = drv_ids_.find(id);
        if (cit != drv_ids_.end()) {
            auto &obj = cit->second;
            if (obj) {
                obj->stop();
            }
        }
    }
    return 0;
}

bool runner_service::update(etc_arg_ptr_t arg)
{
    bool ret = true;
    try {
        std::lock_guard<std::mutex> lk(update_mtx_);
        drv_param_ = arg;
        update_cv_.notify_one();

    } catch (const std::exception &e) {
        LOG_STREAM_ERROR << "start hardware service exception, " << e.what();
        ret = false;
    }
    return ret;
}

void runner_service::start_thread()
{
    runner_thr_ = std::move(common::make_thread("runner_thr", [&]() {
        is_runner_loop_ = true;
        while (is_runner_loop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }));

    worker_thr_ = std::move(common::make_thread("worker_thr", [&] {
        is_loop_ = true;
        while (is_loop_) {
            std::unique_lock<std::mutex> lck(update_mtx_);
            update_cv_.wait(lck);
            if(!is_loop_) {
                break;
            }
            update_param(drv_param_);
        }
    }));
}

int runner_service::update_param(etc_arg_ptr_t &arg)
{
    LOG_STREAM_INFO << "update runner_service param ...";

    //1 get param
    auto cfg_ptr = std::dynamic_pointer_cast<etc::etc_arg_t>(arg);
    if (nullptr == cfg_ptr) {
        LOG_STREAM_ERROR << "get etc_obj_t error";
        return RET_SRV_HW_SERVICE_INIT_PARAM_OBJECT_IS_NULL;
    }

    int state = get_state();
    int ret = RET_OK;
    for (auto &cit : cfg_ptr->param) {

        //! 1\get param object
        auto &drv_id = cit.first;
        auto &drv_param = cit.second;
        if (nullptr == drv_param) {
            LOG_STREAM_ERROR << "get object(" << drv_id << ") config param is nullptr";
            continue;
        }

        if (drv_param->major_type == kHwCalib) {
            continue;
        }

        if(state == core::kStateChangedDynamic && drv_param && drv_param->update == kUpdateChanged) {
            ret = update_web_param(drv_id, drv_param);
        } else {
            ret = update_local_param(drv_id, drv_param);
        }
        LOG_STREAM_INFO << "update object param(" << drv_id << "), return = " << ret;
    }
    LOG_STREAM_INFO << "update runner_service param finished, size = " << cfg_ptr->param.size();

    return ret;
}

//!> 2   check driver object
//!> 2.1 check drv_update
//!> 2.2 check drv_state
//!> 2.3 init/start object
int runner_service::update_local_param(const drv_id_t &drv_id, const drv_param_t &drv_cfg)
{
    int ret = RET_OK;

    if (drv_id.empty() || nullptr == drv_cfg) {
        LOG_STREAM_ERROR << "param(" << drv_id << ") is error";
        return RET_ERROR;
    }

    uint32_t major_type = drv_cfg->major_type;
    uint32_t drv_state = drv_cfg->state;
    uint32_t drv_update = drv_cfg->update;

    auto drv_cit = drv_ids_.find(drv_id);
    switch (drv_update)
    {
    case kUpdateNone: // now use
        break;
    case kUpdateChanged:
    {
        LOG_STREAM_INFO << "chang drv object(" << drv_id << "), state = ("
                 << drv_state << "," << drv_update << ")";
        if (drv_cit == drv_ids_.end()) {
            drv_ptr_t obj = drv::drv_factory::create_object(major_type, drv_id);
            if (nullptr == obj) {
                LOG_STREAM_ERROR << "create object(" << drv_id << ") failed";
                return RET_SRV_HW_SERVICE_INIT_CREATE_DRV_OBJECT_ERROR;
            }
            {
                std::lock_guard<decltype(drv_mtx_)> guard(drv_mtx_);
                drv_types_[major_type][drv_id] = obj;
                drv_ids_[drv_id] = obj;
            }
        }
        ret = drv_cit->second->deinit();
        if (ret != RET_OK) {
            LOG_STREAM_ERROR << "deinit object(" << drv_id << ") failed, ret_code = " << ret;
            return RET_SRV_HW_SERVICE_UNINIT_OBJECT_ERROR;
        }
        ret = drv_cit->second->init(drv_cfg);
        if (ret != RET_OK) {
            LOG_STREAM_WARN << "int object(" << drv_id << ") failed, ret_code = " << ret;
            //return RET_SRV_HW_SERVICE_INIT_DRV_OBJECT_ERROR;
        }
        register_msg(major_type, drv_cit->second);
        if ((drv_state & kDriverEnable) != 0 && (drv_state & kDriverStart) != 0) {
            ret = drv_cit->second->start();
            if (ret != RET_OK) {
                LOG_STREAM_ERROR << "start object(" << drv_id << ") failed, ret_code = " << ret;
                return RET_SRV_HW_SERVICE_START_OBJECT_ERROR;
            }
        }
        ret = RET_OK;
        break;
    }
    case kUpdateDelete:
    {
        LOG_STREAM_INFO << "remove drv object(" << drv_id << "), state = ("
                 << drv_state << "," << drv_update << ")";
        if (drv_cit != drv_ids_.end()) {
            ret = drv_cit->second->deinit();
            if (ret != RET_OK) {
                LOG_STREAM_ERROR << "deinit object(" << drv_id << ") failed, ret_code = " << ret;
                return RET_SRV_HW_SERVICE_UNINIT_OBJECT_ERROR;
            }
            {
                std::lock_guard<decltype(drv_mtx_)> guard(drv_mtx_);
                drv_ids_.erase(drv_id);
                drv_types_[major_type].erase(drv_id);
            }
        } else {
            LOG_STREAM_ERROR << "can't find the drv object which is deleted, id = " << drv_id;
        }
        ret = RET_OK;
        break;
    }
    case kUpdateAdd:
    {
        LOG_STREAM_INFO << "add drv object(" << drv_id << "), state = ("
                 << drv_state << "," << drv_update << ")";
        if (drv_cit != drv_ids_.end()) {
            LOG_STREAM_ERROR << "find the drv object which is added, id = " << drv_id;
            break;
        }

        drv_ptr_t obj = drv::drv_factory::create_object(major_type, drv_id);
        if (nullptr == obj) {
            LOG_STREAM_ERROR << "create object(" << drv_id << ") failed";
            return RET_SRV_HW_SERVICE_INIT_CREATE_DRV_OBJECT_ERROR;
        }

        ret = obj->init(drv_cfg);
        if (ret != RET_OK) {
            LOG_STREAM_WARN << "int object(" << drv_id << ") failed, ret_code = " << ret;
            //return RET_SRV_HW_SERVICE_INIT_DRV_OBJECT_ERROR;
        }
        register_msg(major_type, obj);

        if ((drv_state & kDriverEnable) != 0 && (drv_state & kDriverStart) != 0) {
            ret = obj->start();
            if (ret != RET_OK) {
                LOG_STREAM_ERROR << "start object(" << drv_id << ") failed, ret_code = " << ret;
                return RET_SRV_HW_SERVICE_START_OBJECT_ERROR;
            }
        }
        {
            std::lock_guard<decltype(drv_mtx_)> guard(drv_mtx_);
            drv_types_[major_type][drv_id] = obj;
            drv_ids_[drv_id] = obj;
        }
        ret = RET_OK;
        break;
    }
    default:
        break;
    }

    return ret;
}

int runner_service::update_web_param(const drv_id_t &drv_id, const drv_param_t &drv_cfg)
{
    int ret = RET_ERROR;
    if (drv_id.empty() || nullptr == drv_cfg) {
        LOG_STREAM_ERROR << "drv param(" << drv_id << ") is error";
        return RET_OBJECT_IS_NULL;
    }

    //1 get drv object
    auto cit = drv_ids_.find(drv_id);
    if (cit == drv_ids_.end()) {
        LOG_STREAM_ERROR << "can't find drv object, id = " << drv_id;
        return RET_OBJECT_IS_NO_FOUND;
    }

    auto obj = cit->second;
    if (nullptr == obj) {
        LOG_STREAM_ERROR << "drv object is nullptr, id = " << drv_id;
        return RET_OBJECT_IS_NULL;
    }

    //2 check if stop drv object
    uint32_t drv_state = drv_cfg->state;
    if (drv_cfg->is_restart && (drv_state & kDriverEnable) != 0) {
        ret = obj->stop();
        if (ret != RET_OK) {
            LOG_STREAM_WARN << "stop drv object(" << drv_id << ") error, return = " << ret;
        } else {
            LOG_STREAM_INFO << "stop drv object(" << drv_id << ") success";
        }
    }

    //3 set config
    ret = obj->set_config(drv_cfg);
    if (ret != RET_OK) {
        LOG_STREAM_ERROR << "set drv param error, id = " << drv_id;
        return RET_SET_PARAM_ERROR;
    } else {
        LOG_STREAM_INFO << "set drv object(" << drv_id << ") param success";
    }

    //4 start object
    if (drv_cfg->is_restart && ((drv_state & kDriverEnable) != 0 && (drv_state & kDriverStart) != 0)) {
        ret = obj->start();
        if (ret != RET_OK) {
            LOG_STREAM_WARN << "start drv object(" << drv_id << ") error, return = " << ret;
        } else {
            LOG_STREAM_INFO << "start drv object(" << drv_id << ") success";
        }
    }
    
    return ret;
}

hardware_manage::hardware_manage()
    : runner_service()
{

}

hardware_manage::~hardware_manage()
{

}

bool hardware_manage::register_msg(uint32_t type, drv_ptr_t drv)
{
    if (nullptr == drv) {
        return false;
    }

    switch (type)
    {
    case kHwChassis:
    {
        drv->on("move_feedback", [&](const common::any &data) {
#ifdef DEBUG_DATA
            auto vel = common::any_cast<vel_t>(data);
            printf("[srv] move feedback: v=%f, w=%f\n", vel.v, vel.w);
#endif
            if (data_cb_) {
                data_cb_("move_feedback", data);
            }
        });
        break;
    }
    case kHwJoystick:
    {
        drv->on("joy_data", [&](const common::any &data) {
#ifdef DEBUG_DATA
            auto joy = common::any_cast<joy_data_t>(data);
            printf("[srv] joy feedback: buttons=%d, axes=%d\n",
                joy.buttons.size(), joy.axes.size());
#endif
            if (data_cb_) {
                data_cb_("joy_data", data);
            }
        });
        break;
    }
    
    default:
        break;
    }
    return true;
}

bool hardware_manage::unregister_msg(uint32_t type)
{

}

hardware_manage::drv_id_list_t hardware_manage::get_drv(const std::string &id)
{
    if (id.empty()) {
        return drv_ids_;
    } else {
        drv_id_list_t temp_list;
        {
            std::lock_guard<decltype(drv_mtx_)> lck(drv_mtx_);
            temp_list = drv_ids_;
        }
        drv_id_list_t drv_list;
        auto cit = temp_list.find(id);
        if (cit != temp_list.end()) {
            drv_list[cit->first] = cit->second;
        }
        return drv_list;
    }
}

hardware_manage::drv_id_list_t hardware_manage::get_drv(int32_t type)
{

    if (type < 0) {
        return drv_ids_;
    } else {
        drv_type_list_t temp_list;
        {
            std::lock_guard<decltype(drv_mtx_)> lck(drv_mtx_);
            temp_list = drv_types_;
        }
        drv_id_list_t drv_list;
        auto cit = temp_list.find(type);
        if (cit != temp_list.end()) {
            drv_list = cit->second;
        }
        return drv_list;
    }
}

hardware_manage::drv_type_list_t hardware_manage::get_drvs()
{
    std::lock_guard<decltype(drv_mtx_)> lck(drv_mtx_);
    return drv_types_;
}

} // namespace srv
} // namespace comwise