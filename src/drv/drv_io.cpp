#include "drv/drv_io.h"
#include "common/make_thread.h"
#include "common/linux_chcwd.h"
#include "process/subp_ctrler.h"
#include "process/ros_ctrler.h"
#include "log/log.h"
#include "etc/etc_default.h"
#include "net/net_plugin.h"

namespace comwise {
namespace drv {

drv_io::drv_io(drv_id_t id, drv_type_t type)
    : drv_subp(id, type)
    , net_plugin_(std::make_shared<net_plugin>())
{

}

drv_io::~drv_io()
{
    subp_ctrler_ = nullptr;
}

int drv_io::update(const drv_param_t &param, bool is_init)
{
    int ret = RET_OK;

    try {

        LOG_STREAM_INFO << "set driver object(" << get_id() << ") param ... ";

        // 1\ parse config
        init_drvparam(param);

        // 2\ init sub process
        init_subparam(drv_cfg_);

    } catch (const std::exception &e) {
        std::stringstream ss;
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

int drv_io::_start()
{
    int ret = RET_OK;
    if(drv_cfg_ && drv_cfg_->node) {
        ret = start_rosp();
    } else {
        ret = start_subp();
    }
    return ret;
}

int drv_io::_stop()
{
    bool ret = false;

    if (get_state() == kStatePause || get_state() < kStateReady) {
        return RET_OK;
    }
    
    // pause mcu thread, reset status
    if(get_state() >= kStateReady) {
        set_state(kStatePause);
    }

    // stop process running
    if(subp_ctrler_) {
        ret = subp_ctrler_->stop();
    }

    if(ros_ctrler_) {
        ret = ros_ctrler_->stop();
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

int drv_io::start_subp()
{
    return drv_impl::_start();
}

int drv_io::start_rosp()
{
    std::stringstream ss;

    // 0\ if can running
    if(get_state() >= kStateRunning) {
        LOG_STREAM_WARN << "driver object(" << get_id() << ") is running";
        return RET_OK;
    }

    if(nullptr == drv_cfg_) {
        ss.str("");
        ss << "driver(" << get_id() << ") config is nullptr, please check config";
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    }

    // 1\ create ros object
    int ret = RET_ERROR;
    std::string xml(etc::etc_default::get_xml(drv_cfg_->major_type, drv_cfg_->minor_type));
    std::string name = etc::etc_default::get_drv_type(drv_cfg_->major_type) + 
        "_" + common::get_file_name(drv_cfg_->name);
    ros_ctrler_ = std::make_shared<ros_ctrler_t>(name, xml);
    if(nullptr == ros_ctrler_) {
        ss.str("");
        ss << "create ros_ctrler_t object(" << get_id() << ") error";
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_OBJECT_IS_NULL;
    } else {
        ros_ctrler_->set_name(name);
        ros_ctrler_->set_arg(xml);
    }

    ret = init_ros_args();
    if(RET_OK != ret) {
        ss.str("");
        ss << "init ros arg error, id = " << get_id();
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_PARSE_PARAM_ERROR;
    }

    // 2\set param
    std::vector<std::string> fail_args;
    if (!ros_ctrler_->set_args(ros_args_, fail_args)) {
        std::string errs = "";
        for(auto &s: fail_args)
            errs += s + ".";
        ss.str("");
        ss << "set ros config param error, no support: " << errs;
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_SET_PARAM_ERROR;
    }

    // 3\start roslaunch
    if (!ros_ctrler_->start()) {
        ss.str("");
        ss << "start ros node(" << get_id() << ") failed";
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_RUN_SUB_PROCESS_ERROR;
    }
    set_state(kStateRunning);

    // 4\set ros service
    ret = init_ros_service();
    if(RET_OK != ret) {
        ss.str("");
        ss << "init ros service error, id = " << get_id();
        publish("start ros subprocess", ss.str(), status_obj_t::ERROR);
        return RET_INIT_ERROR;
    } else {
        publish("start driver", "start driver ok");
    }

    // 5\waiting for end
    LOG_STREAM_INFO << "waiting for sub process(" << get_id() << ") to end ...";
    int sub_ret = ros_ctrler_->wait_for_end(); // block here
    if(sub_ret != 0) {
        LOG_STREAM_ERROR << "exit sub process(" << get_id() << ") error, return = " << sub_ret;
    } else {
        LOG_STREAM_INFO << "exit sub process(" << get_id() << ") ok";
    }
    std::string info = std::string("exit subprocess(") + get_id() + "), please check reason";
    publish("start driver", info, status_obj_t::WARN); 
    if((get_state() != kStatePause) && (get_state() != kStateIdle)) {
        set_state(kStateReady);
    }
    if(is_loop_ == false) {
        set_state(kStateExit);
    } else {
        std::string info = std::string("exit subprocess(") + get_id() + "), please check it";
        publish("start ros subprocess", info, status_obj_t::WARN);  
    }
    return (sub_ret != 0)? RET_WAITING_FOR_END_PROCESS_ERROR : RET_OK;
}

int drv_io::init_ros_args()
{
    return etc::etc_default::get_ros_arg(ros_args_, drv_cfg_)? RET_OK : RET_ERROR;
}

int drv_io::init_ros_service()
{
    return RET_OK;
}

bool drv_io::init_drvparam(const drv_param_t &cfg)
{   
    std::stringstream ss;
    auto cfg_ptr = std::dynamic_pointer_cast<etc_obj_t>(cfg);
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
    return true;
}

bool drv_io::init_subparam(const drv_param_t &cfg)
{
    if (cfg && !cfg->node) {
        std::string sub_name = cfg->name;
        std::string sub_cmd = "/bin/bash";
        std::string sub_arg = etc::etc_default::get_node_arg(cfg);
        subp_args_.clear();
        subp_args_.push_back("-c");
        subp_args_.push_back(sub_arg);
        LOG_STREAM_INFO << "subp args = " << sub_arg;

        if (nullptr == subp_ctrler_) {
            subp_ctrler_ = std::move(std::make_shared<process::subp_ctrler>(sub_name, sub_cmd));
        } else {
            subp_ctrler_->set_name(sub_name);
            subp_ctrler_->set_args(subp_args_);
        }
    }
    return true;
}

} // namespace drv
} // namespace comwise

