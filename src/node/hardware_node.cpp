#include "node/hardware_node.h"
#include "var/var.h"
#include "log/log.h"
#include "common/linux_chcwd.h"
#include "etc/etc_subject.h"
#include "etc/etc_observer.h"
#include "etc/etc_provider.h"
#include "srv/hardware_manage.h"
#include "srv/hardware_interface.h"
#include "node/hardware_ros.h"

namespace comwise {
namespace node {

hardware_node::hardware_node(const std::string &filename, bool node_type)
    : cfg_file_(filename)
    , is_ros_(node_type)
    , is_init_(false)
    , is_deinit_(false)
{

}

hardware_node::~hardware_node()
{
    deinit();
}

bool hardware_node::init()
{
    if (is_init_) {
        return true;
    }

    LOGP_INFO("init hardware_node ... ");

    cfg_file_ = common::get_absolute_path(cfg_file_);

    // create module
    if (!create_object()) {
        LOGP_ERROR("create objecte error, please view log");
        return false;
    }

    // init module
    if (!init_object()) {
        LOGP_ERROR("init objecte error, please view log");
        return false;
    }
    is_init_ = true;

    LOGP_INFO("init hardware_node ok");

    return is_init_;
}

bool hardware_node::deinit()
{
    if (is_deinit_) {
        return true;
    }

    LOGP_INFO("deinit hardware_node object ...");

    if (etc_provider_) {
        etc_provider_->deinit();
    }
    etc_provider_ = nullptr;

    if (runner_srv_) {
        is_deinit_ = (RET_OK == runner_srv_->deinit());
    }

    etc_subject_.reset();
    etc_observer_.reset();
    runner_interface_.reset();
    runner_srv_.reset();

    if (is_ros_ && runner_ros_) {
        runner_ros_->deinit();
    }

    LOGP_INFO("deinit hardware_node %s", kRetBoolStr[is_deinit_]);

    return is_deinit_;
}

bool hardware_node::create_object()
{
    if (!runner_srv_) {
        runner_srv_ = std::make_shared<srv::hardware_manage>();
    }

    if (!etc_subject_) {
        etc_subject_ = std::make_shared<etc::etc_subject>();
    }

    if (!etc_observer_) {
        etc_observer_ = std::make_shared<etc::etc_observer>("etc_param");
    }

    if (!etc_provider_) {
        etc_provider_ = std::make_shared<etc::etc_provider>(cfg_file_, etc_subject_);
    }

    if (!runner_interface_) {
        runner_interface_ = std::make_shared<srv::hardware_interface>(runner_srv_, etc_provider_);
    }

    if (!runner_srv_ || !runner_interface_ || !etc_subject_ || !etc_observer_ || !etc_provider_) {
        LOGP_ERROR("create object is nullptr, please view log");
        return false;
    }

    if (etc_subject_) {
        etc_subject_->register_observer(etc_observer_);
        etc_subject_->register_observer(runner_srv_);
    }

    if (is_ros_ && !runner_ros_) {
        runner_ros_ = std::unique_ptr<hardware_ros>(new hardware_ros(runner_interface_));
    }

    return true;
}

bool hardware_node::init_object()
{
    bool ret = true;
    if (is_ros_ && runner_ros_) {
        ret &= runner_ros_->init();
        if(!ret) {
            LOGP_ERROR("init hardware_ros error");
            return false;
        }
    }

    if (!runner_srv_ || !etc_provider_) {
        LOGP_ERROR("runner_service/etc_provider object is nullptr");
        return false;
    }

    ret &= (RET_OK == runner_srv_->init());
    if (!ret) {
        LOGP_ERROR("init runner_service error");
        return false;
    }

    etc_provider_->init();

    return ret;
}

bool hardware_node::start()
{
    bool ret = true;
    LOGP_INFO("waiting for component ready ...");
    return ret;
}

bool hardware_node::stop()
{
    bool ret = true;
    try {
        int hw_ret = RET_ERROR;
        if (runner_srv_) {
            hw_ret = runner_srv_->stop();
        }
        if (hw_ret == RET_ERROR) {
            LOGP_ERROR("stop runner_service failed, ret = %d", ret);
            return false;
        }
        LOGP_INFO("stop runner_service ok");
    } catch (const std::exception &e) {
        LOGP_ERROR("stop runner_service exception, %s", e.what());
        ret = false;
    }
    return ret;
}

} // namespace node
} // namespace comwise
