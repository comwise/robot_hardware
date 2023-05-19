#include "srv/hardware_interface.h"
#include <iostream>
#include "log/log.h"
#include "srv/hardware_manage.h"
#include "drv/drv_factory.h"
#include "common/linux_chcwd.h"
#include "etc/etc_provider.h"
#include "etc/etc_default.h"
#include "json/json.h"

namespace comwise {
namespace srv {

hardware_interface::hardware_interface(
    std::shared_ptr<hardware_manage> srv_obj,
    std::shared_ptr<etc::etc_provider> etc_obj)
    : runner_service_(std::move(srv_obj))
    , etc_provider_(std::move(etc_obj))
{
    init();
}

hardware_interface::~hardware_interface()
{
    deinit();
}

void hardware_interface::init()
{
    if (runner_service_) {
        runner_service_->set_data_cb(
            [&](const std::string &id, const any &data) {
            if (data_cb_) {
                data_cb_(id, data);
            }
        });
    }
}

void hardware_interface::deinit()
{

}

int hardware_interface::get_config(const std::string &id, std::shared_ptr<etc_obj_t> &response)
{
    if (id.empty() || nullptr == runner_service_) {
        LOGP_ERROR("id is empty or hw_service object is nullptr");
        return -1;
    }
    auto obj = runner_service_->get_drv(id);
    if (obj.size() != 1) {
        return -2;
    }
    auto obj_ptr = obj[id];
    if (nullptr == obj_ptr) {
        LOGP_ERROR("get drv object is nullptr, id = %s", id.c_str());
        return -3;
    }
    auto request = std::make_shared<var_obj_t>();
    if (!request) {
        LOGP_ERROR("create param object is nullptr, id = %s", id.c_str());
        return -4;
    }
    request->obj_type = kParamTypeReadConfig;
    request->obj_id = id;
    int code = obj_ptr->read(request, response);
    if(code != RET_OK || !response) {
        LOGP_ERROR("read drv param error, id = %s", id.c_str());
        response = nullptr;
        return -5;
    }
    return RET_OK;
}

int hardware_interface::get_config(const std::string &request, std::string &response)
{
    if (nullptr == etc_provider_) {
        response = "etc_provider object is nullptr";
        LOGP_ERROR("%s", response.c_str());
        return RET_OBJECT_IS_NULL;
    }
    response = std::move(etc_provider_->read_json_param(request));
    return RET_OK;
}

int hardware_interface::set_config(const std::string &request, std::string &response)
{
    if (nullptr == etc_provider_) {
        response = "etc_provider object is nullptr";
        LOGP_ERROR("%s", response.c_str());
        return RET_OBJECT_IS_NULL;
    }

    if (request.empty()) {
        response = "set config param is empty";
        LOGP_ERROR("%s", response.c_str());
        return RET_PARAM_OBJECT_IS_ERROR;
    }

    int ret = etc_provider_->write_json_param(request);
    if(RET_OK != ret) {
        response = "write rosparam param error, code = " + std::to_string(ret);
        LOGP_ERROR("%s", response.c_str());
    }

    return ret;
}

int hardware_interface::get_template(const std::string &major, const std::string &minor, std::string &templates)
{
    if (nullptr == etc_provider_) {
        templates = "";
        LOGP_ERROR("etc_provider object is nullptr");
        return RET_OBJECT_IS_NULL;
    }
    templates = std::move(etc_provider_->read_template(major, minor));
    return RET_OK;
}

int hardware_interface::get_comment(const std::string &major, const std::string &minor, std::string &comments)
{
    if (nullptr == etc_provider_) {
        comments = "";
        LOGP_ERROR("etc_provider object is nullptr");
        return RET_OBJECT_IS_NULL;
    }
    comments = std::move(etc_provider_->read_comment(major, minor));
    return RET_OK;
}

int hardware_interface::get_component(const std::string &request, std::string &response)
{
    int hw_type = -1;
    if (!request.empty()) {
        hw_type = etc::etc_default::get_drv_type(request);
    }
    component_map_t components;
    if (RET_OK == get_component(hw_type, components)) {
        Json::Value root;
        for (auto& obj : components) {
            auto &type = obj.first;
            auto &ids = obj.second;
            for (auto &id : ids) {
                root[type].append(id);
            }
        }
        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = "";
        response = Json::writeString(builder, root);
        return RET_OK;
    } else {
        return RET_ERROR;
    }
}

int hardware_interface::get_component(const int type, component_vector_t &components)
{
    components.clear();
    auto drv_list = runner_service_->get_drv(type);
    for (auto& obj : drv_list) {
        if (!obj.first.empty() && obj.second) {
            components.emplace_back(obj.first);
        }
    }
    return RET_OK;
}

int hardware_interface::get_component(const int type, component_map_t &components)
{
    components.clear();
    auto drv_list = runner_service_->get_drv(type);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            std::string type = etc::etc_default::get_drv_type(drv->get_type());
            components[type].emplace_back(id);
        }
    }
    return RET_OK;
}

int hardware_interface::get_component(const std::string &id, component_map_t &components)
{
    components.clear();
    auto drv_list = runner_service_->get_drv(id);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            std::string type = etc::etc_default::get_drv_type(drv->get_type());
            components[type].emplace_back(id);
        }
    }
    return RET_OK;
}

int hardware_interface::get_status(const std::string &id, std::string &status)
{
    status_type_t status_list;
    if (RET_OK == get_status(id, status_list)) {
        Json::Value root;
        for (auto& obj : status_list) {
            auto &type = obj.first;
            auto &items = obj.second;
            for (auto &item : items) {
                Json::Value status_item;
                status_item[item.first] = item.second;
                root[type].append(status_item);
            }
        }
        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = "";
        status = Json::writeString(builder, root);
        return RET_OK;
    } else {
        return RET_ERROR;
    }
}

int hardware_interface::get_status(const std::string &id, status_map_t &status)
{
    status.clear();
    auto drv_list = runner_service_->get_drv(id);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            status[id] = drv->get_code();
        }
    }
    return RET_OK;
}

int hardware_interface::get_status(const std::string &id, status_type_t &status)
{
    status.clear();
    auto drv_list = runner_service_->get_drv(id);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            std::string type = etc::etc_default::get_drv_type(drv->get_type());
            status[type][id] = drv->get_code();
        }
    }
    return RET_OK;
}

int hardware_interface::get_status(const int type, status_map_t &status)
{
    status.clear();
    auto drv_list = runner_service_->get_drv(type);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            status[id] = drv->get_code();
        }
    }
    return RET_OK;
}

int hardware_interface::get_status(const int type, status_type_t &status)
{
    status.clear();
    auto drv_list = runner_service_->get_drv(type);
    for (auto& obj : drv_list) {
        auto &id = obj.first;
        auto &drv = obj.second;
        if (!id.empty() && drv) {
            std::string type = etc::etc_default::get_drv_type(drv->get_type());
            status[type][id] = drv->get_code();
        }
    }
    return RET_OK;
}

int hardware_interface::set_move_cmd(const move_cmd_t &cmd)
{
    if (nullptr == runner_service_) {
        LOGP_ERROR("hw_service object is nullptr");
        return -1;
    }
    auto obj = runner_service_->get_drv(kHwChassis);
    if (obj.size() != 1) {
        LOGP_ERROR("chassis no. config should = 1");
        return -2;
    }
    auto obj_ptr = obj.begin()->second;
    if (nullptr == obj_ptr) {
        LOGP_ERROR("driver object is nullptr");
        return -3;
    }
    auto request = std::make_shared<move_cmd_t>(cmd);
    if (!request) {
        LOGP_ERROR("create move_cmd param object error");
        return -4;
    }
    request->obj_type = kParamTypeWriteSpeed;
    int code = obj_ptr->write(request, nullptr);
    if (code != RET_OK) {
        LOGP_ERROR("write move cmd error, code = %d", code);
        return -5;
    }
    return RET_OK;
}

int hardware_interface::get_move_feedback(move_cmd_t &cmd)
{
    if (nullptr == runner_service_) {
        LOGP_ERROR("hw_service object is nullptr");
        return -1;
    }
    auto obj = runner_service_->get_drv(kHwChassis);
    if (obj.size() != 1) {
        LOGP_ERROR("chassis no. config should = 1");
        return -2;
    }
    auto obj_ptr = obj.begin()->second;
    if (nullptr == obj_ptr) {
        LOGP_ERROR("driver object is nullptr");
        return -3;
    }
    auto request = std::make_shared<var_obj_t>();
    if (!request) {
        LOGP_ERROR("create move feedback rquest param object error");
        return -4;
    }
    request->obj_type = kParamTypeReadSpeed;
    auto response = std::make_shared<move_cmd_t>();
    int code = obj_ptr->read(request, response);
    if (code != RET_OK) {
        LOGP_ERROR("read move feedback error, code = %d", code);
        return -4;
    }
    cmd = *response;

    return RET_OK;
}

} // namespace srv
} // namespace comwise