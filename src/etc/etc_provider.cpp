#include "etc/etc_provider.h"
#include "log/log.h"
#include "etc/etc_pool.h"
#include "etc/etc_data.h"
#include "etc/etc_subject.h"
#include "etc/etc_launch.h"
#include "etc/etc_template.h"

namespace comwise {
namespace etc {

namespace {
    static const char* kLaunchFile   = "/etc/launch";
    static const char* kTemplateFile = "/etc/hardware_template.json";
    static const char* kConfigFile   = "/etc/hardware_config.json";
}

etc_provider::etc_provider(const std::string &file, std::shared_ptr<etc_subject> obj)
    : cfg_file_(file)
    , etc_subject_(obj)
{

}

etc_provider::~etc_provider()
{
    deinit();
}

//! init/deinit
bool etc_provider::init()
{
    bool ret = true;
    //1\ get path
    std::string dir = common::get_exe_dir();
    auto pos = dir.find("bin");
    if (pos != std::string::npos) {
        dir = dir.substr(0, pos-1);
    }
    launch_file_ = dir + kLaunchFile;
    template_file_ = dir + kTemplateFile;
    cfg_file_ = dir + kConfigFile;

    //2\ init etc parse
    if (nullptr == etc_launch_) {
        etc_launch_ = std::make_shared<etc_launch>(launch_file_);
    }

    if (nullptr == etc_template_) {
        etc_template_ = std::make_shared<etc_template>(template_file_);
    }

    if (nullptr == etc_data_) {
        etc_data_ = std::make_shared<etc_data>(cfg_file_);
    }

    if (etc_data_) {
        etc_data_->set_notify_callback(std::bind(&etc_provider::data_notify,
            this, std::placeholders::_1));
    }
    
    ret &= init_config();
    ret &= init_component();

    return ret;
}

bool etc_provider::init_config()
{
    bool ret = true;
    //1 read launch file
    std::string err;
    bool launch_ret = false;
    if (etc_launch_) {
        launch_ret = etc_launch_->load(err);
    }
    if (!launch_ret) {
        LOGS_ERROR << "read launch file error, " << err;
    } else {
        LOGS_INFO << "read launch file success, file = " << launch_file_;
    }
    ret &= launch_ret;

    //2 read template config file
    bool template_ret = false;
    if (etc_template_) {
        template_ret = etc_template_->load();
    }
    LOGS_INFO << "read template file " << 
        kRetBoolStr[template_ret] << ", file = " << template_file_;
    ret &= template_ret;

    //3 read json config file
    bool parse_ret = false;
    if (etc_data_) {
        parse_ret = etc_data_->load();
    }
    LOGS_INFO << "read config file " << 
        kRetBoolStr[parse_ret] << ", file = " << cfg_file_;
    ret &= parse_ret;

    return ret;
}

bool etc_provider::init_component()
{
    //! notify all observer
    auto arg_ptr = std::make_shared<etc_arg_t>();
    if (nullptr != arg_ptr) {
        const etc_param_t &data = ETC_POOL->get_param();
        arg_ptr->set_param(data);
        if (etc_subject_) {
            etc_subject_->set_data(core::kStateChangedNode, arg_ptr);
        }
    }
    return true;
}

bool etc_provider::deinit()
{
    return true;
}

etc_param_t etc_provider::get_calib_param() const
{
    etc_param_t param;
    if (etc_data_) {
        param = etc_data_->get_calib_param();
    }
    return param;
}

void etc_provider::set_calib_param(const etc_param_t &param)
{
    if (etc_data_) {
        etc_data_->set_calib_param(param);
    }
}

etc_value_t etc_provider::get_calib_param(const etc_id_t &id) const
{
    return etc_data_? etc_data_->get_calib_param(id) : nullptr;
}

void etc_provider::set_calib_param(const etc_id_t &id, etc_value_t param)
{
    if (etc_data_ && !id.empty()) {
        etc_data_->set_calib_param(id, param);
    }
}

std::string etc_provider::read_json_param(const std::string &request)
{
    if (nullptr == etc_data_) {
        LOGP_ERROR("etc_data object is nullptr");
        return "";
    }

    std::string response;
    if (!etc_data_->read(request, response)) {
        LOGP_ERROR("etc_data read data error, request = %s", request.c_str());
    }

    return response;
}

int etc_provider::write_json_param(const std::string &request)
{
    if (nullptr == etc_data_) {
        LOGP_ERROR("etc_data object is nullptr");
        return RET_OBJECT_IS_NULL;
    }

    int ret = RET_OK;
    if (!etc_data_->write("", request)) {
        LOGP_ERROR("etc_data write json object error");
        ret = RET_ERROR;
    }

    return ret;
}

void etc_provider::data_notify(const etc_param_t &data)
{
    auto arg_ptr = std::make_shared<etc_arg_t>();
    if (nullptr != arg_ptr) {
        arg_ptr->set_param(data);
        if (etc_subject_) {
            etc_subject_->set_data(core::kStateChangedDynamic, arg_ptr);
        }
    }
}

std::string etc_provider::read_template(const std::string &major, const std::string &minor)
{
    std::string response;
    if (nullptr == etc_template_) {
        LOGP_ERROR("etc_template object is nullptr, param = %s", major.c_str());
        return response;
    }

    if (etc_template_->read(major, minor, response)) {
        LOGP_INFO("read template param ok, param = %s", major.c_str());
    } else {
        LOGP_ERROR("read template param failed, param = %s", major.c_str());
    }

    return response;
}

std::string etc_provider::read_comment(const std::string &major, const std::string &minor)
{
    std::string response;
    if (nullptr == etc_template_) {
        LOGP_ERROR("etc_template object is nullptr, param = (%s, %s)",
            major.c_str(), minor.c_str());
        return response;
    }

    if (etc_template_->read(major, minor, response)) {
        LOGP_INFO("read template comment ok, param = (%s, %s)", major.c_str(), minor.c_str());
    } else {
        LOGP_ERROR("read template comment failed, param = (%s, %s)", major.c_str(), minor.c_str());
    }

    return response;
}

} //namespace etc
} //namespace comwise
