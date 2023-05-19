#include "etc/etc_data.h"
#include <json/json.h>
#include "log/log.h"
#include "etc/etc_default.h"
#include "etc/etc_pool.h"

namespace comwise {
namespace etc {


etc_data::etc_data(const std::string &filename)
    : etc_parse(filename)
{

}

etc_data::~etc_data()
{

}

bool etc_data::parse(const json_value_t &value)
{
    bool ret = true;
    auto &pool_buffer = ETC_POOL->get_param();
    Json::Value::Members member_names = value.getMemberNames();
    for (auto &member : member_names) {
        std::string &module = member;
        uint32_t size = value[module].size();
        for (uint32_t i = 0; i < size; i++) {
            const Json::Value &obj = value[module][i];
            std::string id = obj["name"].asString();
            std::string type = obj["type"].asString();
            auto cfg = find_object(id, pool_buffer);
            uint32_t update = 0;
            if (nullptr == cfg) {
                update = kUpdateAdd;
            }
            if (obj.isMember("update")) {
                update = obj["update"].asUInt();
            }
            if (update == kUpdateDelete && cfg) {
                cfg->update = update;
                cfg->state = kDriverEnable | kDriverStop;
                update_param_[id] = cfg;
            } else {
                if (parse_value(obj, module, type, cfg) && cfg) {
                    cfg->update = update;
                    cfg->state = kDriverEnable | kDriverStart;
                    update_param_[id] = cfg;
                }
            }
        }
    }
    ETC_POOL->set_param(update_param_);
    return ret;
}

bool etc_data::pack(json_value_t &value)
{
    if (!pack_data(value)) {
        LOGP_ERROR("package data to file error, file = %s", cfg_file_.c_str());
        return false;
    }
    return true;
}

bool etc_data::pack_data(json_value_t &root, const etc_id_t &id)
{
    bool ret = true;
    auto calib_param = ETC_POOL->get_param();
    for (auto & cit : calib_param) {

        auto obj = cit.second;
        if (nullptr == obj) {
            continue;
        }

        if (!id.empty() && obj->name != id) {
            continue;
        }

        std::string module = etc_default::get_drv_type(obj->major_type);
        json_value_t val;
        if (!module.empty() && pack_value(obj, val)) {
            root[module].append(val);
        }
    }
    return ret;
}

bool etc_data::read(const std::string &id, std::string &value)
{
    bool ret = true;
    try {
        Json::Value root;
        if (!pack_data(root, id)) {
            LOGP_ERROR("pack buffer data to json error, id = %s", id.c_str());
            value = "";
            return false;
        }
        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = "";
        value = Json::writeString(builder, root);
    } catch (std::exception &e) {
        value = "";
        ret = false;
        LOGP_ERROR("read json param exception, %s", e.what());
    }
    return ret;
}

bool etc_data::write(const std::string &id, const std::string &data)
{
    int ret = RET_OK;
    try {
        Json::Reader reader;
        Json::Value value;
        if (!reader.parse(data, value)) {
           LOGP_ERROR("write json param error, %s", data.c_str());
           ret = RET_ETC_ETC_PARSE_JSON_ERROR;
        }
        if (RET_OK == ret) {
            ret = write(value, id)? RET_OK : RET_ETC_ETC_PARSE_WRITE_JSON_TO_FILE_ERROR;
        }
    } catch (std::exception &e) {
        LOGP_ERROR("write json param exception, %s", e.what());
        ret = RET_ETC_ETC_PARSE_JSON_EXCEPTION;
    }

    return ret == RET_OK;
}

bool etc_data::read(const etc_id_t &id, etc_param_t &value)
{
    bool is_find = false;
    auto params = ETC_POOL->get_param();
    if (id.empty()) {
        value = params;
        is_find = true;
    } else {
        for (auto &cit: params) {
            auto obj = cit.second;
            if (obj && obj->name == id) {
                value[cit.first] = obj;
                is_find = true;
                break;
            }
        }
    }
    return is_find;
}

bool etc_data::write(const etc_id_t &id, const etc_param_t &value)
{
    update_param_.clear();
    update_param_ = value;

    if (data_cb_ && !update_param_.empty()) {
        data_cb_(update_param_);
    }

    return true;
}

bool etc_data::read(json_value_t &value, const etc_id_t &id)
{
    bool ret = true;
    try {
        ret = pack_data(value, id);
        if (false == ret) {
            LOGP_ERROR("pack buffer to json param error");
        }
    } catch (std::exception &e) {
        LOGP_ERROR("pack buffer to json param exception, %s", e.what());
        ret = false;
    }
    return ret;
}

bool etc_data::write(json_value_t &value, const etc_id_t &id)
{
    int ret = RET_OK;
    try {
        update_param_.clear();
        ret = parse(value)? RET_OK : RET_ETC_ETC_PARSE_SUB_OBJECT_ERROR;
        if (RET_OK == ret) {
            ret = save()? RET_OK : RET_ETC_ETC_PARSE_WRITE_JSON_TO_FILE_ERROR;
        }
    } catch (std::exception &e) {
        LOGP_ERROR("write json param exception, %s", e.what());
        ret = RET_ETC_ETC_PARSE_JSON_EXCEPTION;
    }

    if ((RET_OK == ret) && data_cb_ && !update_param_.empty()) {
        data_cb_(update_param_);
    }

    return ret == RET_OK;
}

etc_param_t etc_data::get_calib_param() const
{
    etc_param_t etc_params;
    auto params = ETC_POOL->get_param();
    for (auto &cit: params) {
        auto obj = cit.second;
        if (obj && obj->major_type == kHwCalib) {
            etc_params[cit.first] = obj;
        }
    }
    return etc_params;
}

etc_value_t etc_data::get_calib_param(const etc_id_t &id) const
{
    auto params = ETC_POOL->get_param();
    for (auto &cit : params) {
        auto obj = cit.second;
        if (obj && obj->name == id) {
            return obj;
        }
    }
    return nullptr;
}

void etc_data::set_calib_param(const etc_param_t &params)
{
    ETC_POOL->set_param(params);
}

void etc_data::set_calib_param(const etc_id_t &id, etc_value_t param)
{
    ETC_POOL->set_param(id, param);
}

bool etc_data::write_calib_param(const etc_id_t &id, etc_value_t param)
{
    bool ret = true;
    try {
        update_param_.clear();
        update_param_[id] = param;
        set_calib_param(id, param);
        ret &= save();
    } catch (std::exception &e) {
        LOG_STREAM_ERROR << "write json param exception, " << e.what();
        ret = false;
    }

    if(ret && data_cb_ && !update_param_.empty()) {
        data_cb_(update_param_);
    }

    return ret;
}

bool etc_data::write_calib_param(const etc_param_t &param)
{
    bool ret = true;
    try {
        update_param_ = param;
        set_calib_param(param);
        ret &= save();
    } catch (std::exception &e) {
        LOG_STREAM_ERROR << "write json param exception, " << e.what();
        ret = false;
    }

    if(ret && data_cb_ && !update_param_.empty()) {
        data_cb_(update_param_);
    }

    return ret;
}

etc_value_t etc_data::find_object(const std::string &id, etc_param_t &param)
{
    auto cit = param.find(id);
    return cit == param.end()? nullptr : cit->second;
}

bool etc_data::reset_object(const std::set<std::string> &ids, etc_param_t &params)
{
    auto param = params.begin();
    for( ; param != params.end(); ++param) {
        auto id = param->first;
        auto obj = param->second;
        if(obj && obj->major_type == kHwCalib) {
            auto cit = std::find(ids.begin(), ids.end(), id);
            if(cit == ids.end()) {
                params.erase(param);
            }
        }
    }
    return true;
}

} // namespace etc
} // namespace comwise
