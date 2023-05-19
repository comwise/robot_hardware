#include "etc/etc_template.h"
#include <json/json.h>
#include "log/log.h"
#include "etc/etc_default.h"
#include "etc/etc_cfg.h"
#include "etc/etc_pool.h"

namespace comwise {
namespace etc {

etc_template::etc_template(const std::string &file)
    : etc_parse(file)
{

}

etc_template::~etc_template()
{

}

bool etc_template::parse(const json_value_t &root)
{
    bool ret = true;
    template_ = std::make_shared<json_value_t>();
    Json::Value::Members mem = root.getMemberNames();
    for (auto &iter : mem) {
        std::shared_ptr<template_obj_t> template_obj;
        std::string module = iter;
        for(auto &val : root[module]) {
            std::string type;
            template_obj_t obj;
            obj.major_type = etc_default::get_drv_type(module);
            if (val.isMember("id")) {
                obj.id = val["id"].asString();
            }
            if (val.isMember("type")) {
                type = val["type"].asString();
                if (type.empty()) {
                    type = module;
                }
                obj.type = type;
            }
            if (val.isMember("adapter")) {
                obj.adapter = val["adapter"].asString();
            }
            if (val.isMember("node")) {
                obj.node = val["node"].asString() == "ros";
            }
            if (val.isMember("url")) {
                obj.url = val["url"].asString();
            }
            if (val.isMember("env")) {
                for (auto &item : val["env"]) {
                    obj.env.push_back(item.asString());
                }
            }
            if (val.isMember("depend")) {
                for (auto &item : val["depend"]) {
                    obj.depend.push_back(item.asString());
                }
            }
            if (val.isMember("args")) {
                for (auto &item : val["args"]) {
                    obj.args.push_back(item.asString());
                }
            }
            if (val.isMember("default")) {
                json_value_t def_obj = val["default"];
                def_obj["node"] = obj.node;
                etc_value_t def_val = nullptr;
                json_value_t new_json;
                new_json[module].append(def_obj);
                (*template_)[module].append(def_obj);
                Json::StreamWriterBuilder builder;
                builder["commentStyle"] = "None";
                builder["indentation"] = " ";
                obj.config = Json::writeString(builder, new_json);
                bool ret = parse_value(def_obj, module, type, def_val);
                obj.def = def_val;
            }

            template_obj = std::make_shared<template_obj_t>(obj);
            type_index_[module][type] = template_obj;
            id_index_[obj.id] = template_obj;
            if (ETC_POOL) {
                ETC_POOL->set_template(module, type, template_obj);
            }
        }
    }
    return ret;
}

bool etc_template::read(const std::string &id, std::string &value)
{
    bool is_find = false;
    auto cit = id_index_.find(id);
    if (cit != id_index_.end()) {
        auto obj = std::dynamic_pointer_cast<template_obj_t>(id_index_[id]); 
        if (obj) {
            value = obj->config;
            is_find = true;
        }
    } else if (id.empty()) {
        Json::StreamWriterBuilder builder;
        value = Json::writeString(builder, *template_);
        is_find = true;
    } else {
        is_find = false;
    }
    return is_find;
}

bool etc_template::read(const std::string &major, const std::string &minor, std::string &value)
{
    bool is_find = false;
    if (major.empty()) {
        Json::StreamWriterBuilder builder;
        value = Json::writeString(builder, *template_);
        return true;
    }

    auto cit = type_index_.find(major);
    if (cit != type_index_.end()) {
        etc_value_t find_obj = nullptr;
        auto &minor_map = cit->second;
        if (minor.empty()) {
            if(!minor_map.empty()) {
                find_obj = minor_map.begin()->second;
            }
        } else {
            if (minor_map.find(minor) != minor_map.end()) {
                find_obj = minor_map[minor];
            }
        }
        auto obj = std::dynamic_pointer_cast<template_obj_t>(find_obj);
        if (obj) {
            value = obj->config;
            is_find = true;
        }
    }

    return is_find;
}

bool etc_template::read(const etc_id_t &id, etc_param_t &value)
{
    bool is_find = false;
    auto cit = id_index_.find(id);
    if (cit != id_index_.end()) {
        value[id] = cit->second;
        is_find = true;
    } else {
        value = id_index_;
    }
    return is_find;
}

bool etc_template::read(const std::string &major, const std::string &minor, etc_param_t &value)
{
    bool is_find = false;
    if (major.empty()) {
        value = id_index_;
        return true;
    }

    auto cit = type_index_.find(major);
    if (cit != type_index_.end()) {
        auto &val = type_index_[major];
        if (minor.empty()) {
            if (!val.empty()) {
                const auto &val_find = val.begin();
                value[val_find->first] = val_find->second;
                is_find = true;
            }
        } else if (val.find(minor) != val.end()) {
            value[minor] = val[minor];
            is_find = true;
        }
    }
    return is_find;
}

} // namespace etc
} // namespace comwise
