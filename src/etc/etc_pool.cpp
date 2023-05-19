#include "etc/etc_pool.h"
#include "log/log.h"

namespace comwise {
namespace etc {

etc_pool::etc_pool()
{
}

etc_pool::~etc_pool()
{

}

etc_param_t& etc_pool::get_param()
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    return etc_param_;
}

etc_param_t etc_pool::get_param() const
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    return etc_param_;
}

etc_value_t etc_pool::get_param(const etc_id_t &id) const
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    auto cit = etc_param_.find(id);
    if (!id.empty() && cit != etc_param_.end()) {
        return cit->second;
    } else {
        return nullptr;
    }
}

void etc_pool::set_param(const etc_param_t &param)
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    for (auto &cit : param) {
        auto find_cit = etc_param_.find(cit.first);
        if (find_cit != etc_param_.end()) {
            if (cit.second->update == kUpdateDelete) {
                etc_param_.erase(find_cit);
            } else {
                etc_param_[cit.first] = cit.second;
            }
        } else {
            if (cit.second->update != kUpdateDelete) {
                etc_param_[cit.first] = cit.second;
            }
        }
    }
}

void etc_pool::set_param(const etc_id_t &id, etc_value_t param)
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    etc_param_[id] = param;
}

void etc_pool::earse_param(const etc_id_t &id)
{
    std::lock_guard<std::mutex> lck(etc_mtx_);
    auto cit = etc_param_.find(id);
    if (!id.empty() && cit != etc_param_.end()) {
        etc_param_.erase(cit);
    }
}

etc_pool::pool_value_t etc_pool::get_value(pool_key_t key) const
{
    std::lock_guard<std::mutex> lck(topics_mtx_);
    auto cit = topics_.find(key);
    return (cit != topics_.end()) ? cit->second : "";
}
void etc_pool::set_value(pool_key_t key, const pool_value_t &value)
{
    std::lock_guard<std::mutex> lck(topics_mtx_);
    topics_[key] = value;
}

void etc_pool::set_value(const std::map<pool_key_t, pool_value_t> &value)
{
    std::lock_guard<std::mutex> lck(topics_mtx_);
    for (auto &cit : value) {
        topics_[cit.first] = cit.second;
    }
}

void etc_pool::set_template(const std::string &major,
        const std::string &minor, const template_value_t &value)
{
    std::lock_guard<std::mutex> lck(template_mtx_);
    template_param_[major][minor] = value;
}

bool etc_pool::get_template(const std::string &major,
    const std::string &minor, template_value_t &value)
{
    bool ret = false;
    std::string type = minor;
    if (major.empty()) {
        return false;
    }
    if (minor.empty()) {
        type = major;
    }

    if (template_param_.find(major) != template_param_.end()) {
        if (template_param_[major].find(type) != template_param_[major].end()) {
            value = template_param_[major][type];
            ret = true;
        }
    } 

    return ret;
}

etc_value_t etc_pool::get_default(const std::string &major, const std::string &minor)
{
    etc_value_t val = nullptr;
    if (major.empty()) {
        return val;
    }

    if (template_param_.find(major) == template_param_.end()) {
        return val;
    }

    std::string type = minor;
    if (type.empty()) {
        type = major;
    }
    if (template_param_[major].find(type) != template_param_[major].end()) {
        val = template_param_[major][type]->def;
    }
    return val;
}

etc_param_t etc_pool::get_default(const std::string &major)
{
    etc_param_t val;
    auto cit = template_param_.find(major);
    if (!major.empty()) {
        if(cit != template_param_.end() && !cit->second.empty()) {
            val[major] = cit->second.begin()->second->def;
        }
    } else {
        for(auto &item : template_param_) {
            if(!item.second.empty()) {
                val[major] = cit->second.begin()->second->def;
            }
        }
    }
    return val;
}

} // namespace etc
} // namespace comwise
