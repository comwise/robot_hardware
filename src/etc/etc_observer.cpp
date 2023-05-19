#include "etc/etc_observer.h"
#include "log/log.h"

namespace comwise {
namespace etc {


etc_observer::etc_observer(const std::string &id)
    : i_observer(id)
{

}

etc_observer::~etc_observer()
{

}

bool etc_observer::update(std::shared_ptr<arg_t> arg)
{
    if(etc_state_ <= core::kStateChangedNone) {
        return true;
    }

    auto ptr = std::dynamic_pointer_cast<etc_arg_t>(arg);
    if (nullptr == ptr) {
        LOGP_ERROR("etc param is nullptr");
        return false;
    }

    bool ret = false;
    for (auto &item : ptr->param) {
        if (etc_map_.find(item.first) != etc_map_.end()) {
            etc_map_[item.first] = item.second;
            ret = true;
        } else {
            etc_map_[item.first] = item.second;
            ret = true;
        }
    }

    if(ret) {
        etc_state_ = core::kStateChangedNone;
    }
    
    return ret;
}

void etc_observer::notify(const etc_param_t &data)
{
    set_state(core::kStateChangedNode);
    //call state_cb_ to notify data
}

} // namespace etc
} // namespace comwise
