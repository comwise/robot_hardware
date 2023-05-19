#include "etc/etc_subject.h"
#include "log/log.h"

namespace comwise {
namespace etc {

void etc_subject::register_observer(observer_ptr_t object)
{
    std::lock_guard<std::mutex> lck(observer_mtx_);
    if(object) {
        object->set_callback(std::bind(&etc_subject::observer_notify,
            this, std::placeholders::_1, std::placeholders::_2));
        observer_list_.emplace_back(object);
    }
}

void etc_subject::unregister_observer(observer_ptr_t object)
{
    std::lock_guard<std::mutex> lck(observer_mtx_);
    observer_list_t::const_iterator cit = observer_list_.begin();
    for ( ; cit != observer_list_.end(); cit++) {
        if (*cit == object) {
            observer_list_.erase(cit);
        }
    }
}

bool etc_subject::notify_observer(arg_ptr_t arg)
{
    if(etc_state_ <= core::kStateChangedNone) {
        return false;
    }

    bool ret = true;
    std::lock_guard<std::mutex> lck(observer_mtx_);
    for (auto &object : observer_list_) {
        if (object) {
            object->set_state(etc_state_);
            bool sub_ret = object->update(arg);
            if(!sub_ret) {
                LOG_STREAM_ERROR << "update argument error, id = " << object->get_id();
            }
            ret &= sub_ret;
        }
    }

    if(ret) {
        etc_state_ = core::kStateChangedNone;
    } else {
        LOG_STREAM_ERROR << "notify observer argument error";
    }

    return ret;
}

bool etc_subject::observer_notify(uint32_t state, arg_ptr_t object)
{
    if(etc_state_ != core::kStateChangedNone) {
        return false;
    }
    set_state((etc_state_t)state);
    return notify_observer(object);
}

void etc_subject::set_data(uint32_t state, arg_ptr_t arg)
{
    observer_notify(state, arg);
}

} // namespace etc
} // namespace comwise
