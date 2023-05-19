#ifndef __COMWISE_ETC__ETC_SUBJECT__H__
#define __COMWISE_ETC__ETC_SUBJECT__H__

#include <atomic>
#include <vector>
#include <mutex>
#include "core/subject.h"

namespace comwise {
namespace etc {

class etc_subject : public core::i_subject
{
    using etc_state_t = core::etc_state_t;
public:
    using observer_list_t = std::vector<observer_ptr_t>;

public:
    virtual void register_observer(observer_ptr_t object) override;
    virtual void unregister_observer(observer_ptr_t object) override;

    //! notify observer
    virtual bool notify_observer(arg_ptr_t argument = nullptr) override;

    //! set notify data
    void set_data(uint32_t state, arg_ptr_t arg);

protected:
    virtual void set_state(etc_state_t state) { etc_state_ = state; }
    virtual etc_state_t get_state() const { return etc_state_; }

private:
    //! observer notify data
    bool observer_notify(uint32_t state, arg_ptr_t data);

protected:
    //! all observer list
    observer_list_t observer_list_;
    std::mutex observer_mtx_;

    //! etc state
    std::atomic<etc_state_t> etc_state_{core::kStateChangedNone};
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_SUBJECT__H__
