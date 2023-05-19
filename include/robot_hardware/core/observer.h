#ifndef __COMWISE_CORE__IOBSERVER__H__
#define __COMWISE_CORE__IOBSERVER__H__

#include <atomic>
#include <memory>
#include <functional>
#include "arg.h"

namespace core {

class i_observer
{
public:
    using state_callback_t = std::function<bool(uint32_t, std::shared_ptr<arg_t>)>;

public:
    i_observer(const std::string &id) : etc_id_(id) { }
    virtual ~i_observer() {}

    //! get/set name
    virtual std::string get_id() const { return etc_id_; }
    virtual void set_id(const std::string &id) { etc_id_ = id; }

    //! get/set state
    virtual etc_state_t get_state() const { return etc_state_; }
    virtual void set_state(etc_state_t state) { etc_state_ = state; }

    //! update argument
    virtual bool update(std::shared_ptr<arg_t> arg = nullptr) = 0;

    //! set state callback
    virtual bool set_callback(state_callback_t cb) = 0;

protected:
    std::string etc_id_;
    std::atomic<etc_state_t> etc_state_ {kStateChangedNone};
};

} // namespace core

#endif // __COMWISE_CORE__IOBSERVER__H__
