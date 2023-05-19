#ifndef __COMWISE_CORE__STATE__H__
#define __COMWISE_CORE__STATE__H__

#include <cstdint>
#include <atomic>

namespace core {

class state {
public:
    enum state_t {
        kStateIdle = 0,
        kStateReady,
        kStateRunning,
        kStatePause,
        kStateExit,
    };
public:
    virtual ~state() { }

    virtual void set_state(const state_t &_state) { state_ = _state; }
    virtual int get_state() const { return state_; }

private:
    state_t state_{kStateIdle};
};

} // namespace core

#endif // __COMWISE_CORE__STATE__H__
