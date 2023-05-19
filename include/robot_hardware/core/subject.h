#ifndef __COMWISE_CORE__ISUBJECT__H__
#define __COMWISE_CORE__ISUBJECT__H__

#include <memory>
#include <vector>
#include "observer.h"

namespace core {

class i_subject
{
public:
    using observer_ptr_t = std::shared_ptr<i_observer>;
    using arg_ptr_t = core::arg_ptr_t;

public:
    virtual void register_observer(observer_ptr_t obj) = 0;
    virtual void unregister_observer(observer_ptr_t obj) = 0;
    virtual bool notify_observer(arg_ptr_t arg = nullptr) = 0;
};

} // namespace core

#endif // __COMWISE_CORE__ISUBJECT__H__
