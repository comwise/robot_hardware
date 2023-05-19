#ifndef __COMWISE_CORE__IARG__H__
#define __COMWISE_CORE__IARG__H__

#include "var/var.h"

namespace core {

using arg_t = comwise::var_obj_t;
using arg_ptr_t = std::shared_ptr<arg_t>;

enum etc_state_t {
    kStateChangedNone,
    kStateChangedDynamic,
    kStateChangedNode,
    kStateChangedAll,
};

} // namespace core

#endif //__COMWISE_CORE__IARG__H__
