#ifndef __COMWISE_ETC__ETC_ARG__H__
#define __COMWISE_ETC__ETC_ARG__H__

#include "core/arg.h"

namespace comwise {
namespace etc {

using arg_t = core::arg_t;
using arg_ptr_t = core::arg_ptr_t;

using etc_id_t = std::string;
using etc_value_t = std::shared_ptr<comwise::etc_obj_t>;
using etc_param_t = std::map<etc_id_t, etc_value_t>;

class etc_arg_t : public arg_t
{
public:
    etc_param_t param;

public:
    void set_param(const etc_param_t _param) { param = _param; }
    etc_param_t get_param() const { return param; }
};

} // namespace etc
} // namespace comwise

#endif //__COMWISE_ETC__ETC_ARG__H__
