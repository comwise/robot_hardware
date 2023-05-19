#ifndef __COMWISE_ETC__ETC_OBSERVER__H__
#define __COMWISE_ETC__ETC_OBSERVER__H__

#include <memory>
#include "core/observer.h"
#include "etc_arg.h"

namespace comwise {
namespace etc {

class etc_observer : public core::i_observer
{
    using etc_state_t = core::etc_state_t;
public:
    etc_observer(const std::string &id);
    virtual ~etc_observer();

    //! update argument
    virtual bool update(std::shared_ptr<arg_t> arg = nullptr) override;

    //! set state callback
    virtual bool set_callback(state_callback_t cb) override { state_cb_ = cb; }

protected:
    virtual void notify(const etc_param_t &data);

protected:
    //! state param
    state_callback_t state_cb_{nullptr};

    //! etc param
    etc_param_t etc_map_;
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_OBSERVER__H__
