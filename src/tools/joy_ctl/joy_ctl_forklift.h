#ifndef __COMWISE_JOY__JOY_CTL_FORKLIFT__H__
#define __COMWISE_JOY__JOY_CTL_FORKLIFT__H__

#include "joy_ctl_base.h"

namespace comwise {
namespace joy {

class joy_ctl_forklift : public joy_ctl_base
{
public:
  joy_ctl_forklift(const std::string &name);
  ~joy_ctl_forklift();

  virtual void set_joy_data(const joy_data &joy) override;

private:
  bool is_send_{false};
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_CTL_FORKLIFT__H__
