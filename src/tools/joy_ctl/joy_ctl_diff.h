#ifndef __COMWISE_JOY__JOY_CTL_DIFF__H__
#define __COMWISE_JOY__JOY_CTL_DIFF__H__

#include "joy_ctl_base.h"

namespace comwise {
namespace joy {

class joy_ctl_diff : public joy_ctl_base
{
public:
  joy_ctl_diff(const std::string &name);
  ~joy_ctl_diff();

  virtual void set_joy_data(const joy_data &joy) override;

private:
  bool is_send_{false};
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_CTL_DIFF__H__
