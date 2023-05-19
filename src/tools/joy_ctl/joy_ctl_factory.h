#ifndef __COMWISE_JOY__JOY_CTL_FACTORY__H__
#define __COMWISE_JOY__JOY_CTL_FACTORY__H__

#include "common/singleton.h"
#include "joy_ctl_diff.h"
#include "joy_ctl_forklift.h"

namespace comwise {
namespace joy {

class joy_ctl_factory final : public common::singleton<joy_ctl_factory>
{
public:
  std::shared_ptr<joy_ctl_base> create(const std::string &id) {
    if (id == "joy_ctl_diff") {
      return std::make_shared<joy_ctl_diff>("diff");
    } else if (id == "joy_ctl_forklift") {
      return std::make_shared<joy_ctl_forklift>("forklift");
    } else {
      return nullptr;
    }
  }
};

} // namespace joy
} // namespace comwise

#define JOY_FACTORY comwise::joy::joy_ctl_factory::instance()

#endif //__COMWISE_JOY__JOY_CTL_FACTORY__H__
