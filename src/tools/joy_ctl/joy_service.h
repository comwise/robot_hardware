#ifndef __COMWISE_JOY__JOY_SERVICE__H__
#define __COMWISE_JOY__JOY_SERVICE__H__

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <map>
#include "joy_data.h"

namespace ros {
  class NodeHandle;
  class Publisher;
  class Subscriber;
  class ServiceClient;
  class ServiceServer;
}

namespace comwise {
namespace joy {

class joy_ctl_base;
class joy_service
{
  using joy_ptr_t = std::shared_ptr<joy_ctl_base>;
  using joy_app_t = std::map<std::string, joy_ptr_t>;
  using joy_cfg_t = std::map<std::string, bool>;
public:
  explicit joy_service(const std::string &file ="");
  virtual ~joy_service();

  //!> init/deinit
  bool init(const std::string &file = "");
  bool deinit();

  //!> start/stop
  bool start();
  bool stop();

  //!> chassis move control
  void set_speed(const vel_cmd_t & vel);
  bool get_speed(vel_list_t &vel);
  void set_brake();

public:
  void joy_handler(const joy_data &joy);

private:
  std::string cfg_file_;
  joy_cfg_t joy_cfgs_;
  joy_app_t joy_apps_;

  //!> is inited/deinit
  bool is_init_ {false};
  bool is_deinit_ {false};

  //!> joy
  std::mutex joy_mtx_;
  std::shared_ptr<ros::Subscriber> joy_sub_ {nullptr};
  std::shared_ptr<ros::Publisher> joy_pub_ {nullptr};

  //!> chassis
  vel_list_t move_vel_;
  std::mutex move_vel_mtx_;
  std::shared_ptr<ros::Publisher> move_cmd_pub_ {nullptr};
  std::shared_ptr<ros::Subscriber> move_feedback_sub_ {nullptr};
  std::shared_ptr<ros::Publisher> move_brake_pub_ {nullptr};
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_SERVICE__H__
