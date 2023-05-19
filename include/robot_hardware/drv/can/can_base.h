#ifndef __COMWISE_CAN__CAN_BASE__H__
#define __COMWISE_CAN__CAN_BASE__H__

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "can_data.h"
#include "core/error.h"
#include "core/state.h"

namespace comwise {
namespace can {

class can_base : public core::error_code<int>, public core::state
{
public:
  using callback_t = std::function<
    void(int channel, const std::vector<can_frame_t> &frame)>;

public:
  explicit can_base(const std::string &id) : id_(id) { }
  virtual ~can_base() { }

  virtual int init(const can_param_t &can_param) = 0;
  virtual int deinit() = 0;

  virtual int start() = 0;
  virtual int stop() = 0;
  virtual int restart() = 0;

  virtual int read(uint8_t channel, std::vector<can_frame_t> &frames) = 0;
  virtual int write(uint8_t channel, const std::vector<can_frame_t> &frames) = 0;

  virtual void set_data_callback(const callback_t &cb) { data_cb_ = cb; }
  virtual bool check_timeout() = 0;

 protected:
  std::string id_;
  callback_t data_cb_ {nullptr};
  
};

} // namespace can
} // namespace comwise

#endif  // __COMWISE_CAN__CAN_BASE__H__
