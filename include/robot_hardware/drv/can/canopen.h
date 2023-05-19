#ifndef __COMWISE_CAN__CANOPEN_NMT__H__
#define __COMWISE_CAN__CANOPEN_NMT__H__

#include <memory>
#include <string>
#include "can/can_center.h"

namespace comwise {
namespace can {

class canopen
{
public:
  canopen(uint32_t channel, uint32_t node_id);
  ~canopen();

  void start();
  void stop();
  void preoperation();
  void reset_node();
  void reset_communication();

private:
  void init(uint32_t channel, uint32_t node_id);
  void deinit();

  void command(unsigned char cmd);

private:
  std::shared_ptr<msg_publisher> nmt_pub_{nullptr};
  uint32_t channel_{0};
  uint32_t node_id_{0};
};

} // namespace can
} // namespace comwise

#endif  // __COMWISE_CAN__CANOPEN_NMT__H__
