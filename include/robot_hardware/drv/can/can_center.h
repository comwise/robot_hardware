#ifndef __COMWISE_CAN__CAN_CENTER__H__
#define __COMWISE_CAN__CAN_CENTER__H__

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <map>
#include <functional>
#include "common/singleton.h"
#include "common/thread_pool.h"
#include "common/queue/concurrentqueue.h"
#include "can_base.h"

namespace comwise {
namespace can {

class msg_publisher
{
public:
  using msg_cb_t = std::function<bool(uint32_t /*channel*/, 
      uint32_t /*frame_id*/, const uint8_t * /*data*/, uint32_t /*size*/)>;

public:
  explicit msg_publisher(uint32_t channel, uint32_t id, msg_cb_t writer)
    : channel_(channel), id_(id), write_handler_(writer) { }

  bool publish(const uint8_t *data, uint32_t size) {
    return write_handler_(channel_, id_, data, size);
  }

private:
  uint32_t channel_{0};
  uint32_t id_{0};
  msg_cb_t write_handler_{nullptr};
};

class can_center : public common::singleton<can_center>
{
  using can_queue_t = moodycamel::ConcurrentQueue<can_frame_t>;
  using can_callback_t = std::function<void(const can_frame_t)>;
  using can_map_t = std::map<uint32_t, can_callback_t>;

public:
  enum class center_state_t { kUnknown, kReady, kActive, kStopped };

public:
  can_center();
  ~can_center();

  //!> init/deinit can param
  bool init(const can_param_t &can_param);
  bool deinit();

  //!> start/stop can
  bool start();
  bool stop();
  bool restart();

  //!> sub/pub
  void subscribe(uint32_t channel, uint32_t frame_id, const can_callback_t &data_callback);
  std::shared_ptr<msg_publisher> advertise(uint32_t channel, uint32_t frame_id);

  //!> set and get state
  void set_state(center_state_t state);
  center_state_t get_state();

  //!> check timeout
  bool check_timeout();

private:
  //!> read/write can0/1 buffer data
  bool write_can(uint32_t channel, can_queue_t &data);

  //!> write data to can
  bool writer(uint32_t channel, uint32_t frame_id,
              const uint8_t *data, uint32_t size);
  bool write(uint32_t channel, const std::vector<can_frame_t> &data);

  //!> can frame data callback handler
  void msg_handler(uint32_t channel, const std::vector<can_frame_t> &data);

private:
  // param
  can_param_t can_param_;

  // state
  center_state_t state_;

  // can object
  std::shared_ptr<can_base> imp_can_ptr_{nullptr};
  bool is_loop_{false};

  // thread pool
  ::common::ThreadPool thread_pool_;

  // channel 0/1 subsrible map
  can_map_t can0_cb_map_;
  can_map_t can1_cb_map_;

  // send buffer
  can_queue_t can0_send_buf_;
  can_queue_t can1_send_buf_;
};

} // namespace can
} // namespace comwise

#define CAN_CENTER comwise::can::can_center::instance()

#endif // __COMWISE_CAN__CAN_CENTER__H__
