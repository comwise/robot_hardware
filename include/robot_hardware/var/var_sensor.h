#ifndef __COMWISE__VAR_SENSOR__H__
#define __COMWISE__VAR_SENSOR__H__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace comwise {

struct buffer_param_t
{
  uint16_t buffer_size {25};
  uint16_t frame_size {11};
  uint16_t read_size {15};
  std::vector<uint8_t> header;
  std::vector<uint8_t> tail;
};

using serial_buffer_t = buffer_param_t;

struct serial_param_t
{
  std::string device;       // serial port
  uint32_t baudrate{9600};  // baudrate
  uint32_t byte_size{8};    // byte size
  uint8_t  parity{0};       // parity
  uint8_t  stop_bits{1};    // stop_bits
  uint8_t  flow_control{0}; // flow_control
  uint32_t timeout{200};    // timeout(ms)

  uint32_t frequency{100};   // frequency
  serial_buffer_t buffer;   // buffer
  bool is_async{false};     // async

  uint32_t directon{1};     // direction, just for imu
};

struct can_param_t
{
  uint32_t node_id{0};      // node id
  uint32_t channel{0};      // can channel(can_channel_t): 0-can0 1-can1
  int32_t mode{0};          // mode
  uint32_t position{0};     // position
};

class sensor_param_t : public serial_param_t, public can_param_t
{
  
};

} // namespace comwise

#endif // __COMWISE__VAR_SENSOR__H__
