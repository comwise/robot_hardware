#ifndef __COMWISE_SERIAL__SERIAL_DATA__H__
#define __COMWISE_SERIAL__SERIAL_DATA__H__

#include <cstdint>
#include <memory>
#include <chrono>
#include <vector>
#include <map>
#include "var/var_sensor.h"

namespace comwise {
namespace serial {

using serial_buffer_t = serial_buffer_t;
using serial_param_t = serial_param_t;
using serial_params_t = std::map<int, serial_param_t>;

using serial_data_t = std::vector<uint8_t>;
using serial_vector_t = std::vector<serial_data_t>;

using read_func_t = std::function<bool(serial_data_t &)>;
using write_func_t = std::function<bool(serial_vector_t &)>;

struct serial_client_t {
  std::string id;              // id
  read_func_t read{nullptr};   // 读取数据函数
  write_func_t write{nullptr}; // 写入数据函数
  uint32_t write_interval{10}; // 写入多帧数据每帧间隔时间 ms
  int32_t read_timeout{50};    // 读取数据超时时间 ms
  int32_t repeat{1};           // 循环次数 -1为无限次循环 0不执行 >1执行指定次数
};

} // namespace serial
} // namespace comwise

#endif // __COMWISE_SERIAL__SERIAL_DATA__H__