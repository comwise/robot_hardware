#ifndef __COMWISE_SERIAL__SERIAL_DRIVER__H__
#define __COMWISE_SERIAL__SERIAL_DRIVER__H__

#include <cstdint>
#include <memory>
#include <chrono>
#include "core/driver.h"
#include "serial_data.h"

namespace std {
  class thread;
}

namespace serial {
  class Serial;
}
namespace comwise {
namespace serial {

class serial_driver : public core::driver_core<serial_param_t>
{
public:
  using serial_t = ::serial::Serial;
  using serial_ptr_t = std::shared_ptr<serial_t>;
  using serial_cb_t = std::function<void(serial_data_t&)>;

public:
  serial_driver(const std::string &name, int type = 0);
  virtual ~serial_driver();

  virtual int init(const serial_param_t &param) override;
  virtual int deinit() override;

  virtual int start() override;
  virtual int stop() override;

  virtual int read(uint8_t *buffer, uint32_t size);
  virtual int read(std::string &data);
  virtual int read(std::vector<uint8_t> &data);
  virtual int read_line(std::string &data);
  virtual int write(const uint8_t *data, uint32_t size);
  virtual int write(const std::vector<uint8_t> &data);
  virtual int write(const std::string &data);

  virtual bool is_ready();

  virtual void set_callback(const serial_cb_t &cb);

private:
  void handler();

private:
  serial_param_t param_;
  serial_ptr_t serial_{nullptr};
  serial_cb_t cb_{nullptr};

  std::shared_ptr<std::thread> thr_{nullptr};
  bool is_loop_{false};

  std::chrono::steady_clock::time_point time_;
};

} // namespace serial
} // namespace comwise

#endif // __COMWISE_SERIAL__SERIAL_DRIVER__H__