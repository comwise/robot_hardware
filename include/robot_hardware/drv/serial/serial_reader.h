#ifndef __COMWISE_SERIAL__SERIAL_READER__H__
#define __COMWISE_SERIAL__SERIAL_READER__H__

#include "serial/serial_data.h"
#include "core/driver.h"
#include "core/buffer.h"

namespace comwise {
namespace serial {

class serial_driver;
class serial_reader : public ::core::driver_core<serial_param_t>, public ::core::ring_buffer
{
public:
  using serial_cb_t = std::function<void(serial_data_t&)>;

public:
  serial_reader(const std::string &id, int type = 0);
  ~serial_reader();

  virtual int init(const serial_param_t &param) override;
  virtual int deinit() override;

  virtual int start() override;
  virtual int stop() override;

protected:
  virtual int read(uint8_t *data, uint32_t size) override;
  virtual int parse(uint8_t *data, uint32_t size) override;

  void list_port();

protected:
  serial_param_t param_;
  serial_cb_t cb_{nullptr};

  std::shared_ptr<serial::serial_driver> serial_{nullptr};
};

} // namespace serial
} // namespace comwise

#endif // __COMWISE_SERIAL__SERIAL_READER__H__