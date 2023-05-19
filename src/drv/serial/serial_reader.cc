#include "serial/serial_reader.h"
#include "log/log.h"
#include "common/count_time.h"
#include "serial/serial_driver.h"
#include "lib/serial/serial.h"

namespace comwise {
namespace serial {

serial_reader::serial_reader(const std::string &id, int type)
    : core::driver_core<serial_param_t>(id, type)
{
}

serial_reader::~serial_reader()
{
  deinit();
}

int serial_reader::init(const serial_param_t &serial_param)
{
  int ret = -1;
  try {
    param_ = serial_param;
    set_frequency(serial_param.frequency);
    ret = ring_buffer::init(serial_param.buffer);
    serial_ = std::make_shared<serial::serial_driver>(id_);
    if (serial_) {
      ret = serial_->init(param_);
    }
  } catch (std::exception &e) {
    LOGP_ERROR("serial(%s) init excepiton: %s", id_.c_str(), e.what());
    ret = -10;
    //list_port();
  }
  set_state(kStateReady);
  LOGP_INFO("serial(%s) init return %d", id_.c_str(), ret);
  return ret;
}

int serial_reader::deinit()
{
  return stop();
}

void serial_reader::list_port()
{
  std::vector<::serial::PortInfo> devices_found = ::serial::list_ports();
  std::vector<::serial::PortInfo>::const_iterator iter = devices_found.begin();
  for (; iter != devices_found.end(); iter++) {
    ::serial::PortInfo device = *iter;
    printf("(%s, %s, %s)\n", device.port.c_str(),
           device.description.c_str(), device.hardware_id.c_str());
    std::string port = device.port;
    if (!access(port.c_str(), X_OK)) {
      LOGP_INFO("find serial port");
      break;
    }
  }
}

int serial_reader::start()
{
  int ret = 0;
  if (serial_) {
    ret = serial_->start();
  }

  ring_buffer::start();
  set_state(kStateRunning);
  return ret;
}

int serial_reader::stop()
{
  int ret = 0;
  ret = ring_buffer::stop();
  if (serial_) {
    ret = serial_->stop();
  }
  set_state(kStateExit);
  return ret;
}

int serial_reader::read(uint8_t *data, uint32_t size)
{
  return serial_? serial_->read(data, size) : -1;
}

int serial_reader::parse(uint8_t *data, uint32_t size)
{
#ifdef DEBUG_FRAME
  for (uint32_t i = 0; i < size; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
#endif
  return 0;
}

} // namespace serial
} // namespace comwise
