#include "serial/serial_driver.h"
#include <thread>
#include "log/log.h"
#include "lib/serial/serial.h"

namespace comwise {
namespace serial {

serial_driver::serial_driver(const std::string &id, int type)
  : core::driver_core<serial_param_t>(id, type)
{
  LOGP_INFO("serial(%s) construct ok", get_id().c_str());
}

serial_driver::~serial_driver()
{
  deinit();
  LOGP_INFO("serial(%s) destruct ok", get_id().c_str());
}

int serial_driver::init(const serial_param_t &param)
{
  param_ = param;
  serial_ = std::make_shared<::serial::Serial>();
  if (nullptr == serial_) {
    LOGP_ERROR("serial(%d) create object error", get_id().c_str());
    return -1;
  }

  try {
    serial_->setPort(param_.device);
    serial_->setBaudrate(param_.baudrate);
    serial_->setBytesize(static_cast<::serial::bytesize_t>(param_.byte_size));
    serial_->setParity(static_cast<::serial::parity_t>(param_.parity));
    serial_->setFlowcontrol(static_cast<::serial::flowcontrol_t>(param_.flow_control));
    serial_->setStopbits(static_cast<::serial::stopbits_t>(param_.stop_bits));

    // http://docs.ros.org/en/kinetic/api/serial/html/structserial_1_1Timeout.html
    // serial::Timeout to = serial::Timeout::simpleTimeout(10000);
    ::serial::Timeout time = ::serial::Timeout::simpleTimeout(param_.timeout);
    serial_->setTimeout(time);
  } catch (::serial::IOException &e) {
    LOGP_ERROR("serial(%s:%s) init exception: %s",
                get_id().c_str(), param_.device.c_str(), e.what());
    return -2;
  }

  LOGP_INFO("serial(%s:%s) init ok", get_id().c_str(), param_.device.c_str());
  set_state(kStateReady);
  return 0;
}

int serial_driver::deinit()
{
  return stop();
}

int serial_driver::start()
{
  int ret = 0;
  if (nullptr == serial_) {
    LOGP_ERROR("serial(%d) object is nullptr", get_id().c_str());
    return -1;
  }
  try {
    serial_->open();
  } catch (::serial::IOException &e) {
    LOGP_ERROR("serial(%s:%s) open exception: %s",
                get_id().c_str(), param_.device.c_str(), e.what());
    ret = -2;
  }
  if (!serial_->isOpen()) {
    LOGP_INFO("serial(%s:%s) isn't open", get_id().c_str(), param_.device.c_str());
    ret = -3;
  }

  if (param_.is_async && !thr_) {
    thr_ = std::make_shared<std::thread>(
      std::bind(&serial_driver::handler, this));
  }
  if (0 == ret) {
    set_state(kStateRunning);
  }
  return ret;
}

int serial_driver::stop()
{
  is_loop_ = false;
  if (thr_ && thr_->joinable()) {
    thr_->join();
  }
  thr_ = nullptr;

  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
  serial_ = nullptr;

  set_state(kStateExit);

  return 0;
}

void serial_driver::set_callback(const serial_cb_t &cb)
{
  cb_ = cb;
}

bool serial_driver::is_ready()
{
  bool ret = true;
  using namespace std::chrono;
  auto now = steady_clock::now();
  if (duration_cast<std::chrono::milliseconds>(now - time_).count() > param_.timeout) {
    LOGP_WARN("serial(%s) disconnected %dms!!!", get_id().c_str(), param_.timeout);
    ret = false;
  }
  return ret;
}

int serial_driver::read(uint8_t *buffer, uint32_t size)
{
  int ret = -1;
  if (serial_ && serial_->available()) {
    ret = serial_->read(buffer, size);
  }
  return ret;
}

int serial_driver::read(std::string &data)
{
  int ret = -1;
  if (serial_ && serial_->available()) {
    data = serial_->read(serial_->available());
    ret = data.size();
  }
  return ret;
}

int serial_driver::read(std::vector<uint8_t> &data)
{
  int ret = -1;
  if (serial_ && serial_->available()) {
    ret = serial_->read(data, serial_->available());
  }
  return ret;
}

int serial_driver::read_line(std::string &data)
{
  return serial_? serial_->readline(data) : -1;
}

int serial_driver::write(const uint8_t *data, uint32_t size)
{
  return serial_? serial_->write(data, size) : -1;
}

int serial_driver::write(const std::vector<uint8_t> &data)
{
  return serial_? serial_->write(data) : -1;
}

int serial_driver::write(const std::string &data)
{
  return serial_? serial_->write(data.c_str()) : -1;
}

void serial_driver::handler()
{
  while (is_loop_) {
    if (serial_->available()) {
      time_ = std::chrono::steady_clock::now();
      std::string raw_data = serial_->read(serial_->available());
      LOGP_DEBUG("serial(%s:%s) read: %s",
                get_id().c_str(), param_.device.c_str(), raw_data.c_str());
      serial_data_t data;
      data.assign(raw_data.begin(), raw_data.end());
      cb_(data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000/param_.frequency));
  }
}

}  // namespace serial
}  // namespace comwise
