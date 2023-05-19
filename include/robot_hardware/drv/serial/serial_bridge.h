#ifndef __COMWISE_SERIAL__SERIAL_BRIDGE__H__
#define __COMWISE_SERIAL__SERIAL_BRIDGE__H__

#include <memory>
#include <string>
#include <map>
#include <mutex>
#include "common/singleton.h"
#include "serial/serial_data.h"
#include "core/driver.h"

namespace std {
  class thread;
}

namespace common {
class ThreadPool;
}

namespace comwise {
namespace serial {

using serial_dev_t = std::string;
using client_id_t = std::string;

class serial_driver;

/* @brief  : manage all serial devices on one serial
 * @details: serial bridge -> multi serial devices
 * @author : lichanglin
 */
class serial_bridge : public core::driver_core<serial_param_t>
{
  using thread_pool_t = common::ThreadPool;
  using client_map_t = std::map<client_id_t, serial_client_t>;

public:
  explicit serial_bridge(const std::string &id, int type = 0);
  virtual ~serial_bridge();

  virtual int init(const serial_param_t &param) override;
  virtual int deinit() override;

  virtual int start() override;
  virtual int stop() override;

  void reg(const serial_client_t &client);
  void unreg(const client_id_t &id);

private:
  void handler();

  int read(serial_data_t &data, int timeout);

private:
  serial_param_t param_;
  std::shared_ptr<serial_driver> serial_{nullptr};

  std::mutex mutex_;
  std::shared_ptr<thread_pool_t> thr_pool_{nullptr};
  std::shared_ptr<std::thread> thr_{nullptr};
  bool is_loop_{false};
  
  client_map_t clients_;
};

/* @brief  : serial center for manage all serial
 * @details: id -> serial bridge -> multi serial devices
 * @author : lichanglin
 */
class serial_center : public common::singleton<serial_center>
{
  using serial_bridge_ptr = std::shared_ptr<serial_bridge>;
  using serial_map_t = std::map<std::string, serial_bridge_ptr>;

public:
  serial_center();
  ~serial_center();

  //@brief init/deinit serial by serial param
  bool init(const serial_param_t &param);
  bool deinit(const serial_dev_t &dev = "");

  //@brief start/stop serial, you can start/stop all serial when the serial device is empty("")
  bool start(const serial_dev_t &dev);
  bool stop(const serial_dev_t &dev = "");

  //@brief register and unregister one client to one serial for multi devices share one serial
  void reg(const serial_dev_t &dev, const serial_client_t &client);
  void unreg(const serial_dev_t &dev, const client_id_t &id = "");

private:
  serial_map_t serials_;
};

}  // namespace serial
}  // namespace comwise

#define SERIAL_CENTER comwise::serial::serial_center().instance()

#endif  // __COMWISE_SERIAL__SERIAL_BRIDGE__H__
