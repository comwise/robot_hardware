#include "serial/serial_bridge.h"
#include <chrono>
#include <functional>
#include <future>
#include "log/log.h"
#include "common/count_time.h"
#include "common/thread_pool.h"
#include "serial/serial_driver.h"

namespace comwise {
namespace serial {

#define PRINT_DATA(flag, data) \
  std::cout << flag << ":"; \
  for (auto &item : data) { \
    std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (int)item;\
  } \
  std::cout << std::endl;

serial_bridge::serial_bridge(const std::string &id, int type)
  : core::driver_core<serial_param_t>(id, type)
  , thr_pool_(nullptr)
  , thr_(nullptr)
{

}

serial_bridge::~serial_bridge()
{
  deinit();
}

int serial_bridge::init(const serial_param_t &param)
{
  int ret = 0;
  try {
    thr_pool_ = std::make_shared<thread_pool_t>(
      std::thread::hardware_concurrency());
    param_ = param;
    serial_ = std::make_shared<serial_driver>(get_id());
    ret = serial_->init(param);
  } catch (std::exception &e) {
    LOGP_ERROR("init serial(%s) exception: %s",
      param_.device.c_str(), e.what());
    ret = -10;
  }
  if (ret == 0) {
    set_state(kStateReady);
  }
  return ret;
}

int serial_bridge::deinit()
{
  return stop();
}

int serial_bridge::start()
{
  is_loop_ = true;
  thr_ = std::make_shared<std::thread>([&]() {
    common::count_time ct;
    while (is_loop_) {
      ct.end();
      int sleep_time = 1000/param_.frequency - ct.delta();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sleep_time > 0 ? sleep_time : 0));
      ct.begin();
      switch (get_state())
      {
        case kStateIdle:
          init(param_);
          continue;
        case kStateReady:
        {
          if (serial_ && 0 == serial_->start()) {
            set_state(kStateRunning);
          }
          continue;
        }
        case kStateRunning:
          break;
        default:
          continue;
      }
      handler();
    }
  });
  
  return 0;
}

int serial_bridge::stop()
{
  int ret = 0;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    clients_.clear();
  }
  is_loop_ = false;
  if (thr_ && thr_->joinable()) {
    thr_->join();
  }
  thr_ = nullptr;
  if (serial_) {
    ret = serial_->stop();
  }
  set_state(kStateExit);
  return ret;
}

void serial_bridge::reg(const serial_client_t &client)
{
  std::unique_lock<std::mutex> lock(mutex_);
  clients_[client.id] = client;
}

void serial_bridge::unreg(const client_id_t &id)
{
  std::unique_lock<std::mutex> lock(mutex_);
  auto cit = clients_.find(id);
  if (cit != clients_.end()) {
    clients_.erase(cit);
  }
}

void serial_bridge::handler()
{
  std::map<std::string, serial_client_t> clients;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    clients = clients_;
  }
  for (auto &client : clients) {
    if (client.second.repeat == 0) {
      continue;
    }
    // 1. get write data
    auto &w_cb = client.second.write;
    std::future<int> f;
    if (nullptr != w_cb) {
      serial_vector_t write_data;
      if (!w_cb(write_data)) {
        continue;
      }
      for (auto &data : write_data) {
        if(!data.empty()) {
          //PRINT_DATA("W", data);
          if (serial_->write(data) <= 0) {
            break;
          }
          std::this_thread::sleep_for(
            std::chrono::milliseconds(client.second.write_interval));
        } 
      }
    }

    // 2 get read data
    auto &r_cb = client.second.read;
    if (r_cb) {
      serial_data_t data;
#ifdef THREAD_POOL
      if (read(data, client.second.read_timeout) > 0 && thr_pool_ && !data.empty()) {
        thr_pool_->enqueue([r_cb, data]() {
          r_cb(data);
        });
      }
#else
      if (serial_) {
        int size = serial_->read(data);
        if (size > 0 && !data.empty()) {
          //PRINT_DATA("R", data);
          r_cb(data);
        }
      }
#endif
    }

    // 3 repeat time
    if (client.second.repeat > 0) {
      if (--client.second.repeat == 0) {
        // remove client
        unreg(client.first);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int serial_bridge::read(serial_data_t &data, int timeout)
{
  int ret = -1;
  data.resize(0);
  std::future<int> f = std::async(std::launch::async, [&]() {
    return serial_? serial_->read(data) : -1;
  });
  switch (f.wait_for(std::chrono::milliseconds(timeout)))
  {
  case std::future_status::ready:
    ret = f.get();
    break;
  case std::future_status::timeout:
    ret = -2;
    break;
  default:
    ret = -3;
    break;
  }
  return ret;
}

serial_center::serial_center()
{

}

serial_center::~serial_center()
{
  deinit("");
}

bool serial_center::init(const serial_param_t &param)
{
  auto &id = param.device; 
  auto cit = serials_.find(id);
  if (cit == serials_.end()) {
      auto obj = std::make_shared<serial_bridge>(id);
      if (0 != obj->init(param)) {
        return false;
      }
      serials_[id] = obj;
  }
  return true;
}

bool serial_center::deinit(const std::string &dev)
{
  bool ret = stop(dev);
  if (!dev.empty()) {
    auto cit = serials_.find(dev);
    if (cit != serials_.end()) {
      serials_.erase(cit);
    }
  } else {
    serials_.clear();
  }
  return ret;
}

bool serial_center::start(const serial_dev_t &dev)
{
  bool ret = true;
  if (!dev.empty()) {
      return serials_.find(dev) != serials_.end()?
        0 == serials_[dev]->start() : false;
  }

  for (auto &obj : serials_) {
    int sret = obj.second->start();
    if (0 != sret) {
      LOGP_ERROR("start serial(%s) error, %d",obj.second->get_id().c_str(), sret);
      ret &= false;
    }
  }
  return ret;
}

bool serial_center::stop(const serial_dev_t &dev)
{
  bool ret = true;
  if (!dev.empty()) {
    return serials_.find(dev) != serials_.end()?
      0 == serials_[dev]->stop() : false;
  }

  for (auto &obj : serials_) {
    int sret = obj.second->stop();
    if (0 != sret) {
      LOGP_ERROR("stop serial error, %d", sret);
      ret &= false;
    }
  }
  return ret;
} 

void serial_center::reg(const serial_dev_t &dev, const serial_client_t &client) 
{
  auto iter = serials_.find(dev);
  if (iter != serials_.end()) {
    iter->second->reg(client);
  } else {
    LOGP_WARN("can't find this serial(%s)", dev.c_str());
  }
}

void serial_center::unreg(const serial_dev_t &dev, const client_id_t &id)
{
  if (id.empty()) {
    serials_.erase(serials_.find(dev));
  } else {
    auto cit = serials_.find(dev);
    if (cit != serials_.end() && cit->second) {
      cit->second->unreg(id);
    }
  }
}

}  // namespace serial
}  // namespace comwise
