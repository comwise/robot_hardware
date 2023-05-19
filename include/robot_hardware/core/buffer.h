#include <thread>
#include <chrono>
#include "log/log.h"
#include "var/var_sensor.h"

namespace core {

class ring_buffer
{
  using buffer_param_t = comwise::buffer_param_t;

  std::shared_ptr<std::thread> thr_{nullptr};
  bool loop_{false};
  uint32_t frequency_{50};
  buffer_param_t param_;

public:
  ring_buffer() {}
  virtual ~ring_buffer() { deinit(); }

  int init(const buffer_param_t &serial_param)
  {
    param_ = serial_param;
    LOGP_INFO("init buffer ok");
    return 0;
  }

  int deinit()
  {
    return stop();
  }

  int start()
  {
    int ret = 0;
    thr_ = std::make_shared<std::thread>(
      [&]() {
          LOGP_INFO("handle data ...");
          int ret = 0;
          try {
            ret = handler();
          } catch (std::exception &e) {
            LOGP_ERROR("handle data exception: %s", e.what());
            ret = -10;
          }
      });
    LOGP_INFO("stop buffer return %d", ret);
    return ret;
  }

  int stop()
  {
    int ret = 0;
    loop_ = false;
    if (thr_ && thr_->joinable()) {
      thr_->join();
    }
    thr_ = nullptr;
    LOGP_INFO("stop buffer ok");
    return ret;
  }

  void set_frequency(int frequency)
  { 
    frequency_ = frequency; 
  }
  
  int get_frequency() const
  { 
    return frequency_; 
  }

protected:
  //#define DEBUG_RAW
  virtual int handler()
  {
    auto &kBufferConst = param_;
    uint8_t *read_data = new uint8_t[kBufferConst.read_size];

    uint8_t *buffer_data = new uint8_t[kBufferConst.buffer_size];
    memset(buffer_data, 0, kBufferConst.buffer_size);

    uint8_t *buffer_free = buffer_data;

    loop_ = true;
    while (loop_) {
      try {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/frequency_));
        auto start_time = std::chrono::steady_clock::now();

        // read data
        memset(read_data, 0, kBufferConst.read_size);
        int recv_size = read(read_data, kBufferConst.read_size);
  #ifdef DEBUG_RAW
        for (uint32_t i = 0; i < recv_size && recv_size > 0; i++) {
          printf("%02x ", read_data[i]);
        }
        printf("\n");
  #endif
        // copy receive data to buffer
        if (recv_size > 0) {
          uint32_t data_size = buffer_free - buffer_data;
          int32_t move_size = data_size + recv_size - kBufferConst.buffer_size;
          if (move_size > 0) { // remove old buffer data
            memmove(buffer_data, buffer_data + move_size, kBufferConst.buffer_size - move_size);
            buffer_free = buffer_free - move_size;
          }
          // copy receive data to free buffer
          memmove(buffer_free, read_data, recv_size);
          buffer_free = buffer_free + recv_size;
        }

        if (buffer_free - buffer_data >= kBufferConst.frame_size && kBufferConst.header.size() > 0) {
          uint8_t *find_data = (uint8_t*)memchr(buffer_data, kBufferConst.header[0], kBufferConst.buffer_size);
  #ifdef DEBUG_FRAME
          uint32_t size = buffer_free - buffer_data;
          std::cout << "[IMU](" << std::dec << size << ")";
          for (uint32_t i = 0; i < kBufferConst.frame_size; i++) {
            std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<uint16_t>(find_data[i]);
          }
          std::cout << std::endl;
  #endif
          int32_t invalid_size = find_data - buffer_data;
          if (find_data && invalid_size >= 0) { // find header data
            if (invalid_size + kBufferConst.frame_size <= kBufferConst.buffer_size) { // hava one frame packet

              // parse frame data
              int ret = parse((uint8_t *)find_data, kBufferConst.frame_size);
              if (0 != ret) {
                  LOGP_WARN("parse data error: %d", ret);
              }

              if (invalid_size + kBufferConst.frame_size == kBufferConst.buffer_size) {
                memset(buffer_data, 0, kBufferConst.buffer_size);
                buffer_free = buffer_data;
              } else {
                // remove frame data and invalid data from buffer
                uint8_t *new_data = find_data + kBufferConst.frame_size;
                int32_t data_size = buffer_free - new_data;
                if (data_size > 0) {
                  memmove(buffer_data, new_data, data_size);
                  buffer_free = buffer_data + data_size;
                }
              }
            }
          } else { // no find header, reset buffer to 0
            memset(buffer_data, 0, kBufferConst.buffer_size);
            buffer_free = buffer_data;
          }
        }
      } catch (std::exception &e) {
        LOGP_ERROR("handle buffer exception: %s", e.what());
      }
    }
    delete[] buffer_data;
    delete[] read_data;
    return 0;
  }

  virtual int read(uint8_t *data, uint32_t size) { return -1; }
  virtual int write(uint8_t *data, uint32_t size) { return -1; }
  virtual int parse(uint8_t *data, uint32_t size) { return -1; }
};

} // namespace core