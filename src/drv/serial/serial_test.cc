#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "common/ring_buffer.h"
#include "serial/serial_driver.h"
#include "serial/serial_bridge.h"
#include "serial/serial_reader.h"

//#define SERIAL_READER
#define SERIAL_BRIDGE

int main()
{
#ifdef RING_BUFFER
    const int kSize = 32;
    const int kReadSize = 6;
    const int kWriteSize = 6;
    char buffer[kSize] = {};
    ring_buffer_t ring_buff;
    ring_initialize(&ring_buff, buffer, kSize);
    std::thread w_t([&]() {
        while (true)
        {
            char w_buff[kWriteSize] = "12345";
            ring_write(&ring_buff, w_buff, kWriteSize);
#ifdef W_DEBUG
            std::cout << "[w](" << std::dec << std::setw(2) << std::setfill(' ') 
                << ring_buff.first << "," << std::setw(2) << std::setfill(' ') << ring_buff.last << ")";
            for (uint32_t i = 0; i < kSize; i++) {
              std::cout << " " << std::setw(2) << std::setfill('0') << (uint16_t)(buffer[i]);
            }
            std::cout << std::endl;
#endif // W_DEBUG
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });
#ifdef R_THREAD
    std::thread r_t([&]() {
        while (true)
        {
            char r_buff[kReadSize] = {};
            ring_read(&ring_buff, r_buff, kReadSize);
#ifdef R_DEBUG
            std::cout << "[r](" << std::dec << std::setw(2) << std::setfill(' ') 
                << ring_buff.first << "," << std::setw(2) << std::setfill(' ') << ring_buff.last << ")";
            for (uint32_t i = 0; i < kSize; i++) {
              std::cout << " " << std::setw(2) << std::setfill('0') << (uint16_t)(buffer[i]);
            }
            std::cout << std::endl;
#endif // R_DEBUG

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } });
#endif // R_THREAD
#endif // RING_BUFFER


    auto cb = [&](comwise::serial::serial_data_t &data)->bool{
        for (uint32_t i = 0; i < data.size(); i++) {
            std::cout << " " << std::setw(2) << std::setfill('0') << std::hex << (uint16_t)(data[i]);
        }
        std::cout << std::endl;
        return true;
    };


    comwise::serial::serial_param_t param;
    param.device   = "/dev/ttyUSB0";
    param.baudrate = 115200;
    param.buffer.header.push_back(0x55);

#ifdef SERIAL_BRIDGE
    bool iret = SERIAL_CENTER->init(param);
    bool sret = SERIAL_CENTER->start(param.device);

    comwise::serial::serial_client_t client;
    client.id = "test";
    client.read = cb;
    SERIAL_CENTER->reg(param.device, client);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    SERIAL_CENTER->unreg(param.device);
    SERIAL_CENTER->deinit("");
#endif

#ifdef SERIAL_READER
    comwise::serial::serial_reader reader("wit61");
    reader.init(param);
    reader.start();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    reader.stop();
    reader.deinit();
#endif

    return 0;
}