#include "modbus_tcp.h"
#include "modbus_rtu.h"
#include "modbus.h"
#include <unistd.h>
#include <iostream>

int main()
{
#ifdef MODBUS_TCP
    tcp_modbus test

    auto parm_ = std::make_shared<modbus::modbus_tcp_param>();
    parm_->address="192.168.92.10";
    modbus::modbus_tcp modbus("123");
    modbus.init(parm_);
    std::vector<char> w_data(100,false);
    std::vector<char> r_data(1000,false);

    while(true){
        w_data[2] =true;
        int w_flag1 = modbus.write_bits(w_data,8);
        sleep(2);
        int r_flag1 = modbus.read_bits(r_data,8);
        std::cout<<(int)r_data[2]<<std::endl;
        sleep(1);
        w_data[2] =false;
        int w_flag2 = modbus.write_bits(w_data,8);
        sleep(2);
        int r_flag2 = modbus.read_bits(r_data,8);
        std::cout<<(int)r_data[2]<<std::endl;
        sleep(1);

    }
#endif

    // rtu_modbus test
    auto parm_ = std::make_shared<modbus::modbus_rtu_param>();
    parm_->device_name = "io_light";
    modbus::modbus_rtu modbus("123");
    modbus.init(parm_);
    std::vector<char> w_data(100, false);
    std::vector<char> r_data(1000, false);

    while (true) {
        w_data[2] = true;
        int w_flag1 = modbus.write_bits(w_data, 8);
        sleep(2);
        int r_flag1 = modbus.read_bits(r_data, 8);
        std::cout << (int)r_data[2] << std::endl;
        sleep(1);
        w_data[2] = false;
        int w_flag2 = modbus.write_bits(w_data, 8);
        sleep(2);
        int r_flag2 = modbus.read_bits(r_data, 8);
        std::cout << (int)r_data[2] << std::endl;
        sleep(1);
    }
    return 0;
}