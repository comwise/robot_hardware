#ifndef __MODBUS_INTERFACE_INCLUDE_H__
#define __MODBUS_INTERFACE_INCLUDE_H__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct _modbus;
 
namespace modbus {

typedef int modbus_code_t;

class modbus_param_base {
public:
    virtual ~modbus_param_base() { }
};
using modbus_param_ptr = std::shared_ptr<modbus_param_base>;

class modbus_tcp_param : public modbus_param_base {
public:
    std::string address;
    uint16_t port{502};
};

class modbus_rtu_param : public modbus_param_base {
public:
    std::string device_name;
    int baud{115200};
};

class modbus_driver
{
protected:
    using modbus_t = _modbus;

public:
    explicit modbus_driver(const std::string &id, int type = 0) { }
    virtual ~modbus_driver() { }

    virtual modbus_code_t init(const modbus_param_ptr &device_param) = 0;
    virtual modbus_code_t deinit() = 0;

    virtual modbus_code_t read_bits(std::vector<char> &data,uint8_t size) = 0;
    virtual modbus_code_t read_register(std::vector<u_int16_t> &data,uint8_t size) = 0;
    virtual modbus_code_t write_bits(const std::vector<char> &data,uint8_t size) = 0;
    virtual modbus_code_t write_register(const std::vector<u_int16_t> &data,uint8_t size) = 0;
};

} // namespace modbus

#endif // __MODBUS_INTERFACE_INCLUDE_H__
