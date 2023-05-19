#ifndef __MODBUS_TCP_INCLUDE_H__
#define __MODBUS_TCP_INCLUDE_H__

#include "modbus.h"

namespace modbus {

class modbus_tcp : public modbus_driver
{
public:
    explicit modbus_tcp(const std::string &id, int type = 0);
    virtual ~modbus_tcp();

    virtual modbus_code_t init(const modbus_param_ptr &device_param) override;
    virtual modbus_code_t deinit() override;

    virtual modbus_code_t read_bits(std::vector<char> &data, uint8_t size) override;
    virtual modbus_code_t read_register(std::vector<uint16_t> &data, uint8_t size) override;
    virtual modbus_code_t write_bits(const std::vector<char> &data, uint8_t size) override;
    virtual modbus_code_t write_register(const std::vector<uint16_t> &data, uint8_t size);

private:
    modbus_t* mb_tcp_ptr_{nullptr};

};

}

#endif // __MODBUS_TCP_INCLUDE_H__