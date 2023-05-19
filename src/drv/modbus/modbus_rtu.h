#ifndef __MODBUS_RTU_INCLUDE_H__
#define __MODBUS_RTU_INCLUDE_H__

#include "modbus.h"
 
namespace modbus {

class modbus_rtu : public modbus_driver
{
public:
    explicit modbus_rtu(const std::string &id, int type = 0);
    virtual ~modbus_rtu();

    virtual modbus_code_t init(const modbus_param_ptr &param) override;
    virtual modbus_code_t deinit() override;

    virtual modbus_code_t read_bits(std::vector<char> &data, uint8_t size);
    virtual modbus_code_t read_register(std::vector<uint16_t> &data, uint8_t size);
    virtual modbus_code_t write_bits(const std::vector<char> &data, uint8_t size);
    virtual modbus_code_t write_register(const std::vector<uint16_t> &data, uint8_t size);

private:
    modbus_t* mb_rtu_ptr_{nullptr};
};

} // namespace modbus

#endif // __MODBUS_RTU_INCLUDE_H__

