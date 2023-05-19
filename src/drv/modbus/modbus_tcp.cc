#include "modbus_tcp.h"
#include "log/log.h"
#include "lib/modbus/modbus.h"

namespace modbus {

#define MODBUS_MAX_READ_BITS 2000
#define MODBUS_MAX_WRITE_BITS 1968

modbus_tcp::modbus_tcp(const std::string &id, int type) 
    : modbus_driver(id, type)
{

}

modbus_tcp::~modbus_tcp()
{
    deinit();
}

modbus_code_t modbus_tcp::init(const modbus_param_ptr &device_param)
{
    auto param = std::dynamic_pointer_cast<modbus_tcp_param>(device_param);
    mb_tcp_ptr_ = modbus_new_tcp(param->address.c_str(), param->port);
    if (nullptr == mb_tcp_ptr_) {
        LOGP_ERROR("create modbus_new_tcp error");
    }
    modbus_set_slave(mb_tcp_ptr_, 2);
    int connect_flag = modbus_connect(mb_tcp_ptr_);
    if (connect_flag == -1) {
        LOGP_ERROR("tcp_modbus connect failed");
    }

    modbus_set_error_recovery(mb_tcp_ptr_, (modbus_error_recovery_mode)(
        modbus_error_recovery_mode::MODBUS_ERROR_RECOVERY_LINK |
        modbus_error_recovery_mode::MODBUS_ERROR_RECOVERY_PROTOCOL));
    
    return 0;
}

modbus_code_t modbus_tcp::deinit()
{
    modbus_close(mb_tcp_ptr_);
    modbus_free(mb_tcp_ptr_);
    mb_tcp_ptr_ = nullptr;
    return 0;
}

modbus_code_t modbus_tcp::read_bits(std::vector<char> &data, uint8_t size)
{
    uint8_t buffer[MODBUS_MAX_READ_BITS] = {0};
    int nb = modbus_read_bits(mb_tcp_ptr_, 0, size, buffer);
    if (nb > 0) {
        data = std::vector<char>(buffer, buffer + nb);
    } else if (nb < 0) {
        LOGP_ERROR("read bits data failed");
    }
    return nb;
}

modbus_code_t modbus_tcp::read_register(std::vector<uint16_t> &data, uint8_t size)
{
    uint16_t buffer[MODBUS_MAX_READ_BITS] = {0};
    int nb = modbus_read_registers(mb_tcp_ptr_, 0, size, buffer);
    if (nb > 0) {
        data = std::vector<uint16_t>(buffer, buffer + nb);
    } else if (nb < 0) {
        LOGP_ERROR("read registers data failed");
    }
    return nb;
}

modbus_code_t modbus_tcp::write_bits(const std::vector<char> &data, uint8_t size)
{
    if (data.size() <= MODBUS_MAX_WRITE_BITS) {
        return modbus_write_bits(mb_tcp_ptr_, 0, size, (const uint8_t *)data.data());
    } else {
        LOGP_ERROR("write_bits data oversize");
        return -1;
    }
}

modbus_code_t modbus_tcp::write_register(const std::vector<uint16_t> &data, uint8_t size)
{
    if (data.size() <= MODBUS_MAX_WRITE_BITS) {
        return modbus_write_registers(mb_tcp_ptr_, 0, size, (const uint16_t *)data.data());
    } else {
        LOGP_ERROR("write_registers data oversize");
        return -1;
    }
}

} // namespace modbus
