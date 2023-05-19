#ifndef __COMWISE_ETC__ETC_CFG__H__
#define __COMWISE_ETC__ETC_CFG__H__

#include <memory>
#include <string>
#include "etc_arg.h"

namespace Json {
    class Value;
}

namespace comwise {
namespace etc {

using json_value_t = Json::Value;

class etc_cfg
{
public:
    etc_cfg(const std::string &file) : cfg_file_(file) { }
    virtual ~etc_cfg() { }
    
    virtual bool load(std::string &err = kEmptyStr) { return false; }
    virtual bool save(bool is_format = false) { return false; }

    virtual bool read(const std::string &request, std::string &response) { return false; }
    virtual bool write(const std::string &request, const std::string &response) { return false; }

protected:
    std::string cfg_file_;
};

class etc_json : public etc_cfg
{
public:
    etc_json(const std::string &file);
    virtual ~etc_json();

    virtual bool load(std::string &err = kEmptyStr) override;
    virtual bool save(bool is_format = true) override;

    // read/write json string by id
    virtual bool read(const std::string &id, std::string &value) { return false; }
    virtual bool write(const std::string &id, const std::string &value) { return false; }

    // read/write json string by major and minor type
    virtual bool read(const std::string &major, const std::string &minor, std::string &value) { return false; }
    virtual bool write(const std::string &major, const std::string &minor, const std::string &value) { return false; }

    // read/write c++ object by id
    virtual bool read(const etc_id_t &id, etc_param_t &value) { return false; }
    virtual bool write(const etc_id_t &id, const etc_param_t &value) { return false; }

    // read/write c++ object by major and minor type
    virtual bool read(const std::string &major, const std::string &minor, etc_param_t &value) { return false; }
    virtual bool write(const std::string &major, const std::string &minor, const etc_param_t &value) { return false; }

protected:
    virtual bool parse(const json_value_t &value);
    virtual bool pack(json_value_t &value);

protected:
    std::shared_ptr<json_value_t> cfg_obj_;
};

class etc_parse : public etc_json
{
public:
    etc_parse(const std::string &file);
    virtual ~etc_parse();

    virtual bool parse_value(const json_value_t &value,
            uint32_t major, uint32_t minor, etc_value_t &etc_val);
    virtual bool parse_value(const json_value_t &value,
            const std::string &major, const std::string &minor, etc_value_t &etc_val);
    virtual bool pack_value(etc_value_t &etc_val, json_value_t &json_val);

    //!> parse module
    virtual bool parse_motor    (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_chassis  (const json_value_t &json_val, etc_value_t &etc_val);
    
    virtual bool parse_camera   (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_laser    (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_laser2d  (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_laser3d  (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_can      (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_net      (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_serial   (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_modbus   (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_io       (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_carrier  (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_joy      (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_sensor  (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_audio    (const json_value_t &json_val, etc_value_t &etc_val);
    virtual bool parse_tf       (const json_value_t &json_val, etc_value_t &etc_val);

    //!> pack module
    virtual bool pack_motor     (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_chassis   (etc_value_t &etc_val, json_value_t &json_val);
    
    virtual bool pack_camera    (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_laser     (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_can       (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_net       (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_serial    (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_modbus    (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_io        (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_carrier   (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_joy       (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_sensor    (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_audio     (etc_value_t &etc_val, json_value_t &json_val);
    virtual bool pack_tf        (etc_value_t &etc_val, json_value_t &json_val);

    virtual bool parse_calib(const json_value_t &obj, calib_param_t &param);
    virtual bool pack_calib(json_value_t &obj, const calib_param_t &param);

    template <typename T>
    bool parse(const json_value_t &json, T &val);
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_CFG__H__
