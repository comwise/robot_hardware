#ifndef __COMWISE_ETC__ETC_DATA__H__
#define __COMWISE_ETC__ETC_DATA__H__

#include <set>
#include "etc_cfg.h"

namespace comwise {
namespace etc {

class etc_data : public etc_parse
{
public:
    using data_notify_t = std::function<void(const etc_param_t&)>;

public:
    explicit etc_data(const std::string &filename);
    virtual ~etc_data();

    //!> read/write json string by id
    virtual bool read(const etc_id_t &id, std::string &data);
    virtual bool write(const etc_id_t &id, const std::string &data);

    //!> read/write c++ object by id
    virtual bool read(const etc_id_t &id, etc_param_t &data);
    virtual bool write(const etc_id_t &id, const etc_param_t &data);

    //!> read/write json object by id
    virtual bool read(json_value_t &data, const etc_id_t &id = "");
    virtual bool write(json_value_t &data, const etc_id_t &id = "");


    //!> get/set all param
    etc_param_t get_calib_param() const;
    void set_calib_param(const etc_param_t &param);

    //!> get/set param by id
    etc_value_t get_calib_param(const etc_id_t &id) const;
    void set_calib_param(const etc_id_t &id, etc_value_t param);

    //!> read/write json string from memory data
    bool write_calib_param(const etc_id_t &id, etc_value_t param);
    bool write_calib_param(const etc_param_t &param);


    //!> when data changed, then notify callback function
    void set_notify_callback(const data_notify_t &cb) { data_cb_ = cb; }

private:
    //! parse/pack data
    virtual bool parse(const json_value_t &value) override;
    virtual bool pack(json_value_t &value) override;

    bool pack_data(json_value_t &value, const etc_id_t &id = "");

    //! other function
    etc_value_t find_object(const std::string &id, etc_param_t &param);
    bool reset_object(const std::set<std::string> &ids, etc_param_t &param);

private:
    data_notify_t data_cb_{nullptr};
    etc_param_t update_param_;
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_DATA__H__
