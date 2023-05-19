#ifndef __COMWISE_ETC__ETC_TEMPLATE__H__
#define __COMWISE_ETC__ETC_TEMPLATE__H__

#include <memory>
#include <string>
#include <map>
#include "etc_cfg.h"

namespace comwise {
namespace etc {

class etc_template : public etc_parse
{
public:
    explicit etc_template(const std::string &file);
    virtual ~etc_template();

    virtual bool read(const std::string &id, std::string &value) override;
    virtual bool read(const std::string &major, const std::string &minor, std::string &value) override;
    virtual bool read(const std::string &id, etc_param_t &value) override;
    virtual bool read(const std::string &major, const std::string &minor, etc_param_t &value) override;

private:
    virtual bool parse(const json_value_t &value) override;

    virtual bool write(const std::string &id, const std::string &value) override { return false; }
    virtual bool write(const std::string &major, const std::string &minor, const std::string &value) override { return false; }
    virtual bool write(const etc_id_t &id, const etc_param_t &value) override { return false; }
    virtual bool write(const std::string &major, const std::string &minor, const etc_param_t &value) override { return false; }

private:
    std::map<std::string, etc_param_t> type_index_;
    std::map<std::string, etc_value_t> id_index_;
    std::shared_ptr<json_value_t> template_;
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_TEMPLATE__H__
