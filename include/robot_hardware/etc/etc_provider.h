#ifndef __COMWISE_ETC__ETC_PROVIDER__H__
#define __COMWISE_ETC__ETC_PROVIDER__H__

#include "etc_arg.h"

namespace comwise {
namespace etc {

class etc_data;
class etc_subject;
class etc_launch;
class etc_template;

class etc_provider
{
public:
    explicit etc_provider(const std::string &file, std::shared_ptr<etc_subject> obj);
    ~etc_provider();

    //>! init/deinit
    bool init();
    bool deinit();

    //>! get/set all param
    etc_param_t get_calib_param() const;
    void set_calib_param(const etc_param_t &param);

    //>! get/set param by id
    etc_value_t get_calib_param(const etc_id_t &id) const;
    void set_calib_param(const etc_id_t &id, etc_value_t param);

    //>! read/write json string from memory data
    std::string read_json_param(const std::string &param = "");
    int write_json_param(const std::string &param);

    //>! read template/comment by major/minor type
    std::string read_template(const std::string &major, const std::string &minor="");
    std::string read_comment(const std::string &major, const std::string &minor="");

private:
    //>! init config and component
    bool init_config();
    bool init_component();

    //>! notify data
    void data_notify(const etc_param_t &data);

private:
    //>! config file
    std::string launch_file_;
    std::string template_file_;
    std::string cfg_file_;
    
    //>! param buffer
    etc_param_t etc_map_;

    //>! parse config from json
    bool is_first_{false};

    //>! parse data from file
    std::shared_ptr<etc::etc_subject> etc_subject_{nullptr};

    std::shared_ptr<etc::etc_data> etc_data_{nullptr};
    std::shared_ptr<etc::etc_launch> etc_launch_{nullptr};
    std::shared_ptr<etc::etc_template> etc_template_{nullptr};
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_PROVIDER__H__
