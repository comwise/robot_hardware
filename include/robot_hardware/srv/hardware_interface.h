#ifndef __COMWISE_SRV__RUNNER_INTERFACE__H__
#define __COMWISE_SRV__RUNNER_INTERFACE__H__

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <chrono>
#include <mutex>
#include "var/var.h"
#include "common/any.h"

namespace comwise {
namespace etc {
class etc_provider;
}
namespace srv {
class hardware_manage;

class hardware_interface
{
public:
    using data_cb_t = std::function<void(const std::string &id, const any &data)>;
    using component_vector_t = std::vector<std::string>;
    using component_map_t = std::map<std::string, component_vector_t>;
    using status_map_t = std::map<std::string, int>;
    using status_type_t = std::map<std::string, status_map_t>;
public:
    explicit hardware_interface(std::shared_ptr<hardware_manage> srv_obj,
         std::shared_ptr<etc::etc_provider> etc_obj);
    virtual ~hardware_interface();

    //!> get and set config
    int get_config(const std::string &id, std::shared_ptr<etc_obj_t> &param);
    int get_config(const std::string &request, std::string &response);
    int set_config(const std::string &request, std::string &response);

    //!> get template and comment
    int get_template(const std::string &major, const std::string &minor, std::string &templates);
    int get_comment(const std::string &major, const std::string &minor, std::string &comments);

    //!> get all component
    int get_component(const std::string &id, std::string &components);
    int get_component(const std::string &id, component_map_t &components);
    int get_component(const int type, component_vector_t &components);
    int get_component(const int type, component_map_t &components);

    //!> status
    int get_status(const std::string &id, std::string &status);
    int get_status(const int type, status_map_t &status);
    int get_status(const int type, status_type_t &status);
    int get_status(const std::string &id, status_map_t &status);
    int get_status(const std::string &id, status_type_t &status);

    //!> chassis
    int set_move_cmd(const move_cmd_t &cmd);
    int get_move_feedback(move_cmd_t &cmd);

    //!> set data callback
    void set_data_cb(const data_cb_t &cb) { data_cb_ = cb; }

private:
    void init();
    void deinit();

private:
    std::shared_ptr<hardware_manage> runner_service_{nullptr};
    std::shared_ptr<etc::etc_provider> etc_provider_{nullptr};
    data_cb_t data_cb_{nullptr};
};

} // namespace srv
} // namespace comwise
 
#endif // __COMWISE_SRV__RUNNER_INTERFACE__H__
