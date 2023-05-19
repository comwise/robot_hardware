#ifndef __COMWISE_NODE__RUNNER_NODE__H__
#define __COMWISE_NODE__RUNNER_NODE__H__

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include <mutex>

namespace std {
    class thread;
}

namespace comwise {
namespace etc {
    class etc_subject;
    class etc_observer;
    class etc_provider;
}
namespace srv {
    class hardware_manage;
    class hardware_interface;
}
namespace node {

class hardware_ros;
class hardware_node
{
    using etc_subject_t = etc::etc_subject;
    using etc_observer_t = etc::etc_observer;
    using hw_service_t = srv::hardware_manage;
    using hw_interface_t = srv::hardware_interface;
    using etc_provider_t = etc::etc_provider;
public:
    hardware_node(const std::string &filename, bool node_type = true);
    ~hardware_node();

    //! init/deinit node
    bool init();
    bool deinit();

    //! start/stop node
    bool start();
    bool stop();

  private:
    bool create_object();
    bool init_object();

private:
    std::string cfg_file_ = "$/robot_hardware.json";
    bool is_ros_{false};

    bool is_init_{false};
    bool is_deinit_{false};

    std::unique_ptr<hardware_ros> runner_ros_;
    std::shared_ptr<etc_subject_t> etc_subject_{nullptr};
    std::shared_ptr<etc_observer_t> etc_observer_{nullptr};
    std::shared_ptr<hw_service_t> runner_srv_{nullptr};
    std::shared_ptr<hw_interface_t> runner_interface_{nullptr};
    std::shared_ptr<etc_provider_t> etc_provider_{nullptr};
};

} // namespace node
} // namespace comwise

#endif // __COMWISE_NODE__RUNNER_NODE__H__
