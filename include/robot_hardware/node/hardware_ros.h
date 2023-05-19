#ifndef __COMWISE_NODE__RUNNER_ROS__H__
#define __COMWISE_NODE__RUNNER_ROS__H__

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

namespace ros {
    class NodeHandle;
    class ServiceServer;
    class Publisher;
    class Subscriber;
    class Timer;
}

namespace comwise {
namespace srv {
    class hardware_manage;
    class hardware_interface;
}
namespace node {

class hardware_ros
{
public:
    hardware_ros(std::shared_ptr<srv::hardware_interface> hw_interface);
    ~hardware_ros();

    bool init();
    bool deinit();

private:
    // config
    std::string cfg_file_ = "$/robot_hardware.json";

    // service
    std::shared_ptr<srv::hardware_interface> interface_{nullptr};

    // ros
    std::shared_ptr<::ros::NodeHandle> ros_handle_{nullptr};
    std::vector<std::shared_ptr<::ros::ServiceServer>> ros_srvs_;
    std::vector<std::shared_ptr<::ros::Subscriber>> ros_subs_;
    std::map<std::string, std::shared_ptr<::ros::Publisher>> ros_pubs_;
    std::mutex pubs_mtx_;

};

} // namespace node
} // namespace comwise
 
#endif // __COMWISE_NODE__RUNNER_ROS__H__
