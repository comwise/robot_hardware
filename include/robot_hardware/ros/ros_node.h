#ifndef __COMWISE_ROS__ROS_NODE__H__
#define __COMWISE_ROS__ROS_NODE__H__

#include <memory>
#include <string>
#include <thread>

namespace ros {
    class NodeHandle;
}

namespace comwise {
namespace ros {

static const std::string kNodeName = "hardware_node";

class ros_node {
    using node_handle_t = ::ros::NodeHandle;
public:
    static std::shared_ptr<ros_node> instance(const std::string &ins = kNodeName) {
        if (nullptr == instance_) {
            instance_ = std::shared_ptr<ros_node>(new ros_node(ins));
            while (instance_ == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return instance_;
    }
    ~ros_node();

    static bool start();
    static void spin(bool enable_once = false);
    static void shut_down();

    std::shared_ptr<node_handle_t> node_handle();

private:
    ros_node(const std::string &name = kNodeName);

    static std::shared_ptr<ros_node> instance_;
    std::shared_ptr<node_handle_t> ros_handle_{nullptr};
};

} // namespace ros
} // namespace comwise

#define ROS_NODE comwise::ros::ros_node::instance()

#endif // __COMWISE_ROS__ROS_NODE__H__
