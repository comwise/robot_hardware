#include "ros/ros_node.h"
#include <ros/ros.h>
#include <signal.h>
#include <csignal>

namespace comwise {
namespace ros {

std::shared_ptr<ros_node> ros_node::instance_ = nullptr;

ros_node::ros_node(const std::string &name)
{
    int argc = 0;
    ::ros::init(argc, NULL, name);
    if (nullptr == ros_handle_) {
        ros_handle_ = std::make_shared<node_handle_t>();
    }
}

ros_node::~ros_node()
{
    shut_down();
}

std::shared_ptr<ros_node::node_handle_t> ros_node::node_handle()
{
    return ros_handle_;
};

bool ros_node::start()
{
    ::ros::start();
}

void ros_node::spin(bool enable_once)
{
    if (enable_once)
        ::ros::spinOnce();
    else
        ::ros::spin();
}

void ros_node::shut_down()
{
    ::ros::shutdown();
}

} // namespace ros 
} // namespace comwise
