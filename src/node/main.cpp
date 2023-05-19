#include "node/hardware_node.h"
#include "log/log.h"
#include "ros/ros_node.h"

int main()
{
    int quit_code = 0;
    std::string cfg_file = "$/hardware_config.json";
    std::shared_ptr<comwise::node::hardware_node> node {nullptr};
    bool is_exit_ok = false;

    try {
        LOG_STREAM_INFO << "hardware_node init ros node ...";
        ROS_NODE->start();

        bool ret = false;
        node = std::make_shared<comwise::node::hardware_node>(cfg_file);
        if(nullptr == node) {
            LOG_STREAM_ERROR << "hardware_node create object error";
            return -1;
        }
        ret = node->init();
        LOG_STREAM_INFO << "hardware_node init, return = " << ret;

        ret = node->start();
        LOG_STREAM_INFO << "hardware_node is starting ...";
        
        ROS_NODE->spin();

        ret = node->deinit();
        LOG_STREAM_INFO << "hardware_node exit, return = " << ret;
        quit_code = 0;
        is_exit_ok = true;

    } catch (const std::exception &e) {
        LOG_STREAM_ERROR << "hardware_node run exception, " << e.what();
        quit_code = -2;
        is_exit_ok = true;
    }
    return quit_code;
}
