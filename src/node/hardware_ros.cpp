#include "node/hardware_ros.h"
#include "ros/ros_node.h"
#include "var/var.h"
#include "log/log.h"
#include <ros/ros.h>
#include "ros_node/std_api.h"
#include "ros_node/std_json.h"
#include "ros_node/move_cmd.h"
#include "ros_node/move_feedback.h"
#include <sensor_msgs/Joy.h>
#include "json/json.h"
#include "srv/hardware_interface.h"

namespace comwise {
namespace node {

hardware_ros::hardware_ros(std::shared_ptr<srv::hardware_interface> interface)
    : interface_(interface)
{
    
}

hardware_ros::~hardware_ros()
{
    deinit();
}

bool hardware_ros::init()
{
     ros_handle_ = ROS_NODE->node_handle();
    if (nullptr == ros_handle_) {
        LOGP_ERROR("current ros node object is null");
        return false;
    }

    if (nullptr == interface_) {
        LOGP_ERROR("runner interface object is null");
        return false;
    }

    using namespace ros_node;

    // set config
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiSetConfigServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->set_config(req.data, res.data);
                        res.msg = res.data;
                        res.code = 0;
                        res.data = "";
                    } else {
                        res.code = 500;
                        res.msg = "set config error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 501;
                    res.msg = std::string("set config excepiton: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));
    
    // get config
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiGetConfigServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->get_config(req.id, res.data);
                        res.msg = "get config success";
                        res.code = 0;
                    } else {
                        res.code = 510;
                        res.msg = "get config error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 511;
                    res.msg = std::string("get config excepiton: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));

    // get component
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiGetComponentServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->get_component(req.type, res.data);
                        res.msg = "get component success";
                        res.code = 0;
                    } else {
                        res.code = 510;
                        res.msg = "get component error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 511;
                    res.msg = std::string("get component excepiton: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));

    // get template
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiGetTemplateServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->get_template(req.module, req.type, res.data);
                        res.msg = "get template success";
                        res.code = 0;
                    } else {
                        res.code = 510;
                        res.msg = "get template error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 511;
                    res.msg = std::string("get template exception: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));

    // get comment
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiGetCommentServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->get_comment(req.module, req.type, res.data);
                        res.msg = "get comment success";
                        res.code = 0;
                    } else {
                        res.code = 510;
                        res.msg = "get comment error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 511;
                    res.msg = std::string("get comment excepiton: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));

    // get status
    ros_srvs_.push_back(std::make_shared<::ros::ServiceServer>(
        ros_handle_->advertiseService<std_api::Request, std_api::Response>(
            kApiGetStatusServiceName, [&](std_api::Request &req, std_api::Response &res) {
                try {
                    if (interface_) {
                        interface_->get_status(req.id, res.data);
                        res.msg = "get status success";
                        res.code = 0;
                    } else {
                        res.code = 510;
                        res.msg = "get status error: interface object is not initialized";
                        res.data = "";
                    }
                } catch(std::exception &e) {
                    res.code = 511;
                    res.msg = std::string("get status excepiton: ") + e.what();
                    res.data = "";
                }
                return true;
            }, ::ros::VoidConstPtr())));

    ros_subs_.emplace_back(std::make_shared<::ros::Subscriber>(
        ros_handle_->subscribe<ros_node::move_cmd>(kMoveCmdTopic, 20,
            [&](const ros_node::move_cmd::ConstPtr& vel) {
                if(interface_ && vel) {
                    move_cmd_t cmd;
                    cmd.type = vel->model;
                    cmd.priority = (vel_priority_t)vel->level;
                    cmd.velocity.resize(1);
                    cmd.velocity[0].v = vel->velocity;
                    cmd.velocity[0].w = vel->omega;
                    cmd.velocity[0].angle = vel->angle;
                    interface_->set_move_cmd(cmd);
                }
        })));

    ros_pubs_[kMoveFeedbackTopic] = std::make_shared<::ros::Publisher>(
        ros_handle_->advertise<ros_node::move_feedback>(kMoveFeedbackTopic, 20));

    ros_pubs_[kJoyStickTopic] = std::make_shared<::ros::Publisher>(
        ros_handle_->advertise<sensor_msgs::Joy>(kJoyStickTopic, 20));

    
    interface_->set_data_cb([&](const std::string &id, const any &data){
        if (id == "joy_data") {
            auto joy_data = common::any_cast<comwise::joy_data>(data);
#ifdef DEBUG_JOY_DATA
            std::stringstream ss;
            ss << "[ros] " << joy_data.time;
            ss << " button:";
            for(auto &item : joy_data.buttons) {
              ss << " " << item;
            }
            ss << " axes:";
            for(auto &item : joy_data.axes) {
              ss << " " << item;
            }
            std::cout << ss.str() << std::endl;
#endif
            sensor_msgs::Joy msg;
            msg.header.stamp.fromNSec(joy_data.time);
            msg.axes = joy_data.axes;
            msg.buttons = joy_data.buttons;
            ros_pubs_[kJoyStickTopic]->publish(msg);
        }

        if (id == "move_feedback") {
            auto vel = common::any_cast<vel_t>(data);
#ifdef DEBUG_MOVE_DATA
            printf("[ros] feedback: v=%f, w=%f\n", vel.v, vel.w);
#endif
            ros_node::move_feedback msg;
            msg.velocity    = vel.v;
            msg.omega       = vel.w;
            msg.angle       = vel.angle;
            ros_pubs_[kMoveFeedbackTopic]->publish(msg);
        }
    });

    return true;
}

bool hardware_ros::deinit()
{
    return true;
}

} // namespace node
} // namespace comwise