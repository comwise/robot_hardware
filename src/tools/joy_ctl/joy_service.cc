#include "joy_service.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Accel.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include "ros_node/move_cmd.h"
#include "ros_node/move_feedback.h"
#include "joy_ctl_factory.h"
#include "common/string.h"
#include "common/linux_chcwd.h"

namespace comwise {
namespace joy {

namespace {

#define SPD_INFO  printf
#define SPD_ERROR printf

static constexpr char kJoyTopic[]           = "/comwise/joy";
static constexpr char kMoveCmdTopic[]       = "/comwise/move_cmd";
static constexpr char kMoveFeedbackTopic[]  = "/comwise/move_feedback";
static constexpr char kMoveBrakeTopic[]     = "/comwise/move_brake";

}

joy_service::joy_service(const std::string &file)
    : cfg_file_(file)
{

}

joy_service::~joy_service()
{
    deinit();
}

bool joy_service::init(const std::string &file)
{
    if (is_init_) {
        return true;
    }

    //!> 1.read config
    cfg_file_ = file;
    std::string dir = common::get_exe_dir();
    auto pos = dir.find("bin");
    if(pos != std::string::npos) {
        dir = dir.substr(0, pos-1);
    }
    cfg_file_ = dir + "/etc/hardware_joystick.yaml";

    std::ifstream ifs;
    ifs.open(cfg_file_.c_str());
    if (ifs) {
        std::string str;
        while (ifs >> str) {
            std::size_t pos = str.find(":");
            if (pos != std::string::npos) {
                std::string app = str.substr(0, pos);
                bool enable = atoi(str.substr(pos+1).c_str()) == 1;
                joy_cfgs_[app] = enable;
            }
        }
    }

    //!> 2. ros message
    ros::NodeHandle nh;
    move_cmd_pub_ = std::make_shared<::ros::Publisher>(
        nh.advertise<ros_node::move_cmd>(kMoveCmdTopic, 20));

    joy_sub_ = std::make_shared<::ros::Subscriber>(
        nh.subscribe<sensor_msgs::Joy>(kJoyTopic, 20,
        static_cast<boost::function<void (const sensor_msgs::Joy::ConstPtr&)>>(
            [&](const sensor_msgs::Joy::ConstPtr &msg) {
                joy_data data;
                data.time = msg->header.stamp.toNSec()/1000000;
                data.axes = msg->axes;
                data.buttons = msg->buttons;
                joy_handler(data);
            }
        )
    ));

    move_feedback_sub_ = std::make_shared<::ros::Subscriber>(
        nh.subscribe<ros_node::move_feedback>(kMoveFeedbackTopic, 20,
        [&] (const ros_node::move_feedback::ConstPtr &msg) {
            std::lock_guard<std::mutex> lck(move_vel_mtx_);
            if (msg) {
                move_vel_.resize(1);
                move_vel_[0].v = msg->velocity;
                move_vel_[0].w = msg->omega;
            }
        }
    ));

    //!> 3. start app
    for (auto &cfg : joy_cfgs_) {
        if (cfg.first.empty() || !cfg.second || !JOY_FACTORY) {
            continue;
        }
        auto app = JOY_FACTORY->create(cfg.first);
        if (nullptr == app) {
            continue;
        }
        joy_apps_[cfg.first] = app;
        app->on("move_cmd", [&](const common::any &data) {
            auto vel = common::any_cast<vel_cmd_t>(data);
#ifdef DEBUG_MOVE_DATA
            printf("[srv] feedback: v=%f, w=%f\n", vel.vel[0].v, vel.vel[0].w);
#endif
            set_speed(vel);
        });
    }

    SPD_INFO("init joy_service ok\n");
    is_init_ = true;

    return is_init_;
}

bool joy_service::deinit()
{
    if (is_deinit_) {
        return true;
    }
    is_deinit_ = true;

    SPD_INFO("deinit joy_service ok\n");

    return true;
}

bool joy_service::start()
{
    for (auto &app : joy_apps_) {
        if (app.second) {
            int ret = app.second->start();
            SPD_INFO("start app(%s) return %d\n", app.first.c_str(), ret);
        }
    }
    return true;
}

bool joy_service::stop()
{
    for (auto &app : joy_apps_) {
        if (app.second) {
            int ret = app.second->stop();
            SPD_INFO("stop app(%s) return %d\n", app.first.c_str(), ret);
        }
    }
    return true;
}

void joy_service::joy_handler(const joy_data &data)
{
    for (auto &app : joy_apps_) {
        if (app.second) {
            app.second->set_joy_data(data);
        }
    }
}

void joy_service::set_speed(const vel_cmd_t &vel_cmd)
{
    if (vel_cmd.vel.size() != 1) {
        return;
    }
    ros_node::move_cmd send_cmd;
    send_cmd.model     = vel_cmd.type;
    send_cmd.level     = vel_cmd.priority;
    send_cmd.velocity  = vel_cmd.vel[0].v;
    send_cmd.omega     = vel_cmd.vel[0].w;
    if (move_cmd_pub_) {
        move_cmd_pub_->publish(send_cmd);
    }
#ifdef MOVE_DEBUG
    printf("move_cmd: (model, priority, v, w,)=(%d, %d, %lf, %lf)\n",
        vel_cmd.type, vel_cmd.priority, vel_cmd.vel[0].v, vel_cmd.vel[0].w);
#endif
}

bool joy_service::get_speed(vel_list_t &vel)
{
    std::lock_guard<std::mutex> lck(move_vel_mtx_);
    vel = move_vel_;
    return true;
}

void joy_service::set_brake()
{
    ros::NodeHandle nh;
    geometry_msgs::Accel ros_msg {};
    if (nullptr == move_brake_pub_) {
        move_brake_pub_ = std::make_shared<::ros::Publisher>(
            nh.advertise<geometry_msgs::Accel>(kMoveBrakeTopic, 1));
    }
    if (move_brake_pub_) {
        move_brake_pub_->publish(ros_msg);
    }
#ifdef MOVE_DEBUG
    printf("brake: (v, w)=(%lf, %lf)\n", ros_msg.linear.x, ros_msg.angular.z);
#endif
}

} // namespace joy
} // namespace comwise
