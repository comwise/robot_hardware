#include "net/net_plugin.h"
#include "log/log.h"
#include "common/make_thread.h"
#include "net/net_utils.h"

namespace comwise {
namespace net {

net_plugin::net_plugin(const std::string &id)
    : id_(id)
{

}

net_plugin::~net_plugin()
{
    is_loop_ = false;
    is_setting_ = false;
}

int net_plugin::execute()
{
    if (!is_setting_) {
        is_loop_ = true;
        future_ = std::async(std::launch::async, 
            &net_plugin::update_network, this);
    }
    return 0;
}

int net_plugin::execute(const std::string &dev, const std::string &ip)
{
    set_network(dev, ip);
    return execute();
}

void net_plugin::update_network()
{
    bool ret = false;
    err_count_ = 0;
    std::stringstream ss;
    do {
        is_setting_ = true;
        ss.str("");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!net_param_.ip.empty() && !net_param_.device.empty()) {
            std::string err;
            ret = set_network(err);
            if (!ret) {
                ss << "set subprocess(" << id_ <<") network(" << net_param_.device << ", " << net_param_.ip << ") error, " << err;
                if(status_cb_) {
                    status_cb_(kPluginWarn, ss.str());
                }
                if ((++err_count_) % 10 == 1) {
                    err_count_ = 1;
                    LOG_STREAM_ERROR << ss.str();
                }
                continue;
            } else {
                ss << "set subprocess(" << id_ <<") network(" << net_param_.device << ", " << net_param_.ip << ") ok";
                if(status_cb_) {
                    status_cb_(kPluginOK, ss.str());
                }
                LOG_STREAM_INFO << ss.str();
                break;
            }
        } else {
            LOG_STREAM_INFO << "set subprocess(" << id_ <<") network(" << net_param_.device << ", " << net_param_.ip << ") none";
            break;
        }

    } while (is_loop_ && !ret);
    is_setting_ = false;
}

bool net_plugin::set_network(std::string &err)
{
    bool ret = true; 
    try {
        err = "";
        bool setup_net = nmcli_setup_new_con(net_param_.device, net_param_.ip);
        if(!setup_net) {
            err += "set nmcli_setup_new_con error";
        }
         ret &= setup_net;
        
        bool up_net = nmcli_up_con(net_param_.device);
        if(!up_net) {
            if(!err.empty()) {
                err += ", ";
            }
            err += "set nmcli_up_con error";
        }
        ret &= up_net; 
    } catch (const std::exception &e) {
        std::stringstream ss;
        ss << "set network(" << net_param_.device << ", " << net_param_.ip << ") exception, " << e.what();
        err = ss.str();
        LOG_STREAM_ERROR << ss.str();
        ret = false;
    }
    return ret;
}

void net_plugin::set_network(const net_param_t &param)
{
    set_network(param.device, param.ip);
}

void net_plugin::set_network(const std::string &dev, const std::string &ip)
{
    if(dev != net_param_.device || ip != net_param_.ip) {
        LOG_STREAM_INFO << "subprocess(" << id_ <<") network changed: (" << net_param_.device << ", "
            << net_param_.ip << ") -> (" << dev << ", " << ip << ")";
        is_loop_ = false;
        is_setting_ = false;
    }
    net_param_.device = dev;
    net_param_.ip = ip;
}

} // namespace net
} // namespace comwise

