#include "net/net_utils.h"
#include <Poco/Process.h>
#include <Poco/PipeStream.h>
#include <sys/capability.h>
#include <sys/prctl.h>
#include <ifaddrs.h>
#include <future> // std::async, std::future
#include <chrono> // std::chrono::milliseconds
#include "common/linux_chcwd.h"
#include "log/log.h"

//#define ENABLE_TEST

namespace {

    bool set_inherit_ambient_cap(cap_value_t *newcaps, int size)
    {
        cap_t caps = cap_get_proc();
        if(caps == nullptr) {
            LOG_STREAM_WARN << "Get proc cap fail...";
            return false;
        }
#ifdef ENABLE_TEST
        SSPD_LOG_INFO_F("Current capabilities: {}", cap_to_text(caps, NULL));
#endif
        if(cap_set_flag(caps, CAP_INHERITABLE, size, newcaps, CAP_SET) == -1) {
            LOG_STREAM_WARN << "Set cap flag fail...";
            return false;
        }
        if(cap_set_proc(caps) == -1) {
            LOG_STREAM_WARN << "Set proc cap fail...";
            return false;
        }
#ifdef ENABLE_TEST
        SSPD_LOG_INFO_F("After set capabilities: {}", cap_to_text(caps, NULL));
#endif
        if(cap_free(caps) == -1) {
            LOG_STREAM_WARN << "Free cap fail...";
            return false;
        }

        for(int i = 0; i < size; i++) {
            int ret = 0;
            if (!(ret = prctl(PR_CAP_AMBIENT, PR_CAP_AMBIENT_IS_SET, newcaps[i], 0, 0))) {
                if ((ret = prctl(PR_CAP_AMBIENT, PR_CAP_AMBIENT_RAISE, newcaps[i], 0, 0))) {
                    LOG_STREAM_WARN << "Error when prctl set ambient capabilities of " << cap_to_name(newcaps[i]) << ", code:" << ret;
                    return false;
                }
            }
        }
        return true;
    }


    bool run_sub_process(std::shared_ptr< Poco::PipeInputStream > out_s, std::shared_ptr< Poco::PipeInputStream > err_s, const std::string &name)
    {
        bool err_flag = false;
#ifdef ENABLE_STD_OUT
        while(out_s && out_s->good()) {
            std::string info;
            if(std::getline(*out_s, info)) {
                LOG_STREAM_INFO << "[" << name << "] " << info;
            }
        }
#endif

        while(err_s && err_s->good()) {
            std::string info;
            if(std::getline(*err_s, info)) {
                LOG_STREAM_WARN << "[" << name << "] " << info;
                if(info.find("don't match") != std::string::npos) {
                    err_flag = false;
                } else {
                    err_flag = true;
                }
            }
        }

        return err_flag;
    }

    bool log_sub_process(std::shared_ptr<Poco::PipeInputStream> out_s, std::shared_ptr<Poco::PipeInputStream> err_s, const std::string &name)
    {
        std::future<bool> fut = std::async(run_sub_process, out_s, err_s, name);
        std::chrono::seconds span(10);
        while (fut.wait_for(span) == std::future_status::timeout)
            break;
        return fut.get();
    }
}

namespace comwise {
namespace net {

std::string get_args(const std::string &cmd,  const std::vector<std::string> &args)
{
    std::string new_args(cmd);
    for(auto &arg : args) {
        if(!new_args.empty()) {
            new_args += " ";
        }
        new_args += arg;
    }
    return new_args;
}

std::vector< std::string > get_dev_lists()
{
    std::vector< std::string > outs;
    struct ifaddrs *addrs,*tmp;

    ::getifaddrs(&addrs);
    tmp = addrs;

    while (tmp)
    {
        if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_PACKET)
            outs.push_back(tmp->ifa_name);
        tmp = tmp->ifa_next;
    }

    freeifaddrs(addrs);
    return outs;
}

bool set_ifconfig_dev_ip_mask(const std::string &dev, const std::string &ip, const std::string &mask)
{
    cap_value_t newcaps[1] = { CAP_NET_ADMIN, };        // To get net admin capabilities for sub process
    if(!::set_inherit_ambient_cap(newcaps, 1))
        return false;

    Poco::Process::Args args;
    args.push_back(dev);
    args.push_back(ip);
    args.push_back("netmask");
    args.push_back(mask);

    std::string cmd = get_args("/sbin/ifconfig", args);

    try {
        Poco::Pipe inPipe, outPipe, errPipe;

        auto handle = Poco::Process::launch("/sbin/ifconfig", args, &inPipe, &outPipe, &errPipe);

        bool err_flag = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe),
                                          std::make_shared< Poco::PipeInputStream >(errPipe), "ifconfig-set-ip");

        int ret = Poco::Process::wait(handle);
        if (ret == 0) { 
            return !err_flag;
        } else {
            LOG_STREAM_ERROR << "nmcli ifconfig-set-ip return " << ret << ", cmd = " << get_args("/sbin/ifconfig", args);
            return false;
        }
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli ifconfig-set-ip exception(" << e.what() << "), cmd = " << cmd;
        throw;
    }
}

bool set_ifconfig_dev_mtu(const std::string &dev, int mtu)
{
    cap_value_t new_caps[1] = { CAP_NET_ADMIN };
    if(!::set_inherit_ambient_cap(new_caps, 1))
        return false;

    Poco::Process::Args args;
    args.push_back(dev);
    args.push_back("mtu");
    args.push_back(std::to_string(mtu));

    std::string cmd = get_args("/sbin/ifconfig", args);

    try {
        Poco::Pipe inPipe, outPipe, errPipe;

        auto handle = Poco::Process::launch("/sbin/ifconfig", args, &inPipe, &outPipe, &errPipe);

        bool err_flag = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe),
                                          std::make_shared< Poco::PipeInputStream >(errPipe), "ifconfig-set-mtu");

        int ret = Poco::Process::wait(handle);
        if (ret == 0) { 
            return !err_flag;
        } else {
            LOG_STREAM_ERROR << "nmcli ifconfig-set-mtu return " << ret << ", cmd = " << get_args("/sbin/ifconfig", args);
            return false;
        } 
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli ifconfig-set-mtu exception(" << e.what() << "), cmd = " << cmd;
        throw;
    }
}

bool up_if_dev(const std::string &dev, bool do_up)
{
    cap_value_t new_caps[1] = { CAP_NET_ADMIN };
    if(!::set_inherit_ambient_cap(new_caps, 1))
        return false;
    
    Poco::Process::Args args;
    args.push_back(dev);
    if(do_up)
        args.push_back("up");
    else
        args.push_back("down");

    std::string cmd = get_args("/sbin/ifconfig", args);

    try {
        Poco::Pipe inPipe, outPipe, errPipe;

        auto handle = Poco::Process::launch("/sbin/ifconfig", args, &inPipe, &outPipe, &errPipe);

        bool err_flag = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe),
                                          std::make_shared< Poco::PipeInputStream >(errPipe), do_up ? "ifconfig-up" : "ifconfig-down");

        int ret = Poco::Process::wait(handle);
        if (ret == 0) { 
            return !err_flag;
        } else {
            LOG_STREAM_ERROR << "nmcli ifconfig-up/down return " << ret << ", cmd = " << cmd;
            return false;
        }
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli ifconfig-up/down exception(" << e.what() << "), cmd = " << cmd;
        throw;
    }
}

bool down_if_dev(const std::string &dev)
{
    return up_if_dev(dev, false);
}

bool nmcli_setup_new_con(const std::string &dev, const std::string &ip_with_mask)
{
    // 1. first try to modify current exist connection
    // nmcli con mod "ens33" ipv4.addr ip/mask connection.interface-name ens33
    std::string cmd_url = common::get_absolute_path("$/cap_wrapper");

    Poco::Process::Args args;
    std::string cmd = std::string("nmcli con mod ") + dev + " ipv4.addr " + ip_with_mask + " connection.interface-name " + dev;
    args.push_back(cmd);
    try {
        Poco::Pipe inPipe, outPipe, errPipe;

#ifdef ENABLE_TEST
        LOG_STREAM_INFO << "cap_wrapper = " << cmd_url;
        LOG_STREAM_INFO << "nmcli_mod_cmd =  " << cmd;
#endif
    
        auto handle = Poco::Process::launch(cmd_url, args, &inPipe, &outPipe, &errPipe);

        bool err_flag = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe),
                                          std::make_shared< Poco::PipeInputStream >(errPipe), "nmcli_setup_new_con_using_mod");

        // mod not fail, then return, else just add one
        int ret = Poco::Process::wait(handle);
        if (ret == 0) {
            return !err_flag;
        } else {
            LOG_STREAM_ERROR << "nmcli modify con return " << ret << ", cmd = " << cmd;
        }
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli modify con excepiton(" << e.what() << "), cmd = " << cmd;
        throw;
    }

    // 2. if mod fail, then using add tool
    // nmcli con add con-name "ens33" ifname ens33 type ethernet ip4 192.168.0.99/24
    Poco::Process::Args args2;
    std::string cmd2 = std::string("nmcli con add con-name ") + dev + " ifname " + dev + " type ethernet ip4 " + ip_with_mask;
    args2.push_back(cmd2);
#ifdef ENABLE_TEST
    LOG_STREAM_INFO << "nmcli_add_cmd =  " << cmd2;
#endif
    try { 
        Poco::Pipe inPipe2, outPipe2, errPipe2;

        auto handle2 = Poco::Process::launch(cmd_url, args2, &inPipe2, &outPipe2, &errPipe2);

        bool err_flag2 = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe2),
                                          std::make_shared< Poco::PipeInputStream >(errPipe2), "nmcli_setup_new_con_using_add");

        int ret = Poco::Process::wait(handle2);
        if (ret == 0) { 
            return !err_flag2;
        } else {
            LOG_STREAM_ERROR << "nmcli add con return " << ret << ", cmd = " << cmd2;
            return false;
        }
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli add con exception(" << e.what() << "), cmd = " << cmd2;
        throw;
    }
}

bool nmcli_up_con(const std::string &name, bool do_up)
{
    Poco::Process::Args args;
    std::string cmd;
    if(do_up)
        cmd = std::string("nmcli -w 10 con up ") + name;
    else
        cmd = std::string("nmcli -w 10 con down ") + name;
    args.push_back(cmd);

#ifdef ENABLE_TEST
    LOG_STREAM_INFO << "nmcli_up/down_cmd =  " << cmd;
#endif
    try {
        Poco::Pipe inPipe, outPipe, errPipe;

        auto handle = Poco::Process::launch(common::get_absolute_path("$/cap_wrapper"), args, &inPipe, &outPipe, &errPipe);

        bool err_flag = ::log_sub_process(std::make_shared< Poco::PipeInputStream >(outPipe),
                                          std::make_shared< Poco::PipeInputStream >(errPipe), do_up ? "nmcli_up" : "nmcli_down");

        int ret = Poco::Process::wait(handle);
        if (ret == 0) { 
            return !err_flag;
        } else {
            LOG_STREAM_ERROR << "nmcli up/down con return " << ret << ", cmd = " << cmd;
            return false;
        }
    } catch(std::exception &e) {
        LOG_STREAM_ERROR << "nmcli up/down con exception(" << e.what() << "), cmd = " << cmd;
        throw;
    }
}

bool nmcli_down_con(const std::string &name)
{
    return nmcli_up_con(name, false);
}

} //namespace net
} //namespace comwise

