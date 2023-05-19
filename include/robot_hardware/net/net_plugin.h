#ifndef __COMWISE_NET__NET_PLUGIN__H__
#define __COMWISE_NET__NET_PLUGIN__H__

#include <atomic>
#include <functional>
#include <future>
#include <string>

namespace std {
    class thread;
}

namespace comwise {
namespace net {

class net_plugin
{
public:
    class net_param_t {
    public:
        std::string device;
        std::string ip;
        uint16_t port;
    };
    enum plugin_code_t {kPluginOK, kPluginWarn, KPluginError};
    using status_cb_t = std::function<void(int, const std::string &)>;

public:
    net_plugin(const std::string &id = "");
    virtual ~net_plugin();

    virtual int execute();
    virtual int execute(const std::string &dev, const std::string &ip);

    void set_network(const net_param_t &param);
    void set_network(const std::string &dev, const std::string &ip);

    void set_status_callback(const status_cb_t &cb) { status_cb_ = cb; }

protected:
    //! update network service
    void update_network();
    
    //! update network configure
    bool set_network(std::string &err);

private:
    std::string id_;
    uint64_t err_count_{0};
    net_param_t net_param_;

    std::atomic<bool> is_loop_ {false};
    std::atomic<bool> is_setting_ {false};

    std::future<void> future_;

    status_cb_t status_cb_{nullptr};
};

} // namespace net
} // namespace comwise
 
#endif // __COMWISE_NET__NET_PLUGIN__H__
