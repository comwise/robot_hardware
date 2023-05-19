#ifndef __COMWISE_NET__NET_UTILS__H__
#define __COMWISE_NET__NET_UTILS__H__

#include <string>
#include <vector>

namespace comwise {
namespace net {

    std::vector< std::string > get_dev_lists();

    bool set_ifconfig_dev_ip_mask(const std::string &dev, const std::string &ip, const std::string &mask);
    bool set_ifconfig_dev_mtu(const std::string &dev, int mtu);

    bool up_if_dev(const std::string &dev, bool do_up = true);
    bool down_if_dev(const std::string &dev);

    bool nmcli_setup_new_con(const std::string &dev, const std::string &ip_with_mask);
    bool nmcli_up_con(const std::string &name, bool do_up = true);
    bool nmcli_down_con(const std::string &name);

} // namespace net
} // namespace comwise

#endif // __COMWISE_NET__NET_UTILS__H__
