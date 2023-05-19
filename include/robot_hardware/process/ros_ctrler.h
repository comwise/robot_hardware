#ifndef __COMWISE_PROCESS__ROS_CTRLER__H__
#define __COMWISE_PROCESS__ROS_CTRLER__H__

#include <memory>
#include <atomic>
#include <map>
#include <vector>
#include <string>
#include "process_ctrler.h"

namespace std {
    class thread;
}

namespace Poco {
    class ProcessHandle;
    class PipeInputStream;
}

namespace ros {
    class NodeHandle;
}

namespace comwise {
namespace process {

bool generate_xacro_xml(const std::string &xml_str,
                        const std::map<std::string, std::string> &args,
                        std::vector<std::string> &fail_args,
                        std::string &out_file_name);

class ros_ctrler : public process_ctrler {
    using process_handle_t = Poco::ProcessHandle;
    using pipe_input_stream_t = Poco::PipeInputStream;

public:
    ros_ctrler(const std::string &name, const std::string &xml_str);
    virtual ~ros_ctrler();

    virtual bool start() override;
    virtual bool stop() override;

    virtual bool is_running() override;
    virtual int  wait_for_end() override; // will block process until end

    bool set_args(const std::map<std::string, std::string> &args,
                std::vector<std::string> &fail_args);

private:
    std::shared_ptr<process_handle_t> ph_{nullptr};
    std::shared_ptr<pipe_input_stream_t> out_s_{nullptr};
    std::shared_ptr<pipe_input_stream_t> err_s_{nullptr};

    std::unique_ptr<std::thread> out_thr_{nullptr};
    std::unique_ptr<std::thread> err_thr_{nullptr};

    std::atomic_bool is_loop_ {false};
};
    
} // namespace process
} // namespace comwise

#endif // __COMWISE_PROCESS__ROS_CTRLER__H__
