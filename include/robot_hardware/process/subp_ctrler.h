#ifndef __COMWISE_PROCESS__SUBP_CTRLER__H__
#define __COMWISE_PROCESS__SUBP_CTRLER__H__

#include <memory>
#include <atomic>
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

namespace comwise {
namespace process {

class subp_ctrler : public process_ctrler {
public:
    subp_ctrler(const std::string &name, const std::string &cmd);
    virtual ~subp_ctrler();

    void set_args(const std::vector<std::string> &args) { args_ = args; }
    std::vector<std::string> get_args() const { return args_; } 

    virtual bool start() override;
    virtual bool stop() override;
    
    virtual bool is_running() override;
    virtual int  wait_for_end() override; // will block process until end
    virtual bool close() override;
    virtual bool kill(int sig = 15) override;

protected:
    std::vector<std::string> args_;
    std::shared_ptr<Poco::ProcessHandle> ph_{nullptr};
    std::shared_ptr<Poco::PipeInputStream> out_s_{nullptr};
    std::shared_ptr<Poco::PipeInputStream> err_s_{nullptr};

    std::unique_ptr<std::thread> out_thr_{nullptr};
    std::unique_ptr<std::thread> err_thr_{nullptr};

    std::atomic_bool use_std_out_err_{false};
    std::atomic_bool is_loop_{false};
};

class once_subp_ctrler : public subp_ctrler {
public:
    once_subp_ctrler(const std::string &name, const std::string &cmd, bool use_std_out_err = false);
    virtual ~once_subp_ctrler() = default;

    bool start() override;

    void get_output_lines(const std::vector<std::string> &args) { output_lines_ = args; }
    const std::vector<std::string> &get_output_lines() { return output_lines_; };

private:
    std::vector<std::string> output_lines_;
};

} // namespace process
} // namespace comwise

#endif //__COMWISE_PROCESS__SUBP_CTRLER__H__
