#ifndef __COMWISE_PROCESS__PROCESS_CTRLER__H__
#define __COMWISE_PROCESS__PROCESS_CTRLER__H__

#include <memory>
#include <string>

namespace comwise {
namespace process {

class process_ctrler {
public:
    process_ctrler(const std::string &name, const std::string &arg)
        : name_(name), arg_(arg) { }
    virtual ~process_ctrler() { }

    virtual void set_name(const std::string &name) { name_ = name; }
    virtual std::string get_name() const { return name_; }

    virtual void set_arg(const std::string &arg) { arg_ = arg; }
    virtual std::string get_arg() const { return arg_; }

    virtual bool start() { return false; }
    virtual bool stop() { return false; }

    virtual bool is_running() { return false; }
    virtual int  wait_for_end() { return 0; }

    virtual bool close()  { return false; }
    virtual bool kill(int sig = 15) { return false; }

protected:
    std::string name_;
    std::string arg_;
};

} // namespace process
} // namespace comwise

#endif // __COMWISE_PROCESS__PROCESS_CTRLER__H__
