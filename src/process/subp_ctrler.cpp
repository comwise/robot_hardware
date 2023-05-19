#include "process/subp_ctrler.h"
#include <Poco/Process.h>
#include <Poco/PipeStream.h>
#include <Poco/UUIDGenerator.h>
#include "log/log.h"
#include "common/make_thread.h"
#include "common/signal.hpp"
#include <unistd.h>

namespace comwise {
namespace process {

subp_ctrler::subp_ctrler(const std::string &name, const std::string &cmd)
 : process_ctrler(name, cmd)
{
    
}

subp_ctrler::~subp_ctrler()
{
    wait_for_end();
}

bool subp_ctrler::start()
{
    Poco::Process::Args args;
    args.push_back("-oL");
    args.push_back("-eL");
    args.push_back(arg_);
    for(auto &a : args_){
        args.push_back(a);
    }

    Poco::Pipe inPipe, errPipe;
    Poco::PipeOutputStream ostr(inPipe);

    ph_ = std::make_shared< Poco::ProcessHandle >(
        Poco::Process::launch("/usr/bin/stdbuf", args, &inPipe, nullptr, &errPipe));

    err_s_ = std::make_shared< Poco::PipeInputStream >(errPipe);

    ostr.close();

    std::signal(SIG_WAKE_SYS_CALL_BLOCK, [](int sig){});

    LOG_STREAM_INFO << "[subp] start to start sub process(" << name_ << ") ...";

    err_thr_ = std::move(common::make_thread(std::string("Serr_") + name_, [&, this]() {
        is_loop_ = true;
        while(is_loop_ && err_s_ && err_s_->good()) {
            std::string info;
            if(std::getline(*err_s_, info)) {
                if (use_std_out_err_)
                    std::cerr << info << std::endl;
                else
                    LOG_STREAM_ERROR << "[subp:" << name_ << "] " << info;
            }
        }
    }));

    return is_running();
}

bool subp_ctrler::stop()
{
    try {
        if(ph_ && is_running())
            Poco::Process::requestTermination(ph_->id());
    } catch (const Poco::NotFoundException &e) {
        LOG_STREAM_WARN << "[subp] termination subprocess(" << name_ << ") excepiton, " << e.what();
    }
    return true;
}

bool subp_ctrler::is_running()
{
    if(ph_) {
        return ::kill(ph_->id(), 0) == 0;
    }
    return false;
}

int subp_ctrler::wait_for_end()
{
    int ret = 0;
    if(this->is_running())
        ret = Poco::Process::wait(*ph_);
    close();
    return ret;
}

bool subp_ctrler::close()
{
    if(nullptr == ph_)
        return true;
    
    is_loop_ = false;

    if(err_s_) {
        err_s_->close();
    }
    err_s_ = nullptr;

    if(err_thr_) {
        pthread_kill(err_thr_->native_handle(), SIG_WAKE_SYS_CALL_BLOCK);
    }

    if(err_thr_ && err_thr_->joinable())
        err_thr_->join();
    err_thr_ = nullptr;

    ph_ = nullptr;

    return true;
}

bool subp_ctrler::kill(int sig)
{
    return ph_? ::kill(ph_->id(), sig) == 0 : false;
}

once_subp_ctrler::once_subp_ctrler(
    const std::string &name,
    const std::string &cmd,
    bool use_std_out_err)
    : subp_ctrler(name, cmd)
{
    use_std_out_err_ = use_std_out_err;
}

bool once_subp_ctrler::start()
{
    Poco::Process::Args args;
    args.push_back("-o0");
    args.push_back("-e0");
    args.push_back(arg_);
    for(auto &a : args_){
        args.push_back(a);
    }

    Poco::Pipe inPipe, errPipe;
    Poco::PipeOutputStream ostr(inPipe);

    ph_ = std::make_shared< Poco::ProcessHandle >(
        Poco::Process::launch("/usr/bin/stdbuf", args, &inPipe, nullptr, &errPipe));

    err_s_ = std::make_shared< Poco::PipeInputStream >(errPipe);

    ostr.close();
    output_lines_.clear();
    std::signal(SIG_WAKE_SYS_CALL_BLOCK, [](int sig){});

    err_thr_ = std::move(common::make_thread(std::string("Oerr_") + name_, [&, this]() {
        is_loop_ = true;
        while(is_loop_ && err_s_ && err_s_->good()) {
            std::string info;
            if(std::getline(*err_s_, info)) {
                if (use_std_out_err_)
                    std::cerr << info << std::endl;
                else
                    LOG_STREAM_ERROR << "[subp:" << name_ << "] " << info;
            }
        }
    }));

    return this->is_running();
}

} // namespace process
} // namespace comwise