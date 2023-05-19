/******************************************************************************************
// how to deal with signal in Multi-threading and get performance bottleneck with gperftools
!!!ATTENTION!!! Should be very careful about signals setting and dealing,
                affecting sub thread, sub process, etc.
*******************************************************************************************/
#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include <atomic>
#include "common/make_thread.h"

// User defined real-time signals
#define SIG_WAKE_SYS_CALL_BLOCK     (SIGRTMIN+3)    // signal number should be: 37
#define SIG_EXCEPTION_ABORT         (SIGRTMIN+1)    // signal number should be: 35
#define SIG_PROFILE_START           (SIGRTMAX-4)    // signal number should be: 60
#define SIG_PROFILE_STOP            (SIGRTMAX-3)    // siganl number should be: 61

namespace common {

class MainSignalProcess
{
public:
    static std::shared_ptr< MainSignalProcess > Instance(std::function<void (int sig)> stop_cb = nullptr) {
        static auto _instance = std::shared_ptr< MainSignalProcess >(new MainSignalProcess(stop_cb));
        if(stop_cb) _instance->stop_cb_ = stop_cb;
        return _instance;
    }
    std::atomic<int> term_signal;

private:
    MainSignalProcess(std::function<void (int sig)> stop_cb): stop_cb_(stop_cb), term_signal(0) {
         InstallSignalHandle();
    }
    /*
    install signalHandle
    tip:
    pthread_sigmask(int how, const sigset_t *set, sigset_t *oset)
    1) how :SIG_BLOCK:结果集是当前集合参数集的并集；
       SIG_UNBLOCK:结果集是当前集合参数集的差集;
       SIG_SETMASK:结果集是由参数集指向的集
    2) notes 每个线程均有自己的信号屏蔽集(掩码),使用pthread_sigmask函数来屏蔽某个线程对某些信号的
        的响应处理,仅留下需要处理该信号的线程来处理指定的信号.实现方式是:利用线程信号屏蔽集的继承关系
        (在主线程中对sigmask进行设置后,主进程创建出来的线程将继承主线程的掩码)
    */
    void InstallSignalHandle() {
        // 1. about kills and terms
        auto handler1 = [](int sig) {
            MainSignalProcess::Instance()->term_signal = sig;
        };
        std::signal(SIGINT, handler1);
        std::signal(SIGTERM, handler1);
        std::signal(SIGHUP, handler1);
        std::signal(SIG_EXCEPTION_ABORT, handler1);

        // 2. about wake up blocks
        auto handler2 = [](int sig) {
            // just do nothing
        };
        std::signal(SIG_WAKE_SYS_CALL_BLOCK, handler2);


        // 3. about special usage signals
        // TODO
        sigset_t bset, oset;
        sigemptyset(&bset);
        sigaddset(&bset, SIG_PROFILE_START);
        sigaddset(&bset, SIG_PROFILE_STOP);

        if (pthread_sigmask(SIG_BLOCK, &bset, &oset) != 0) {
            // impossible here, TODO: exit or logging.
        }

        // 4. start signal handler thread
        sig_running_ = true;
        sig_thr_ = common::make_thread("SIG_PROCESS", std::bind(&MainSignalProcess::_SignalProc, this));
        if(sig_thr_) {
            sig_thr_->detach();
        }
    }
    //void Wait()
    //{
    //sig_thr_.join();
    //}

private:
    void _SignalHandle(int sig)
    {
        if (SIGHUP == sig || SIGINT == sig || SIGTERM == sig || SIG_EXCEPTION_ABORT == sig) {
            if (stop_cb_) {
                stop_cb_(sig);
                kill(getpid(), SIG_WAKE_SYS_CALL_BLOCK);    // wake up possibly blocked system calls
            } else {
                std::abort();
            }
        }
    }
    // signal handle function
    void _SignalProc()
    {
        sigset_t waitset;
        siginfo_t info;
        int rc;
        sigemptyset(&waitset);
        sigaddset(&waitset, SIG_PROFILE_START);
        sigaddset(&waitset, SIG_PROFILE_STOP);
        while(sig_running_) {
            // 1. special siganl handling
            struct timespec tm = {0, 100000000};    // 100 milliseconds
            rc = sigtimedwait(&waitset, &info, &tm);
            if (rc > 0) {
                _SignalHandle(info.si_signo);
            } else if (0 == rc) {
                // timeout
            } else {
                // TODO impossible here, add some log
                //   signalRunning_ = false;
            }
            // 2. kill handling
            if(term_signal != 0) {
                _SignalHandle(term_signal);
                sig_running_ = false;
            }
        }
    }
    std::shared_ptr<std::thread> sig_thr_{nullptr};
    bool sig_running_{false};
    std::function<void(int sig)> stop_cb_{nullptr};
};

}   // namespace common