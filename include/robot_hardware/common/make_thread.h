#ifndef __COMMON__MAKE_THREAD__H__
#define __COMMON__MAKE_THREAD__H__

#include <memory>
#include <string>
#include <thread>
#if _WIN32
#include <windows.h>
#else
#include <sys/prctl.h>
#include <pthread.h>
#endif
#include <errno.h>
#include "log/log.h"


namespace common {
inline int set_thread_name(const std::string &thread_name) {
    int ret = 0;

#if _WIN32
    const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push,8)
    typedef struct tagTHREADNAME_INFO {
        DWORD dwType; // Must be 0x1000.
        LPCSTR szName; // Pointer to name (in user addr space).
        DWORD dwThreadID; // Thread ID (-1=caller thread).
        DWORD dwFlags; // Reserved for future use, must be zero.
    } THREADNAME_INFO;
#pragma pack(pop)

    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = thread_name.c_str();
    info.dwThreadID = std::this_thread::get_id();
    info.dwFlags = 0;

    __try {
        RaiseException(MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info);
    } __except(EXCEPTION_CONTINUE_EXECUTION) {
        // nothing
    }
#else
    std::string fmt_name = std::move(thread_name.substr(0, 15));
    pthread_t thread = pthread_self();
    if (thread == -1) {
        prctl(PR_SET_NAME, fmt_name.c_str());
    } else {
        ret = pthread_setname_np(thread, fmt_name.c_str());
    }
#endif

    return ret;
}

// ATTENTION 1: must not code SSPD_LOG in the body, really will hang the program.
// ATTENTION 2: should rename the thread in self body, not parent rename it, or some complicated permission issues.
template <class Fn, class... Args>
std::unique_ptr<std::thread> make_thread(const std::string &name, Fn&& fn, Args&&... args) {
    auto thr = std::unique_ptr<std::thread>(new std::thread([=](Fn&& fn, Args&&... args) {
        try {
            int rtn = set_thread_name(name);
            if(rtn != 0) {
                LOG_STREAM_WARN << "set thread name(" << name << ") fail, detail: (" << rtn << "): " << std::strerror(rtn);
            }
            auto func = std::bind(fn, args...);
            func();
        } catch(const std::exception &e) {
            LOG_STREAM_ERROR << "create thread(" << name << ") crashed, " << e.what();
        }
    }, std::forward<Fn>(fn), std::forward<Args>(args)...));
    return std::move(thr);
};

} // common

#endif //__COMMON__MAKE_THREAD__H__
