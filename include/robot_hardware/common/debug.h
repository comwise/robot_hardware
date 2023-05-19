#ifndef __COMMON__DEBUG__H__
#define __COMMON__DEBUG__H__

#include <string>
#include <cstring>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <sys/syslog.h>
#endif

namespace common {

#ifdef _WIN32
template<>
int attempt_syslog(const std::basic_string<char> &msg, uint32_t err) {
    if (msg.size() == 0) {
        return -1;
    }

    HANDLE shlog = RegisterEventSourceA(NULL, "Application");
    if (INVALID_HANDLE_VALUE == shlog) {
        return -1;
    }

    const char *strerrs[1];
    strerrs[0] = msg.data();
    BOOL b = ReportEventA(shlog, EVENTLOG_ERROR_TYPE, 0, err, NULL,
        1, 0, strerrs, NULL);

    DeregisterEventSource(shlog);
    return ((b) ? (0) : (-1));
}

template<>
int attempt_syslog(const std::basic_string<wchar_t> &msg, uint32_t err) {
    if (msg.size() == 0) {
        return -1;
    }

    HANDLE shlog = RegisterEventSourceW(NULL, L"Application");
    if (INVALID_HANDLE_VALUE == shlog) {
        return -1;
    }

    const wchar_t *strerrs[1];
    strerrs[0] = msg.data();
    BOOL b = ReportEventW(shlog, EVENTLOG_ERROR_TYPE, 0, err, NULL,
        1, 0, strerrs, NULL);

    DeregisterEventSource(shlog);
    return ((b) ? (0) : (-1));
}

#else

// windows: save to "HKLM\SYSTEM\CurrentControlSet\Services\EventLog\Application"
// centos7: call logger -i(process id)t [app], then write log to "/var/log/messages"
// centos7: call syslog, write to "/var/log/messges"
// NSP：system log write significance error info, no repeat, no frequently, no debug
// POSIX: do not support UNICODE
template<class T>
int attempt_syslog(const std::basic_string<T> &msg, uint32_t err);

    /*
    * cat /var/log/messages | tail -n1
    */
template<>
int attempt_syslog(const std::basic_string<char> &msg, uint32_t err) {
    if (0xC0000001 != err) {
        syslog(LOG_USER | LOG_ERR, "(%d)%s # event:0x%08X", getpid(), msg.c_str(), err);
    }
    else {
        syslog(LOG_USER | LOG_ERR, "(%d)%s", getpid(), msg.c_str());
    }

    return 0;
}

template<>
int attempt_syslog(const std::basic_string<wchar_t> &msg, uint32_t err) {
    if (0xC0000001 != err) {
        syslog(LOG_USER | LOG_ERR, "(%d)%ls # event:0x%04X", getpid(), msg.c_str(), err);
    }
    else {
        syslog(LOG_USER | LOG_ERR, "(%d)%ls", getpid(), msg.c_str());
    }
    return 0;
}
#endif

void write_debug_output(const char *str) {
#if _WIN32
    OutputDebugStringA(str);
#else
    fwrite(str, strlen(str), 1, stderr);
#endif
}

}
#endif // __COMMON__DEBUG__H__