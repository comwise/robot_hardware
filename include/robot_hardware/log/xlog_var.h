/**
* @Copyright 2022
* @file xlog_var.h
* @brief log const var
* @author lichanglin
* @version v1.0 2018/11/02
*/

#ifndef __LOG_XLOG_VAR_H_
#define __LOG_XLOG_VAR_H_

#include <cstdint>
#include <string>
#include "colors.h"
#include "common/time.h"

namespace comwise {
namespace log {

    using systime_t = common::systime_t;
    using namespace common;

    /**
    * xlog level, see the following detail comment.
    */
    enum xlog_level_t {
        Trace = 0, // trace level, write to debugout
        Debug,     // debug level, write to debugout and log file
        Info,      // info level, write to stdout and log file
        Warning,   // warning level, write to stdout and log file
        Error,     // error level, write to stdout and log file
        Fatal,     // fatal level, write to system file
    };

    /**
    * xlog head tag, see the following detail comment.
    */
    enum xlog_field_t {
        None = 0,
        Timestamp = 1 << 0,  // time stamp
        ProcessId = 1 << 1,  // process id
        ThreadId = 1 << 2,   // thread id
        FileLine = 1 << 3,   // file line
        Function = 1 << 4,   // function
        FieldMask = Timestamp | ThreadId | FileLine | Function,
        MinFieldMask = Timestamp | FileLine | Function,
        MaxFieldMask = MinFieldMask | ProcessId | ThreadId
    };

    /**
    * xlog print head content, see the following detail comment.
    */
    struct xlog_head_t {
        int64_t  time;
        const char* filename; // filename
        int line;  // line
        const char* function; // function
        const char* category;  // module
        int thread_id; // thread id
    };

    /**
    * xlog print log message, see the following detail comment.
    */
    struct xlog_entry_t {
        xlog_level_t level; // log level
        xlog_head_t context; // log head
        std::string message; // log body
    };

} // namespace log
} // namespace comwise

#endif
