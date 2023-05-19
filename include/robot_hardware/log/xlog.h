#ifndef __LOG_XLOG_H_
#define __LOG_XLOG_H_

#include <cstdint>
#include <thread>
#include <iostream>
#include <sstream>
#include <atomic>
#include <regex>
#include <condition_variable>
#include "double_queue.h"
#include "xlog_var.h"
//#include "comwise/utils/os.h"

/**
 * log layer. Logging categories and Level can be specified dynamically at runtime. However, even on a category 
 * not covered by the current level, there is some overhead on calling a log macro. For maximum performance, you can 
 * opt out of logging any particular level by defining the following symbols:
 *
 * * #define LOG_NO_DEBUG
 * * #define LOG_NO_INFO
 * * #define LOG_NO_WARNING
 * * #define LOG_NO_ERROR
 *
 * Additionally. the lowest level (Debug) and other level(Trace/Fatal) is disabled by default on release branches.
 */


namespace comwise {
namespace log {

class log_layout;

/**
 * Logging utilities.
 * Logging is accessed through the four macros above, and configuration on the log output
 * can be achieved through static methods on the class. Logging at various levels can be 
 * disabled dynamically (through the Level level) or statically (through the LOG_NO_[VERB]
 * macros) for maximum performance.
 */
class xlog
{
public:

    //! Registers an user defined layout to route log output
   static void register_layout(std::unique_ptr<log_layout>);

   //! Set the message field to be logged, enable or disable tag in log entries
   static void set_field_type(uint32_t field_type);
   //! Get the message field to be logged, enable or disable tag in log entries
   static uint32_t get_field_type();

   //! Sets the verbosity level, allowing for messages equal or under that priority to be logged.
   /* Types of log entry.
   * * Trace: no use, useful for debugging
   * * Debug: Lowest priority. Useful for debugging. Disabled by default on release branches. no use
   * * Info: Low priority. Useful for debugging. Can be disabled statically(#define LOG_NO_INFO) and dynamically.
   * * Warning: Medium priority. Can be disabled statically(#define LOG_NO_WARNING) and dynamically.
   * * Error: High priority. Can only be disabled statically through #define LOG_NO_ERROR.
   * * Fatal: Highest priority, no use
   */
   static void set_level(xlog_level_t);
   //! Returns the current verbosity level.
   static xlog_level_t get_level();

   //! Sets a filter that will pattern-match against log categories, dropping any unmatched categories.
   static void set_category_filter    (const std::regex&);
   //! Sets a filter that will pattern-match against filenames, dropping any unmatched categories.
   static void set_filename_filter    (const std::regex&);
   //! Sets a filter that will pattern-match against the provided error string, dropping any unmatched categories.
   static void set_errorstring_filter (const std::regex&);

   //! Returns the logging engine to configuration defaults.
   static void reset();

   //! Stops the logging thread. It will re-launch on the next call to a successful log macro.
   static void kill_thread();
   // Note: In VS2013, if you're linking this class statically, you will have to call KillThread before leaving
   // main, due to an unsolved MSVC bug.

   /** 
    * Not recommended to call this method directly! Use the following macros:
    *  * logInfo(cat, msg);
    *  * logWarning(cat, msg);
    *  * logError(cat, msg);
    */
   static void queue_log(const std::string &message, const xlog_head_t &context, xlog_level_t level);

private:
    struct resources_t
    {
        // log buffer
        double_queue<xlog_entry_t> logs;

        // log print layout
        std::vector<std::unique_ptr<log_layout>> layouts;

        // log print thread
        std::unique_ptr<std::thread> logging_thread;
        // Condition variable segment.
        std::condition_variable cv;
        std::mutex cv_mtx;
        bool is_logging = false; // is logging
        bool is_work = false; // is working

        // log context configuration variable
        std::mutex config_mtx; // config locker
        uint32_t fields = xlog_field_t::FieldMask; // log head label
        uint32_t reserve_days = 30; // log file reserve days
        std::atomic<xlog_level_t> level; // log level
        std::unique_ptr<std::regex> category_filter; // category filter param
        std::unique_ptr<std::regex> filename_filter; // filename filter param
        std::unique_ptr<std::regex> errorstring_filter; // content filter param

        resources_t();
        ~resources_t();
    };

   static struct resources_t resources_;

private:
   // Applies transformations to the entries compliant with the options selected (such as
   // erasure of certain context information, or filtering by category. Returns false 
   // if the log entry is blacklisted.
   static bool preprocess(xlog_entry_t&);
   static void run();
};

#if defined ( WIN32 )
    #define __func__ __FUNCTION__
#endif

#define LOG_PRINT(cat, msg, level) { \
    if (level >= xlog::xlog::get_level()) { \
        std::stringstream ss; \
        int64_t time = common::get_now_time();\
        int tid = 0/*common::gettid()*/;\
        ss << msg; \
        xlog::xlog::queue_log(ss.str(), xlog_head_t{ time, __FILE__, __LINE__, __func__, cat, tid }, level);\
    }\
}\

#ifndef LOG_NO_ERROR
    #define logError_(cat, msg) { LOG_PRINT(cat, msg, xlog_level_t::Error) }
#else
    #define logError_(cat, msg)
#endif

#ifndef LOG_NO_WARNING
    #define logWarning_(cat, msg) { LOG_PRINT(cat, msg, xlog_level_t::Warning) }
#else
    #define logWarning_(cat, msg)
#endif

#ifndef LOG_NO_INFO
    #define logInfo_(cat, msg) { LOG_PRINT(cat, msg, xlog_level_t::Info) }
#else 
    #define logInfo_(cat, msg)
#endif

#if (defined(__INTERNALDEBUG) || defined(_INTERNALDEBUG)) && (defined(_DEBUG) || defined(__DEBUG)) && (!defined(LOG_NO_DEBUG))
    #define COMPOSITE_LOG_NO_DEBUG 1
#else
    #define COMPOSITE_LOG_NO_DEBUG 0
#endif

#if COMPOSITE_LOG_NO_DEBUG
    #define logDebug_(cat, msg) { LOG_PRINT(cat, msg, xlog_level_t::Debug) }
#else
    #define logDebug_(cat, msg)
#endif

// Logging API:
//! Logs an info message. Disable it through xlog::set_level, #define LOG_NO_DEBUG, or being in a release branch
#define logDebug(cat,msg)   logDebug_(cat,msg)
//! Logs an info message. Disable it through xlog::set_level or #define LOG_NO_INFO
#define logInfo(cat,msg)    logInfo_(cat,msg)
//! Logs a warning. Disable reporting through xlog::set_level or #define LOG_NO_WARNING
#define logWarning(cat,msg) logWarning_(cat,msg)
//! Logs an error. Disable reporting through #define LOG_NO_ERROR
#define logError(cat,msg)   logError_(cat,msg)

} // namespace log
} // namespace comwise

#endif
