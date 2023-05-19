#ifndef __LOG_XLOG_PRIVATE_H_
#define __LOG_XLOG_PRIVATE_H_

#include <thread>
#include <sstream>
#include <atomic>
#include <regex>
#include <mutex>
#include "time.h"
#include "xlog_var.h"

namespace comwise {
namespace log {

class xlog_private
{
public:
    xlog_private();
    ~xlog_private();

    //! set param
    //! set log directory
    void set_directory(
        const char* path);
    //! set log prefix name
    void set_prefix_name(
        const char* prefix_name);
    //! set log head field type, see xlog_field_t
    void set_field_type(
        uint32_t field_type);
    //! set log reserve_days days>=0
    void set_reserve_days(
        uint32_t days);

    //! write log by ascii format
    void print(
        enum xlog_level_t level,
        const char* filename,
        uint32_t line,
        const char* function,
        const systime_t& system_time,
        uint32_t thread_id,
        const char* data,
        int size = -1);
    //! write log by hex format
    void hexdump(
        enum xlog_level_t level,
        const char* filename,
        uint32_t line,
        const char* function,
        const void* data,
        int size);

private:
    void write(
        enum xlog_level_t level,
        const char* message,
        int size,
        const systime_t& system_time);
    void write_to_screen(
        const char *message,
        int size);
    void wirte_to_file(
        const char* message,
        const systime_t& system_time);
    void clear_log_file(
        uint32_t reserve_day);

    int format_line_head(
        char* buffer,
        int size,
        enum xlog_level_t level,
        const char* filename,
        uint32_t line,
        const char* function,
        const systime_t& system_time,
        uint32_t thread_id);
    int sprintf_filename_suffix(
        char* buffer,
        int buffer_size,
        const systime_t& system_time);
    int sprintf_timestamp(
        char* buffer,
        int buffer_size,
        const systime_t& system_time);
    int sprintf_thread_id(
        char* buffer,
        int buffer_size,
        uint32_t thread_id);
    int sprintf_file_and_line(
        char* buffer,
        int buffer_size,
        const char* filename,
        uint32_t line);
    int sprintf_level(
        char* buffer,
        int buffer_size,
        enum xlog_level_t level);
    int sprintf_function(
        char* buffer,
        int buffer_size,
        const char* function);
    int sprintf_head_ending(char *buffer, int buffer_size);

    const char* trim_filename(
        const char* filename);
    std::string get_log_filename(
        const char* suffix);
    std::string get_log_filename(
        time_t log_time);

private:
    static const int kLogFilenameSuffixSize = 16;
    static const int kMaxBufferSize = 2048;
    static const int kLogFatalErrorCode = 0xC0000001;

private:
    std::mutex lock_;

    uint32_t fields_ = xlog_field_t::FieldMask;

    uint32_t reserve_days_ = 30;

    uint32_t filename_depth_ = 1;

#if _WIN32
    std::string log_directory_ = ".\\xlog";
#else
    std::string log_directory_ = "./xlog";
#endif

    std::string prefix_name_ = "N_A";
    char old_suffix_[kLogFilenameSuffixSize];
    char new_suffix_[kLogFilenameSuffixSize];

    FILE* file_ = NULL;
};

} // namespace log
} // namespace comwise

#endif
