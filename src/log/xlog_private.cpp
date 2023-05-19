#include "log/xlog_private.h"
//#include "common/debug.h"
//#include "os.h"

namespace comwise {
namespace log {

static const char kHeaderTimestamp[] = "%02u:%02u:%02u.%03u ";
static const char kHeaderThreadId[] = "%04x ";
static const char kHeaderFileAndLine[] = "%s:%d ";
static const char kHeaderFunction[] = "%s ";
static const char kHeaderLogLevel[] = "%s ";
static const char kLogFilenameSuffix[] = "%04u%02u%02u.log";
static const char* kLevelsStrings[] = { "Trace", "Debug", "Info", "Warning", "Error", "Fatal" };


xlog_private::xlog_private()
{

}

xlog_private::~xlog_private()
{

}

void xlog_private::set_directory(const char* path)
{
    #if 0
    log_directory_.assign(path);
    if (*log_directory_.rbegin() != POSIX_DIR_SYMBOL) {
        log_directory_.push_back(POSIX_DIR_SYMBOL);
    }
    utils::mkdir<char>(log_directory_.c_str());
    #endif
}

void xlog_private::set_prefix_name(const char* prefix_name)
{
    prefix_name_ = prefix_name;
}

void xlog_private::set_field_type(uint32_t field)
{
    fields_ = field;
}

void xlog_private::set_reserve_days(uint32_t days)
{
    reserve_days_ = days;
}

void xlog_private::print
(
    enum xlog_level_t level,
    const char* filename,
    uint32_t line,
    const char* function,
    const systime_t& system_time,
    uint32_t thread_id,
    const char* data,
    int size
)
{
    int body_pos = 0;

    // init log head
    char buffer[kMaxBufferSize + 2] = {0}; // '\n\0'
    int pos = format_line_head(buffer, kMaxBufferSize, level, filename,
        line, function, system_time, thread_id);
    body_pos = pos;

    // format log data
    pos += snprintf(buffer + pos, kMaxBufferSize - pos, "%s", data);
    //pos += posix_vsnprintf(buffer + pos, kMaxBufferSize - pos, "%s", (void*)data);
    if (pos > kMaxBufferSize) pos = kMaxBufferSize - 1;

    // fatal error, then write to syslog
    if (level >= xlog_level_t::Fatal) {
        buffer[pos] = 0;
        //attempt_syslog(std::string(&buffer[body_pos]), kLogFatalErrorCode);
        return;
    }

    // add tail endl char
    buffer[pos++] = '\n';
    buffer[pos++] = 0;

    // print log 
    std::lock_guard<std::mutex> guard(lock_);
    switch (level) {
    case Trace:
        //write_debug_output(buffer);
        return;
    case Debug:
        //write_debug_output(buffer);
        break;
    case Info:
    case Warning:
    case Error:
    case Fatal:
        write_to_screen(buffer, pos);
        break;
    default:
        break;
    }
    wirte_to_file(buffer, system_time);
}

void xlog_private::hexdump
(
    enum xlog_level_t level,
    const char* filename,
    uint32_t line,
    const char* function,
    const void* data,
    int size
)
{
    if (size <= 0) return;

    // get current time
    systime_t system_time = get_local_time();

    // int log head
    std::vector<char> buffer(size * 3 + 2);
    uint32_t pos = format_line_head(buffer.data(), (int)buffer.size(), level, filename, line, function, system_time, 0/*gettid()*/);
    buffer[pos++] = '\n';

    // format hex data
    for (int i = 0; i < size; i++) {
        if (pos + 3 > buffer.size()) break;
        pos += snprintf(buffer.data() + pos, buffer.size() - pos, "%02x",
            *((unsigned char*)data + i));
        if (((i + 1) % 32) == 0 && pos < buffer.size()) {
            buffer[pos++] = '\n';
            continue;
        }
        if (((i + 1) % 4) == 0 && pos < buffer.size()) {
            buffer[pos++] = ' ';
        }
        if (((i + 1) % 16) == 0 && pos < buffer.size()) {
            buffer[pos++] = ' ';
        }
    }

    if (buffer[pos - 1] != '\n')
        buffer[pos++] = '\n';
    buffer[pos++] = 0;

    // print log 
    std::lock_guard<std::mutex> guard(lock_);

    switch (level) {
    case Trace:
        //write_debug_output(buffer.data());
        return;
    case Debug:
        //write_debug_output(buffer.data());
        break;
    case Info:
    case Warning:
    case Error:
    case Fatal:
        write_to_screen(buffer.data(), pos);
        break;
    }
    wirte_to_file(buffer.data(), system_time);
}

void xlog_private::write(enum xlog_level_t level,
    const char* message,
    int size,
    const systime_t& system_time)
{
    std::lock_guard<std::mutex> guard(lock_);

    write_to_screen(message, size);
    wirte_to_file(message, system_time);
}

void xlog_private::write_to_screen(const char *message, int size)
{
    fwrite(message, size - 1, 1, stderr);
}

void xlog_private::wirte_to_file(const char* message, const systime_t& system_time)
{
    sprintf_filename_suffix(new_suffix_, sizeof(new_suffix_), system_time);
    if (strcmp(old_suffix_, new_suffix_) != 0) {
        if (file_ != NULL) fclose(file_);
        clear_log_file(reserve_days_);
        std::string path = get_log_filename(new_suffix_);
#ifdef _WIN32
        file_ = _fsopen(path.c_str(), "a+", _SH_DENYNO);
#else
        file_ = fopen(path.c_str(), "a+");
#endif
        if (file_) {
            memcpy(old_suffix_, new_suffix_, sizeof(old_suffix_));
        }
    }
    if (file_ != NULL) {
        fprintf(file_, "%s", message);
        fflush(file_);
    }
}

void xlog_private::clear_log_file(uint32_t reserve_day)
{
    time_t seconds = time(NULL) - reserve_day * 86400;
    for (seconds -= 24 * 3600;; seconds -= 24 * 3600) {
        std::string filename = get_log_filename(seconds);
        if (::remove(filename.c_str()) != 0) {
            break;
        }
    }
}

int xlog_private::format_line_head
(
    char* buffer,
    int size,
    enum xlog_level_t level,
    const char* filename,
    uint32_t line,
    const char* function,
    const systime_t& system_time,
    uint32_t thread_id
)
{
    int pos = 0;

    //time
    pos += sprintf_timestamp(buffer + pos, size - pos, system_time);
    if (pos >= size) return size;

    // thread id
    pos += sprintf_thread_id(buffer + pos, size - pos, thread_id);
    if (pos >= size) return size;

    // level
    pos += sprintf_level(buffer + pos, size - pos, level);
    if (pos >= size) return size;

    // file and line
    pos += sprintf_file_and_line(buffer + pos, size - pos, filename, line);
    if (pos >= size) return size;

    // print message
    pos += sprintf_function(buffer + pos, size - pos, function);
    if (pos >= size) return size;

    // end message
    pos += sprintf_head_ending(buffer + pos, size - pos);
    if (pos >= size) return size;

    return pos;
}

int xlog_private::sprintf_filename_suffix (char* buffer, int buffer_size, const systime_t& system_time)
{
    return snprintf(buffer, buffer_size, kLogFilenameSuffix,
        system_time.year, system_time.month, system_time.day);
}

int xlog_private::sprintf_timestamp(char* buffer, int buffer_size, const systime_t& system_time)
{
    if (fields_ & Timestamp) {
        return snprintf((char*)buffer, buffer_size, kHeaderTimestamp,
            system_time.hour, system_time.minute, system_time.second, system_time.milli_second);
    }
    return 0;
}

int xlog_private::sprintf_thread_id(char* buffer, int buffer_size, uint32_t thread_id)
{
    if (fields_ & ThreadId) {
        return snprintf(buffer, buffer_size, kHeaderThreadId, thread_id);
    }
    return 0;
}

int xlog_private::sprintf_file_and_line(char* buffer, int buffer_size, const char* filename, uint32_t line)
{
    if (fields_ & FileLine) {
        return snprintf(buffer, buffer_size, kHeaderFileAndLine, trim_filename(filename), line);
    }
    return 0;
}

int xlog_private::sprintf_level(char* buffer, int buffer_size, enum xlog_level_t level)
{
    return snprintf(buffer, buffer_size, kHeaderLogLevel, kLevelsStrings[level]);
}

int xlog_private::sprintf_function(char* buffer, int buffer_size, const char* function)
{
    if (fields_ & FileLine) {
        return snprintf(buffer, buffer_size, kHeaderFunction, function);
    }
    return 0;
}

int xlog_private::sprintf_head_ending(char *buffer, int buffer_size)
{
    return snprintf(buffer, buffer_size, "#: ");
}

const char* xlog_private::trim_filename(const char* filename)
{
    const char* r = filename + strlen(filename) - 1;
    int slash_count = 0;
    while (r-- != filename) {
        //if (*r == POSIX_DIR_SYMBOL) slash_count++;
        if (slash_count == filename_depth_ + 1) return ++r;
    }
    return filename;
}

std::string xlog_private::get_log_filename(const char* suffix)
{
    return log_directory_ + prefix_name_ + suffix;
}

std::string xlog_private::get_log_filename(time_t seconds)
{
    systime_t system_time = get_local_time(seconds);
    char suffix[kLogFilenameSuffixSize] = { };
    sprintf_filename_suffix(suffix, sizeof(suffix), system_time);
    return get_log_filename(suffix);
}

} // namespace log
} // namespace comwise