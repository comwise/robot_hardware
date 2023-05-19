#include "log/stdout_layout.h"
#include <iostream>

namespace comwise {
namespace log {

static const char kHeaderTimestamp[] = "[%04u-%02u-%02u %02u:%02u:%02u.%03u]";
static const char kHeaderThreadId[] = "%04x ";
static const char kHeaderFileAndLine[] = "%s:%d ";
static const char kHeaderFunction[] = "%s ";
static const char kHeaderLogLevel[] = "%s ";
static const char* kLevelsStrings[] = { "Trace", "Debug", "Info", "Warning", "Error", "Fatal" };

void stdout_layout::consume(const xlog_entry_t& entry)
{
   print_header(entry);
   print_context(entry);
}

void stdout_layout::print_header(const xlog_entry_t& entry) const
{
    // time
    char buffer[32] = { };
    common::systime_t t = common::get_local_time(entry.context.time);
    snprintf((char*)buffer, sizeof(buffer) / sizeof(buffer[0]), kHeaderTimestamp,
        t.year, t.month, t.day, t.hour, t.minute, t.second, t.milli_second);
    std::cout << C_WHITE << buffer;

    // module
    std::cout << C_B_WHITE << " [" << entry.context.category << "]";

    // level
    switch (entry.level)
    {
    case xlog_level_t::Debug:
        std::cout << C_B_BLUE << " [Debug]";
        break;
    case xlog_level_t::Info:
        std::cout << C_B_GREEN << " [Info]";
        break;
    case xlog_level_t::Warning:
        std::cout << C_B_YELLOW << " [Warning]";
        break;
    case xlog_level_t::Error:
        std::cout << C_B_RED << " [Error]";
        break;
    default:
        break;
    }
}

void stdout_layout::print_context(const xlog_entry_t& entry) const
{
    // body
   std::cout << C_WHITE << " " << entry.message;

   // tail
   std::cout << C_B_BLUE << " [";
   if (entry.context.filename) {
      std::cout << entry.context.filename << ":" << entry.context.line;
   }
   if (entry.context.function) {
       std::cout << " #" << C_CYAN << entry.context.function;
   }
   std::cout << C_B_BLUE << "]";

   // default
   std::cout << C_DEF << std::endl;
}

} // namespace log
} // namespace comwise

