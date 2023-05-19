#include "log/file_layout.h"
#include <iostream>
#include "log/xlog_private.h"
#include "common/time.h"

namespace comwise {
namespace log {

file_layout::file_layout()
    : impl_(new xlog_private())
{

}

file_layout::~file_layout()
{

}

void file_layout::consume(const xlog_entry_t& entry)
{
    systime_t time = get_local_time(entry.context.time);
    if (impl_) {
        impl_->print(
            entry.level,
            entry.context.filename,
            entry.context.line,
            entry.context.function,
            time,
            entry.context.thread_id,
            entry.message.c_str()
        );
    }
}

} // namespace log
} // namespace comwise