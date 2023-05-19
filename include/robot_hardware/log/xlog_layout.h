#ifndef __LOG_XLOG_LAYOUT_H_
#define __LOG_XLOG_LAYOUT_H_

#include <cstdint>
#include "xlog_var.h"

namespace comwise {
namespace log {

/**
 * Consumes a log entry to output it somewhere.
 */
class log_layout
{
public:
    virtual ~log_layout() {};
    virtual void consume(const xlog_entry_t&) = 0;
};

} // namespace log
} // namespace comwise

#endif
