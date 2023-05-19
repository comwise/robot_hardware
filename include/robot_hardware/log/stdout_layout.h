#ifndef __LOG_STDOUT_LAYOUT_H_
#define __LOG_STDOUT_LAYOUT_H_

#include "xlog_layout.h"

namespace comwise {
namespace log {

class stdout_layout: public log_layout
{
public:
   virtual void consume(const xlog_entry_t&);

private:
   void print_header(const xlog_entry_t&) const;
   void print_context(const xlog_entry_t&) const;
};

} // namespace log
} // namespace comwise

#endif
