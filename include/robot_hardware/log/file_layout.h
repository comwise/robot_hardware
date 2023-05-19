#ifndef __LOG_FILE_LAYOUT_H_
#define __LOG_FILE_LAYOUT_H_

#include <memory>
#include "xlog_layout.h"

namespace comwise {
namespace log {

class xlog_private;

class file_layout: public log_layout
{
public:
    file_layout();
    ~file_layout();

   virtual void consume(const xlog_entry_t&);

private:
    std::unique_ptr<xlog_private> impl_;
};

} // namespace log
} // namespace comwise

#endif //__LOG_FILE_LAYOUT_H_
