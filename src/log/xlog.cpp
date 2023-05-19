#include "log/xlog.h"
#include "log/xlog_layout.h"

namespace comwise {
namespace log {

using namespace std;

struct xlog::resources_t xlog::resources_;

xlog::resources_t::resources_t():
   is_logging(false),
   is_work(false),
   level(xlog_level_t::Info)
{
}

xlog::resources_t::~resources_t()
{
   xlog::kill_thread();
}

void xlog::set_field_type(uint32_t field_type)
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    resources_.fields &= field_type;
}

uint32_t xlog::get_field_type()
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    return resources_.fields;
}

xlog_level_t xlog::get_level()
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    return resources_.level;
}

void xlog::set_level(xlog_level_t level)
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    resources_.level = level;
}

void xlog::set_category_filter(const std::regex& filter)
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    resources_.category_filter.reset(new std::regex(filter));
}

void xlog::set_filename_filter(const std::regex& filter)
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    resources_.filename_filter.reset(new std::regex(filter));
}

void xlog::set_errorstring_filter(const std::regex& filter)
{
    std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
    resources_.errorstring_filter.reset(new std::regex(filter));
}

void xlog::register_layout(std::unique_ptr<log_layout> consumer) 
{
   std::unique_lock<std::mutex> guard(resources_.config_mtx);
   resources_.layouts.emplace_back(std::move(consumer));
}

void xlog::reset()
{
   std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
   resources_.category_filter.reset();
   resources_.filename_filter.reset();
   resources_.errorstring_filter.reset();
   resources_.fields = xlog_field_t::FieldMask;
   resources_.level = xlog_level_t::Info;
   resources_.layouts.clear();
}

void xlog::run()
{
   std::unique_lock<std::mutex> guard(resources_.cv_mtx);
   while (resources_.is_logging)
   {
      while (resources_.is_work)
      {
         resources_.is_work = false;
         guard.unlock();

         resources_.logs.swap();
         while (!resources_.logs.empty())
         {
            std::unique_lock<std::mutex> config_lock(resources_.config_mtx);
            if (preprocess(resources_.logs.front()))
            {
               for (auto& consumer: resources_.layouts)
                  consumer->consume(resources_.logs.front());
            }
            resources_.logs.pop();
         }

         guard.lock();
      }
      if (resources_.is_logging)
         resources_.cv.wait(guard);
   }
}

bool xlog::preprocess(xlog_entry_t& entry)
{
   if (resources_.category_filter && !regex_search(entry.context.category, *resources_.category_filter))
      return false;
   if (resources_.filename_filter && !regex_search(entry.context.filename, *resources_.filename_filter))
      return false;
   if (resources_.errorstring_filter && !regex_search(entry.message, *resources_.errorstring_filter))
      return false;
   if (!(resources_.fields & xlog_field_t::FileLine))
      entry.context.filename = nullptr;
   if (!(resources_.fields & xlog_field_t::Function))
      entry.context.function = nullptr;

   return true;
}

void xlog::kill_thread()
{
   {
      std::unique_lock<std::mutex> guard(resources_.cv_mtx);
      resources_.is_logging = false;
      resources_.is_work = false;
   }
   if (resources_.logging_thread)
   {
      resources_.cv.notify_all();
      if (resources_.logging_thread->joinable())
      {
          resources_.logging_thread->join();
      }
      //resources_.logging_thread.reset();
   }
}

void xlog::queue_log(const std::string &message, const xlog_head_t&context, xlog_level_t level)
{
   {
      std::unique_lock<std::mutex> guard(resources_.cv_mtx);
      if (!resources_.is_logging && !resources_.logging_thread)
      {
         resources_.is_logging = true;
         resources_.logging_thread.reset(new thread(xlog::run));
      }
   }

   resources_.logs.push(xlog_entry_t{level, context, message});
   {
      std::unique_lock<std::mutex> guard(resources_.cv_mtx);
      resources_.is_work = true;
   }
   resources_.cv.notify_all();
}

} //namespace log
} //namespace comwise