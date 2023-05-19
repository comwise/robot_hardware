#ifndef __COMWISE_LOG__SPD_LOGGER__H__
#define __COMWISE_LOG__SPD_LOGGER__H__

#include <memory>
#include <atomic>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/file_sinks.h"
#include "common/singleton.h"
#include "common/linux_chcwd.h"
#include "common/format.h"

namespace comwise {
namespace log {

static const std::string kLoggerName = "robot_hardware";
static const std::string kLoggerPath = "/opt/comwise/log/robot_hardware"; 

class spd_logger : public singleton<spd_logger>
{
public:
    bool create(const std::string &log_path) {
        if (init_step_ >= 1 ) {
            return init_step_ = 1? false : true;
        }

        try {
            init_step_ = 1;
            constexpr std::size_t kLogMaxSize = 1024 * 1024 * 8; // 8MB
            constexpr std::size_t kLogMaxFiles = 3;
            constexpr std::size_t kLogQueueSize = 4;

            auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_st>();
            auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_st>(
                log_path, "log", kLogMaxSize, kLogMaxFiles, true);
            std::vector<spdlog::sink_ptr> sinks{console_sink, rotating_sink};
            // [%Y-%m-%d %T.%e][%l][%t]%!: %v [%s:%#] 
            // [2022-08-01 10:10:10.120][info][120]test(): some text [file.cpp:120]
            spdlog::set_pattern("[%Y-%m-%d %T.%e][%l][%t] %v");
            spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level_));
            logger_ = spdlog::create(kLoggerName, sinks.begin(), sinks.end());
            //spdlog::set_async_mode(kLogQueueSize);
            //spdlog::register_logger(logger_ptr); //create register already
        } catch (std::exception &e) {
            ::printf("create logger exception: %s", e.what());
            init_step_ = 0;
            return false;
        }
        init_step_ = 2;
        return true;
    }

    std::shared_ptr<spdlog::logger> logger(const std::string &path) {
        if (init_step_ == 0) {
            create(path);
        }
        return logger_;
    }

    void set_level(int level) {
        log_level_ = level;
        spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
    }

private:
    std::atomic_int init_step_{0};
    std::mutex mtx_;
    int log_level_{2};
    std::shared_ptr<spdlog::logger> logger_{nullptr};
};

} // namespace log
} // namespace comwise

#define __FILENAME__ common::get_file_name(__FILE__).c_str()
#define SPD_WARPPER  comwise::log::spd_logger::instance()
#define SPD_LOGGER   SPD_WARPPER->logger(comwise::log::kLoggerPath)

// use fmt lib, e.g. LOGP_WARN("warn log {}, {}", 1, 2);
#define SPD_LOG_DEBUG(...)          SPD_LOGGER->debug(__VA_ARGS__)
#define SPD_LOG_INFO(...)           SPD_LOGGER->info(__VA_ARGS__)
#define SPD_LOG_WARN(...)           SPD_LOGGER->warn(__VA_ARGS__)
#define SPD_LOG_ERROR(...)          SPD_LOGGER->error(__VA_ARGS__)
#define SPD_LOG_FATAL(...)          SPD_LOGGER->critical(__VA_ARGS__)

// use like printf, e.g. LOGP_WARN("warn log %d-%d", 1, 2);
#define SPD_PRINTF_DEBUG(fmt, ...)  SPD_LOGGER->debug("{}: {} ({}:{})", __FUNCTION__, \
                                        common::format(fmt, ##__VA_ARGS__).c_str(), __FILENAME__, __LINE__)
#define SPD_PRINTF_INFO(fmt, ...)   SPD_LOGGER->info("{}: {} ({}:{})", __FUNCTION__, \
                                        common::format(fmt, ##__VA_ARGS__).c_str(), __FILENAME__, __LINE__)
#define SPD_PRINTF_WARN(fmt, ...)   SPD_LOGGER->warn("{}: {} ({}:{})", __FUNCTION__, \
                                        common::format(fmt, ##__VA_ARGS__).c_str(), __FILENAME__, __LINE__)
#define SPD_PRINTF_ERROR(fmt, ...)  SPD_LOGGER->error("{}: {} ({}:{})", __FUNCTION__, \
                                        common::format(fmt, ##__VA_ARGS__).c_str(), __FILENAME__, __LINE__)
#define SPD_PRINTF_FATAL(fmt, ...)  SPD_LOGGER->critical("{}: {} ({}:{})", __FUNCTION__, \
                                        common::format(fmt, ##__VA_ARGS__).c_str(), __FILENAME__, __LINE__)

// use like stream, e.g. LOG_STREAM_WARN() << "warn log: " << 1;
#define SPD_STREAM_TRACE            SPD_LOGGER->trace()    << __FUNCTION__ << ": "
#define SPD_STREAM_DEBUG            SPD_LOGGER->debug()    << __FUNCTION__ << ": " 
#define SPD_STREAM_INFO             SPD_LOGGER->info()     << __FUNCTION__ << ": "
#define SPD_STREAM_WARN             SPD_LOGGER->warn()     << __FUNCTION__ << ": "
#define SPD_STREAM_ERROR            SPD_LOGGER->error()    << __FUNCTION__ << ": "
#define SPD_STREAM_FATAL            SPD_LOGGER->critical() << __FUNCTION__ << ": "


#endif // __COMWISE_LOG__SPD_LOGGER__H__