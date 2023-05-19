#include <thread>
#include <chrono>
#include "log/stdout_layout.h"
#include "log/file_layout.h"
#include "log/xlog_private.h"
#include "log/logger.h"

const char* ROOT = "dds";
const char* PROFILES = "profiles";
const char* PROFILE_NAME = "profile_name";
const char* DEFAULT_PROF = "is_default_profile";
const char* NAME = "test";

using namespace comwise::log;

int main()
{
    printf("hello");
#if 0
    //std::unique_ptr<MockConsumer> consumer(new MockConsumer);
    std::unique_ptr<stdout_layout> consumer(new stdout_layout);
    std::unique_ptr<file_layout> consumer2(new file_layout);
    //mockConsumer = consumer.get();
    //xlog::register_layout(std::move(consumer));
    comwise::log::xlog::register_layout(std::move(consumer2));
    comwise::log::xlog::set_level(xlog_level_t::Info);

    int writer_guid = 11;
    logInfo(NAME, "Loading writer " << writer_guid);
    logError(NAME, "Loading reader is error, id = " << writer_guid);

    //xlog::reset();
    //xlog::kill_thread();
#endif
#if 0
    auto logger = spdlog::stdout_logger_mt("stderr");
    logger->set_pattern("[%Y-%m-%d %T.%e][%l][%t][%!]: %v [%s:%#]");
    //spdlog::get("stderr")->info("loggers can be");

    constexpr std::size_t kLogMaxSize = 1024 * 1024 * 8; // 8MB
    constexpr std::size_t kLogMaxFiles = 3;
    constexpr std::size_t kLogQueueSize = 4;
    std::string log_path = kLoggerPath;
    auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_st>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_st>(
        log_path, "log", kLogMaxSize, kLogMaxFiles);
    std::vector<spdlog::sink_ptr> sinks{console_sink, rotating_sink};
    auto logger_ptr = spdlog::create("test11", sinks.begin(), sinks.end());
    //spdlog::set_async_mode(kLogQueueSize);
    //spdlog::register_logger(logger_ptr);
    // [%Y-%m-%d %T.%e][%l][%t]%!: %v [%s:%#] 
    // [2022-08-01 10:10:10.120][info][120]test(): some text [file.cpp:120]
    spdlog::set_pattern("[%Y-%m-%d %T.%e][%l][%t]: %v");
    spdlog::set_level(static_cast<spdlog::level::level_enum>(2));

    auto log_test = spdlog::get("test11");
    log_test->info("hello");
    log_test->flush();
#endif
#if 1
    //SPD_WARPPER->create(kLoggerPath);
    //auto ptr = spdlog::get("cote_runner");
    //ptr->error("hello world");
    SPD_PRINTF_INFO("%s, %d", "hello", 11);
    SPD_STREAM_ERROR  << "jtest wli";
    SPD_LOG_INFO("{}{}", "13214", 899.01);
    SPD_PRINTF_ERROR("1234asa");
#endif

    getchar();

    return 0;
}