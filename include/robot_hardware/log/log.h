#ifndef __COMWISE_LOG__LOG__H__
#define __COMWISE_LOG__LOG__H__

#include "xlog.h"

#define SPD_LOG
#ifdef SPD_LOG
#include "logger.h"

// use like sprintf, e.g. LOG_PRINTF_WARN("warn %d-%d", 1, 2);
#define LOG_PRINTF_DEBUG    SPD_PRINTF_DEBUG
#define LOG_PRINTF_INFO     SPD_PRINTF_INFO
#define LOG_PRINTF_WARN     SPD_PRINTF_WARN
#define LOG_PRINTF_ERROR    SPD_PRINTF_ERROR
#define LOG_PRINTF_FATAL    SPD_PRINTF_FATAL

// use like stream , e.g. LOG_STREAM_WARN() << "warn log: " << 1;
#define LOG_STREAM_TRACE    SPD_STREAM_TRACE
#define LOG_STREAM_DEBUG    SPD_STREAM_DEBUG
#define LOG_STREAM_INFO     SPD_STREAM_INFO
#define LOG_STREAM_WARN     SPD_STREAM_WARN
#define LOG_STREAM_ERROR    SPD_STREAM_ERROR
#define LOG_STREAM_FATAL    SPD_STREAM_FATAL

// short define
#define LOGP_DEBUG          LOG_PRINTF_DEBUG
#define LOGP_INFO           LOG_PRINTF_INFO
#define LOGP_WARN           LOG_PRINTF_WARN
#define LOGP_ERROR          LOG_PRINTF_ERROR
#define LOGP_FATAL          LOG_PRINTF_FATAL

#define LOGS_DEBUG          LOG_STREAM_DEBUG
#define LOGS_INFO           LOG_STREAM_INFO
#define LOGS_WARN           LOG_STREAM_WARN
#define LOGS_ERROR          LOG_STREAM_ERROR
#define LOGS_FATAL          LOG_STREAM_FATAL

#else

// use like sprintf, e.g. LOG_PRINTF_WARN("warn log, %d-%d", 1, 2);
#define LOG_PRINTF_DEBUG    printf
#define LOG_PRINTF_INFO     printf
#define LOG_PRINTF_WARN     printf
#define LOG_PRINTF_ERROR    printf
#define LOG_PRINTF_FATAL    printf

// use like stream , e.g. LOG_STREAM_WARN() << "warn log: " << 1;
#define LOG_STREAM_TRACE    std::cout
#define LOG_STREAM_DEBUG    std::cout
#define LOG_STREAM_INFO     std::cout
#define LOG_STREAM_WARN     std::cout
#define LOG_STREAM_ERROR    std::cout
#define LOG_STREAM_FATAL    std::cout

#endif

#endif // __COMWISE_LOG__LOG__H__
