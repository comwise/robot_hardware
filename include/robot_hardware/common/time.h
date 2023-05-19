#ifndef __COMMON__TIME__H__
#define __COMMON__TIME__H__

#include <cstdint>
#include <ctime>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace common {

using namespace std::chrono;

//! timestamp detail show
typedef struct _systime_t
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int milli_second;
} systime_t;

//! timestamp ms
typedef int64_t system_ms;

//! get current time. return millisecond
static system_ms get_now_time()
{
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<milliseconds>(now).count();
}

//! get current time. return systime_t
static systime_t get_local_time()
{
    auto sys_now = std::chrono::system_clock::now();
    auto count_ms = std::chrono::duration_cast<milliseconds>(sys_now.time_since_epoch()).count();
    std::time_t seconds = std::chrono::system_clock::to_time_t(sys_now);
    const int ms = count_ms % 1000;

    struct tm tm_now = {};
#ifdef _WIN32
    localtime_s(&tm_now, &seconds);
#else
    localtime_r(&seconds, &tm_now);
#endif

    return systime_t{
        tm_now.tm_year + 1900,
        tm_now.tm_mon + 1,
        tm_now.tm_mday,
        tm_now.tm_hour,
        tm_now.tm_min,
        tm_now.tm_sec,
        ms };
}

#ifdef _WIN32
//! get time point detail. return systime_t
static systime_t get_local_time(system_ms milliseconds)
{
    struct tm tm_now = { };
    std::time_t t = milliseconds/1000;
    localtime_s(&tm_now, &t);

    return systime_t{
        tm_now.tm_year + 1900,
        tm_now.tm_mon + 1,
        tm_now.tm_mday,
        tm_now.tm_hour,
        tm_now.tm_min,
        tm_now.tm_sec,
        milliseconds % 1000};
}

#else
//! get time point detail. return systime_t
static systime_t get_local_time(system_ms milliseconds)
{
    struct tm tm_now = { };
    std::time_t seconds = milliseconds/1000;
    localtime_r(&seconds, &tm_now);
    int ms = milliseconds % 1000;

    return systime_t{
        tm_now.tm_year + 1900,
        tm_now.tm_mon + 1,
        tm_now.tm_mday,
        tm_now.tm_hour,
        tm_now.tm_min,
        tm_now.tm_sec,
        ms};
}

#endif

inline std::string systime_to_string(const std::chrono::system_clock::time_point &tp)
{
    std::time_t now_c = std::chrono::system_clock::to_time_t(tp - std::chrono::hours(24));
    std::stringstream conv;
    char foo[64] = {};
    if (0 < strftime(foo, sizeof(foo), "%F %T", std::localtime(&now_c)))
        conv << foo;
    return conv.str();
}

inline const std::string get_current_systime(const std::string &fmt="%04d%02d%02d%02d%02d%02d")
{
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm *ptm = localtime(&now);
    char date[64] = {};
    sprintf(date, fmt.c_str(),
        (int)ptm->tm_year + 1900,
        (int)ptm->tm_mon + 1,
        (int)ptm->tm_mday,
        (int)ptm->tm_hour,
        (int)ptm->tm_min,
        (int)ptm->tm_sec);
    return std::move(std::string(date));
}


/**
 * Structure Time_t, used to describe times.
 * @ingroup COMMON_MODULE
 */
struct Time_t
{
    //! Seconds
    int32_t seconds;
    //! Fraction of second (1 fraction = 1/(2^32) seconds)
    uint32_t fraction;
    //! Default constructor. Sets values to zero.
    Time_t()
    {
        seconds = 0;
        fraction = 0;
    }
    /**
     * @param sec Seconds
     * @param frac Fraction of second
     */
    Time_t(int32_t sec, uint32_t frac)
    {
        seconds = sec;
        fraction = frac;
    }
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * Comparison assignment
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if equal
 */
static inline bool operator==(const Time_t &t1, const Time_t &t2)
{
    return (t1.seconds == t2.seconds) && (t1.fraction == t2.fraction);
}

/**
 * Comparison assignment
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if not equal
 */
static inline bool operator!=(const Time_t &t1, const Time_t &t2)
{
    return (t1.seconds != t2.seconds) || (t1.fraction != t2.fraction);
}

/**
 * Checks if a Time_t is less than other.
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if the first Time_t is less than the second
 */
static inline bool operator<(const Time_t &t1, const Time_t &t2)
{
    return (t1.seconds < t2.seconds) || ((t1.seconds == t2.seconds) && (t1.fraction < t2.fraction));
}

/**
 * Checks if a Time_t is less or equal than other.
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if the first Time_t is less or equal than the second
 */
static inline bool operator<=(const Time_t &t1, const Time_t &t2)
{
    return (t1.seconds < t2.seconds) || ((t1.seconds == t2.seconds) && (t1.fraction <= t2.fraction));
}

inline std::ostream &operator<<(std::ostream &output, const Time_t &t)
{
    return output << t.seconds << "." << t.fraction;
}

#endif

const Time_t c_TimeInfinite(0x7fffffff, 0xffffffff);
const Time_t c_TimeZero(0, 0);
const Time_t c_TimeInvalid(-1, 0xffffffff);

typedef Time_t Duration_t;

} // namespace common

#endif // __COMMON__TIME__H__