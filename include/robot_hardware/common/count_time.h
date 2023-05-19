/*! @file count_time.hpp
 *  @code usage:
 *        #include <count_time.hpp>
 *        count_time time;
 *        time.begin();
 *        coding .....
 *        time.end();
 *        printf("use time = %f\n",getTime());
 *  @endcode
 *
 */

#ifndef __COMMON__COUNT_TIME__H__
#define __COMMON__COUNT_TIME__H__

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <chrono>

namespace common {

class count_time
{
public:
    count_time() {
        begin();
    }

    void begin() {
        begin_time_ = std::chrono::steady_clock::now();
    }

    void end() {
        end_time_ = std::chrono::steady_clock::now();
    }

    int64_t diff() const {
        return std::chrono::duration_cast<
            std::chrono::milliseconds>(end_time_ - begin_time_).count();
    }

    int64_t delta() {
        end();
        return std::chrono::duration_cast<
            std::chrono::milliseconds>(end_time_ - begin_time_).count();
    }

    void print() {
        printf("@time count = %ldms\n", diff());
    }

private:
    std::chrono::steady_clock::time_point begin_time_;
    std::chrono::steady_clock::time_point end_time_;
};

} // namespace common

#endif //__COMMON__COUNT_TIME__H__
