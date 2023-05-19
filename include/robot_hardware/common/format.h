#ifndef __COMWISE_COMMON__FORMAT__H__
#define __COMWISE_COMMON__FORMAT__H__

#if defined(__linux__)

#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace common {

static std::string format(const char* format, ...);
static std::string trace(char* fmt, ...);

std::string format(const char* format, ...)
{
    std::string result;
    size_t buffer_size_bytes = 256;
    char *buffer = (char *)malloc(buffer_size_bytes);
    if (buffer == NULL)
        return result;
    try {
        va_list arg;

        bool done = false;
        unsigned int loop_count = 0;
        while (!done) {
            va_start(arg, format);
            int return_value = vsnprintf(buffer, buffer_size_bytes, format, arg);
            if (return_value < 0) {
                done = true;
            } else if (return_value >= buffer_size_bytes) {
                // Realloc and try again.
                buffer_size_bytes = return_value + 1;
                char *new_buffer_ptr = (char *)realloc(buffer, buffer_size_bytes);
                if (new_buffer_ptr == NULL) {
                    done = true;
                } else {
                    buffer = new_buffer_ptr;
                }
            } else {
                result = buffer;
                done = true;
            }
            va_end(arg);
            if (++loop_count > 5) {
                done = true;
            }
        }
        free(buffer);
    } catch (std::exception &e) {
        result = "";
        if (NULL != buffer) {
            free(buffer);
        }
    } catch (...) {
        result = "";
    }

    return result;
}

std::string trace(char* fmt, ...)
{ 
    std::string str;
    try {
        char buff[4096] = { 0 };
        va_list arg;
        va_start(arg, fmt);
        if(-1 == vsnprintf(buff, sizeof(buff)/sizeof(char), fmt, arg))
            return "";
        va_end(arg);

        str = std::string(buff);
        std::size_t pos = str.find_last_of('\n');
        if (pos >= 0 && pos < str.length()) {
            str = str.substr(0, pos);
        }
    } catch(std::exception& e) {
        str = "";
    } catch(...) {
        str = "";
    }
    return str;
}

} // namespace common

#endif // defined(__linux__)

#endif // __COMWISE_COMMON__FORMAT__H__
