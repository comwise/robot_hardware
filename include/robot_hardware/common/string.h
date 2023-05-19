#ifndef __COMWISE_COMMON__STRING__H__
#define __COMWISE_COMMON__STRING__H__

#include <string>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <limits.h>

#if defined(_MSC_VER)
    #define VSNPRINTF _vsnprintf
#else
    #define VSNPRINTF vsnprintf
#endif // _MSC_VER

#if defined(__linux__)
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace common {

/**
   Returns a string contructed from the a format specifier
   and a va_list of arguments, analogously to vprintf(3).
   @param format the format specifier.
   @param args the va_list of arguments.
**/
static std::string vform(const char* format, va_list args)
{
    size_t size = 1024;
    char* buffer = new char[size];

    while (true) {
        va_list args_copy;

#if defined(_MSC_VER) || defined(__BORLANDC__)
        args_copy = args;
#else
        va_copy(args_copy, args);
#endif
    
        int n = VSNPRINTF(buffer, size, format, args_copy);

        va_end(args_copy);

        // If that worked, return a string.
        if ((n > -1) && (static_cast<size_t>(n) < size)) {
            std::string s(buffer);
            delete [] buffer;
            return s;
        }

        // Else try again with more space.
        size = (n > -1) ? 
            n + 1 :   // ISO/IEC 9899:1999
        size * 2; // twice the old size

        delete [] buffer;
        buffer = new char[size];
    }
}

/**
   Returns a string identical to the given string but without leading
   or trailing HTABs or spaces.
**/
static std::string trim(const std::string& s)
{
    static const char* whiteSpace = " \t\r\n";

    // test for null string
    if(s.empty())
        return s;

    // find first non-space character
    std::string::size_type b = s.find_first_not_of(whiteSpace);
    if(b == std::string::npos) // No non-spaces
        return "";

    // find last non-space character
    std::string::size_type e = s.find_last_not_of(whiteSpace);

    // return the remaining characters
    return std::string(s, b, e - b + 1);
    
}

/**
   splits a string into a vector of string segments based on the
   given delimiter.
   @param v The vector in which the segments will be stored. The vector
   will be emptied before storing the segments
   @param s The string to split into segments.
   @param delimiter The delimiter character
   @param maxSegments the maximum number of segments. Upon return
   v.size() <= maxSegments.  The string is scanned from left to right
   so v[maxSegments - 1] may contain a string containing the delimiter
   character.
   @return The actual number of segments (limited by maxSegments).
 **/
static unsigned int split(
                            std::vector<std::string> &v,
                            const std::string &s,
                            char delimiter,
                            unsigned int maxSegments = INT_MAX)
{
    v.clear();
    // std::back_insert_iterator<std::vector<std::string>> it(v);
    // return split(it, s, delimiter, maxSegments);
    return -1;
}

/**
   splits a string into string segments based on the given delimiter
   and assigns the segments through an output_iterator.
   @param output The output_iterator through which to assign the string
   segments. Typically this will be a back_insertion_iterator.
   @param s The string to split into segments.
   @param delimiter The delimiter character
   @param maxSegments The maximum number of segments.
   @return The actual number of segments (limited by maxSegments).
**/
template<typename T> 
static unsigned int split(
                            T& output,
                            const std::string& s,
                            char delimiter,
                            unsigned int maxSegments = INT_MAX) 
{
    std::string::size_type left = 0;
    unsigned int i;
    for(i = 1; i < maxSegments; i++) 
    {
        std::string::size_type right = s.find(delimiter, left);
        if (right == std::string::npos) 
        {
            break;
        }
        *output++ = s.substr(left, right - left);
        left = right + 1;
    }
    
    *output++ = s.substr(left);
    return i;
}

static std::string& string_replace(
                                    std::string &str,
                                    const std::string &old_val,
                                    const std::string &new_val)
{
    std::string::size_type pos = 0;
    for(; pos != std::string::npos; pos += new_val.length()) {
        if((pos = str.find(old_val,pos)) != std::string::npos) {
            str.replace(pos, old_val.length(), new_val);
        } else {
            break;
        }
    }
    return str;
}

} // namespace common

#endif // defined(__linux__)

#endif // __COMWISE_COMMON__STRING__H__
