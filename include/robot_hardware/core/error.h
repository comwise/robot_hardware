#ifndef __COMWISE_CORE__ERROR__H__
#define __COMWISE_CORE__ERROR__H__

#include <cstdint>
#include <string>
#include <vector>
#include <tuple>

namespace core {

class error_base
{
public:
    enum level_t {
        kLevelOK = 0,   // all ok
        kLevelTimeout,  // sometime disconnect, can resume it
        kLevelWarn,     // keep report warn periodically or alway, can ignore it
        kLevelError,    // keep report error, need manual operation
        kLevelFatal,    // can't manual, need contact developer
    };
public:
    virtual ~error_base() { }

    virtual void set_code(int code, const std::string &detail = "") { 
        code_ = code; detail_ = detail; }
    virtual int get_code() const { return code_; }

    virtual void set_level(const level_t &level) { detail_ = level; }
    virtual level_t get_level() { return level_; }

    virtual void set_detail(const std::string &detail) { detail_ = detail; }
    virtual std::string get_detail() { return detail_; }

    virtual void set_suggest(const std::string &sd) { suggest_ = sd; }
    virtual std::string get_suggest() { return suggest_; }

    virtual void set_error(int code, const level_t &level, const std::string &detail = "") {
        code_ = code; level_ = level; detail_ = detail; }
    virtual std::tuple<int, level_t, std::string> get_error() {
        return std::make_tuple(code_, level_, detail_); }

    virtual void clear() { code_ = 0; level_ = level_; detail_ = ""; suggest_ = ""; }

protected:
    int code_{0};
    level_t level_{kLevelOK};
    std::string detail_;
    std::string suggest_;
};

template <typename T>
class error_code : public error_base
{
public:
    enum {
      kCodeOK           = 0x0,      // normal
      kCodeSendError    = 0xFFFC,   // send data error
      kCodeInitError    = 0xFFFD,   // init error
      kCodeIllegalParam = 0xFFFE,   // illegal param
      kCodeDisconnected = 0xFFFF    // disconnected alway
    };

public:
    error_code() { }
    virtual ~error_code() { }

    virtual void set_codes(const T &code) { codes_.emplace_back(code); }
    virtual void set_codes(const std::vector<T> &codes) { codes_ = codes; }
    virtual std::vector<T> get_codes() { return codes_; }

    virtual void clear() { codes_.clear(); error_base::clear(); }

private:
    std::vector<T> codes_;
};

} // namespace core

#endif //__COMWISE_CORE__ERROR__H__
