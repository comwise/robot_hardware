#ifndef __COMWISE_CORE__STATUS__H__
#define __COMWISE_CORE__STATUS__H__

#include <memory>
#include <string>

namespace core {

namespace detail {
    struct impl;
}

enum class status_level_t {
    kLevelOK,
    kLevelWarn,
    kLevelError,
    kLevelFatal,
};

enum class decision_level_t {
    kDecisionDeny = -1,    // without consulting with the remaining analyzers
    kDecisionNeutral = 0,  // consulted the next analyzer in the chain
    kDecisionAccept = 1    // without consulting the remaining analyzers
};

class status_data_t {
public:
    int64_t     time_stamp{0};      // time stamp
    
    std::string msg_id;             // message id
    int32_t     msg_level{0};       // message level(status_level_t)
    std::string msg_body;           // message body

    std::string dev_id;             // device id
    int32_t     dev_type{0};        // device type
    int32_t     dev_status{0};      // device status
    int32_t     status_stage{0};    // status stage 
    std::string status_desc;        // status desc
    std::string status_suggest;     // status suggest
};

class status
{
public:
    explicit status(const std::string &id);
    virtual ~status() {}

    std::size_t add(const std::string &name, const std::string &hw_id,
                    status_level_t level, const std::string &message);

    void set(std::size_t position, const std::string &key, const std::string &value);
    void set(std::size_t position, status_level_t level, const std::string &message);
    void set(std::size_t position, status_level_t level, const std::string &message,
             const std::string &key, const std::string &value);

    std::size_t count() const { }
    void clear() { }

    bool is_ok();

private:
    std::shared_ptr<detail::impl> impl;
};

} // namespace core

#endif // __COMWISE_CORE__STATUS__H__
