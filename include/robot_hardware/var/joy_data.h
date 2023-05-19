#ifndef __COMWISE_JOY__JOY_DATA__H__
#define __COMWISE_JOY__JOY_DATA__H__

#include <cstdint>
#include <atomic>
#include <memory>
#include <vector>

namespace comwise {

struct joy_config {
    std::string device;
    uint32_t coalesce_interval{0};
    uint32_t repeat_interval{0};
};

struct joy_data {
    uint64_t time{0};
    std::vector<int32_t> buttons;
    std::vector<float> axes;
};

} // namespace comwise

#endif //__COMWISE_JOY__JOY_DATA__H__
