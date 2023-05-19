#ifndef __COMWISE_JOY__JOY_DATA__H__
#define __COMWISE_JOY__JOY_DATA__H__

#include <cstdint>
#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include "joy_key.h"

namespace comwise {
namespace joy {

enum vel_priority_t {
    kVelPriorityDefault     = 0, // default
    kVelPriorityNav         = 1, // navigation
    kVelPriorityKeybord     = 2, // keyboard
    kVelPriorityJoystick    = 3, // joystikc
    kVelPriorityBrake       = 4, // brake
    kVelPriorityMax         = 10 // max
};

enum chassis_type_t {
    kChassisNone     = 0,
    kChassisSteer    = 1,
    kChassisDiff     = 2,
    kChassisSMSR     = 3,
    kChassisMecanum  = 4,
    kChassisMax,
};

struct joy_data {
    uint64_t time;
    std::vector<int32_t> buttons;
    std::vector<float> axes;
};

struct vel_t {
    double v {0.0}; // speed or velocity m/s
    union {
        double angle {0.0}; // angle rad 
        double theta;   //rad
    };
    union {
        double w {0.0}; // angular_speed m/s
        double pos;  // motor position count
    };
};
typedef std::vector<vel_t> vel_list_t;

struct vel_cmd_t {
    uint16_t type{0};
    uint16_t priority{0};
    vel_list_t vel;
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_DATA__H__
