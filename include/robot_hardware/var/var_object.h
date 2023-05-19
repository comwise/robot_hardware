#ifndef __COMWISE__VAR_OBJECT__H__
#define __COMWISE__VAR_OBJECT__H__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "chassis_data.h"
#include "joy_data.h"
#include "var_sensor.h"

namespace comwise {

//>! pose
class pose_t 
{
public:
    double x{0.0};
    double y{0.0};
    double z{0.0};
    union {
        double w{0.0};
        double angle;
        double theta;
    };
};

//>! rect
class rect_t
{
public:
    double x{0.0};
    double y{0.0};
    double w{0.0};
    double h{0.0};
};

//>! tfransform
using vector3_t = pose_t;
using quaternion_t = pose_t;

//>! vel object
class vel_t
{
public:
    union {
        double v {0.0};     // velocity m/s
        double speed;       // speed
    };
    union {
        double w {0.0};     // angular speed m/s
        double pos;         // motor position count
    };
    union {
        double angle {0.0}; // angle rad
        double theta;       // rad
    };
    double direction;
    double vv;
};
using vel_list_t = std::vector<vel_t>;

//>! vel priority
enum vel_priority_t 
{
    kVelPriorityDefault     = 0, // default
    kVelPriorityNav         = 1, // navigation
    kVelPriorityKeybord     = 2, // keyboard
    kVelPriorityJoystick    = 3, // joystikc
    kVelPriorityBrake       = 4, // brake
    kVelPriorityMax         = 10 // max
};

//>! all base object
class var_obj_t
{
public:
    std::string obj_id;
    int32_t obj_type{0};
    uint64_t time{0};

    virtual ~var_obj_t() {}
    var_obj_t() = default;
    var_obj_t(const var_obj_t &) = default;
    var_obj_t &operator=(const var_obj_t &) = default;
    var_obj_t(var_obj_t &&) = default;
    var_obj_t &operator=(var_obj_t &&) = default;
};

//>! vel command object
class move_cmd_t : public var_obj_t
{
public:
    uint16_t type{0};           // velocity chassis mode
    vel_priority_t priority{kVelPriorityDefault}; // velocity priority
    vel_list_t velocity;        // velocity data
};

class msg_header_t
{
public:
    uint32_t seq{0};
    uint64_t stamp{0};
    std::string frame_id;
    std::string base_frame_id;
};

enum drv_state_t
{
    kDriverStop    = 0x0 << 0,  // stop drvier
    kDriverStart   = 0x1 << 0,  // start driver
    kDriverDisable = 0x0 << 1,  // disable driver
    kDriverEnable  = 0x1 << 1,  // enable driver
};

enum upate_state_t
{
    kUpdateNone,
    kUpdateAdd,
    kUpdateChanged,
    kUpdateDelete,
};

class etc_obj_t : public var_obj_t
{
public:
    std::string name;           // driver name(id)
    std::string type;           // driver minor type: string
    uint32_t major_type {0};    // driver major type: hw_type_t
    uint32_t minor_type {0};    // driver minor type: uint32_t
    std::string url;            // run argument
    std::string arguments;      // all argument
    uint32_t state{0};          // hardware state: drv_state_t
    uint32_t update{0};         // update state: upate_state_t
    bool node {false};     // false-normal true-ros
    bool is_restart {true};     // restart node?
};

//>! can param
class can_obj_t : public etc_obj_t
{
public:
    enum can_type_t {
        kCanNone  = 0,
        kCanTest1 = 1,
        kCanTest2 = 2,
        kCanTest3 = 3,
    };

    enum can_channel_t {
        kCanChannel0 = 0, // can channel 0
        kCanChannel1 = 1, // can channel 1
    };

public:
    //uint32_t type{0};    // can_type_t
    uint32_t channel{0}; // can_channel_t
    uint32_t can0_baudrate{125};
    uint32_t can1_baudrate{1000};
};

//>! network param
class net_obj_t : public etc_obj_t
{
public:
    std::string local_dev;
    std::string local_addr;
};

//>! modbus param
class modbus_obj_t : public etc_obj_t 
{
public:
    std::string protocol;
};

//>! serial param
class serial_obj_t : public etc_obj_t, public serial_param_t
{
public:

};

//>! calib param
class calib_param_t 
{
public:
    double px{0.0};
    double py{0.0};
    double pz{0.0};
    double pitch{0.0};
    double roll{0.0};
    double yaw{0.0};
};

class calib_obj_t : public etc_obj_t
{
public:
    std::string topic;
    std::string base_frame_id;
    std::string frame_id;
    calib_param_t calib_param;
    std::string features;
    std::string purpose_type;
};

class node_obj_t : public etc_obj_t, public sensor_param_t
{
public:

};

using motor_obj_t = node_obj_t;

//>! chassis object
class chassis_obj_t : public etc_obj_t
{
public:
    MoveModelOption move_option;
};
using sensor_obj_t = node_obj_t;
using audio_obj_t = calib_obj_t;

//>! chassis object
class laser_obj_t : public calib_obj_t
{
public:
    std::string laser_ip;
    uint16_t laser_port {0};
};

class laser2d_obj_t : public laser_obj_t
{
public:
    double scan_frequency {50.0};
    double angle_increment {0.5};
    double angle_min {-135.0};
    double angle_max {135.0};
    bool inverted {false};
    bool intensity {true};
};

class laser3d_obj_t : public laser_obj_t
{
public:
    std::string model;

    double min_range{0.0};
    double max_range{130.0};

    double start_angle{0.0};
    double end_angle{360.0};

    uint16_t start_ring {0};
    uint16_t end_ring {15};

    uint32_t rpm{600};
    uint32_t time_offset{0};
    std::string calib_path;

    bool use_pcap{false};
    std::string pcap_file_name;
    int32_t npackets{-1};
    double pcap_repeated_delay{0.0};
    bool pcap_read_fast{false};
    bool pcap_read_once{false};

    uint16_t difop_port {7788};
    uint16_t resolution{10}; //mm
    double frequency{10.0};  //
};

//>! joy object
class joy_obj_t : public etc_obj_t, public joy_config
{

};

//>! calib object

//>! camera object
class camera_param_t
{
public:

};

class camera_obj_t : public calib_obj_t, public camera_param_t
{
public:
    std::string host_ip;
    uint16_t host_port {80};
};

using io_obj_t = calib_obj_t;

class carrier_obj_t : public calib_obj_t 
{
public:

};

/*! brief tf var*/
class transform_t
{
public:
    vector3_t translation;
    quaternion_t rotation;
};

class transform_stamped_t
{
public:
    msg_header_t header;
    transform_t transform;
};

using tf_list_t = std::vector<transform_stamped_t>;

class tf_obj_t : public var_obj_t
{
public:
    std::vector<std::string> ids;
    std::map<std::string, std::shared_ptr<calib_obj_t>> tfs;
};

/*! brief status var*/
class status_obj_t : public var_obj_t
{
public:
    enum status_level_t
    {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        FATAL = 3
    };

    uint64_t stamp{0};      // time stamp
    std::string name;       // status id
    uint32_t level;         // status level(status_level_t)
    std::string message;    // status desc
    std::string suggest;    // status suggest

    std::map<std::string, std::string> values;

    std::string dev_id;     // device id
    std::string dev_name;   // device name
    std::string dev_type;   // device type
    int32_t dev_status{0}; // device status
    std::string dev_desc;   // device status desc
    std::string other;      // reserved
};

class template_obj_t : public etc_obj_t
{
public:
    std::string id;
    std::string type;
    std::string adapter;
    std::string url;
    std::vector<std::string> env;
    std::vector<std::string> args;
    std::vector<std::string> depend;
    std::string config;
    std::shared_ptr<etc_obj_t> def;
};


} // namespace comwise

#endif // __COMWISE__VAR_OBJECT__H__
