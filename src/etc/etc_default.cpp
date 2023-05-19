#include "etc/etc_default.h"
#include "etc/etc_pool.h"
#include <strings.h>
#include "log/log.h"
#include "common/linux_chcwd.h"
#include "var/var.h"

namespace comwise {
namespace etc {

static const char*    kBoolRet[] = {"false", "true"}; 

etc_value_t etc_default::get_drv_default(
    const std::string &major, const std::string &minor)
{
    return ETC_POOL? ETC_POOL->get_default(major, minor) : nullptr;
}

etc_value_t etc_default::get_drv_default(uint32_t type)
{
    std::string major = get_drv_type(type);
    return get_drv_default(major);
}

etc_param_t etc_default::get_drv_default()
{
    etc_param_t param;
#if 1
    if (ETC_POOL) {
        param = ETC_POOL->get_default();
    }
#else
    for (uint32_t i = kHwChassis; i < kHwMax; i++) {
        param[kDrvType[i]] = etc_default::get_drv_default(i);
    }
#endif
    return param;
}

bool etc_default::get_drv_default(etc_value_t obj)
{
    if (nullptr == obj) {
        return false;
    }

    std::string module = get_drv_type(obj->major_type);
    obj = ETC_POOL->get_default(module, obj->type);

    return true;
}

std::string etc_default::get_drv_type(uint32_t type)
{
    return (type >= kHwIO && type < kHwMax) ? kDrvType[type] : "";
}

uint32_t etc_default::get_drv_type(const std::string &type)
{
    for (uint32_t i = kHwIO; i < kHwMax; i++) {
        if (0==strcasecmp(kDrvType[i], type.c_str()))
            return i;
    }
    return kHwMax;
}

std::string etc_default::get_drv_node(uint32_t type)
{
    return (type >= kHwIO && type < kHwMax)? kDrvNodeName[type] : "";
}

std::string etc_default::get_drv_url(uint32_t type)
{
    if(type < kHwIO || type >= kHwMax || type == kHwLaser) {
        return "";
    }
    return get_drv_node(type);
}

std::string etc_default::get_can_type(uint32_t type)
{
    return (type >= kCanNone && type <= kCanLike) ? kCanType[type] : "";
}

uint32_t etc_default::get_can_type(const std::string &type)
{
    for (uint32_t i = kCanNone; i <= kCanLike; i++) {
        if (0==strcasecmp(kCanType[i], type.c_str()))
            return i;
    }
    return kCanNone;
}

std::string etc_default::get_motor_type(uint32_t type)
{
    return (type >= kMotorNone && type < kMotorMax) ? kMotorType[type] : "";
}

uint32_t etc_default::get_motor_type(const std::string &type)
{
    for (uint32_t i = kMotorNone; i < kMotorMax; i++) {
        if (kMotorType[i] == type)
            return i;
    }
    return kMotorNone;
}

std::string etc_default::get_io_type(uint32_t type)
{
    return (type >= kIOCommon && type < kIOMax) ? kIOType[type] : "";
}

uint32_t etc_default::get_io_type(const std::string &type)
{
    for (uint32_t i = kIOCommon; i < kIOMax; i++) {
        if (kLaserType[i] == type)
            return i;
    }
    return kHwMax;
}

std::string etc_default::get_chassis_type(uint32_t type)
{
    return (type > kChassisNone && type < kChassisMax) ? kChassisType[type] : "";
}

uint32_t etc_default::get_chassis_type(const std::string &type)
{
    for (uint32_t i = kChassisNone; i < kChassisMax; i++) {
        if (kChassisType[i] == type)
            return i;
    }
    return kChassisMax;
}

std::string etc_default::get_laser_type(uint32_t type)
{
    return (type >= kLaserSick1XX && type < kLaserTypeMax)? kLaserType[type] : "";
}

uint32_t etc_default::get_laser_type(const std::string &type)
{
    for (uint32_t i = kLaserSick1XX; i < kLaserTypeMax; i++) {
        if (kLaserType[i] == type)
            return i;
    }
    return kLaserTypeMax;
}

std::string etc_default::get_camera_type(uint32_t type)
{
    return (type >= kCamera2D && type < kCameraMax)? kCameraType[type] : "";
}

uint32_t etc_default::get_camera_type(const std::string &type)
{
    for (uint32_t i = kCamera2D; i < kCameraMax; i++) {
        if (kCameraType[i] == type)
            return i;
    }
    return kCameraMax;
}

std::string etc_default::get_carrier_type(uint32_t type)
{
    return (type >= kCarrierRoller && type < kCarrierMax)? kCarriertype[type] : "";
}

uint32_t etc_default::get_carrier_type(const std::string &type)
{
    for (uint32_t i = kCarrierRoller; i < kCarrierMax; i++) {
        if (kCarriertype[i] == type)
            return i;
    }
    return kCarrierMax;
}

std::string etc_default::get_battery_type(uint32_t type)
{
    return (type >= kBatteryNone && type < kBatteryMax)? kBatteryType[type] : "";
}

uint32_t etc_default::get_battery_type(const std::string &type)
{
    for (uint32_t i = kBatteryNone; i < kBatteryMax; i++) {
        if (kBatteryType[i] == type)
            return i;
    }
    return kBatteryMax;
}

std::string etc_default::get_imu_type(uint32_t type)
{
    return (type >= kIMUNone && type < kIMUMax)? kIMUType[type] : "";
}

uint32_t etc_default::get_imu_type(const std::string &type)
{
    for (uint32_t i = kIMUNone; i < kIMUMax; i++) {
        if (kIMUType[i] == type)
            return i;
    }
    return kIMUMax;
}

const std::string etc_default::get_xml(uint32_t major_type, uint32_t minor_type)
{
    switch(major_type) {
        case kHwLaser:
            return ETC_POOL->get_value(etc::etc_default::get_laser_type(minor_type));
        case kHwCamera:
            return ETC_POOL->get_value(etc::etc_default::get_camera_type(minor_type));
        default:
            return "";
    }
}

// X_node [run]
//    [--module MODULE]
//    [--name NAME]
//    [--param PARAM]
std::string etc_default::get_node_arg(const drv_cfg_t param)
{
    bool ret = true;
    std::string args;
    try {
        auto cfg = std::dynamic_pointer_cast<etc_obj_t>(param);
        if (nullptr == cfg) {
            LOGS_ERROR << "get etc_obj_t object is nullptr";
            return "";
        }

        uint32_t type = cfg->major_type;
        switch (type) {
        case kHwChassis:
        {
            std::string name = common::get_file_name(cfg->name);
            get_node_arg(DRV_ARG_NODE, etc::etc_default::get_drv_node(type), args);
            get_node_arg(DRV_ARG_MODULE, etc::etc_default::get_drv_type(type), args);
            get_node_arg(DRV_ARG_NAME, name, args);
            get_node_arg(DRV_ARG_PARAM, cfg->arguments, args);
            break;
        }
        case kHwCarrier:
        {
            uint32_t sub_type = cfg->minor_type;
            std::string name = common::get_file_name(cfg->name);
            get_node_arg(DRV_ARG_MODULE, etc::etc_default::get_carrier_type(sub_type), args);
            get_node_arg(DRV_ARG_NAME, name, args);
            get_node_arg(DRV_ARG_PARAM, cfg->arguments, args);
            break;
        }
        case kHwBattery:
        {
            uint32_t sub_type = cfg->minor_type;
            std::string name = common::get_file_name(cfg->name);
            get_node_arg(DRV_ARG_MODULE, etc::etc_default::get_drv_type(type), args);
            get_node_arg(DRV_ARG_NAME, name, args);
            get_node_arg(DRV_ARG_PARAM, cfg->arguments, args);
            break;
        }
        case kHwLaser:
        case kHwCamera:
        case kHwJoystick:
        case kHwGPS:
        case kHwPlc:
        case kHwAudio:
        case kHwModbus:
        case kHwRfid:
        case kHwCan:
        case kHwSerial:
        {
            std::string name = common::get_file_name(cfg->name);
            get_node_arg(DRV_ARG_NODE, etc::etc_default::get_drv_node(type), args);
            get_node_arg(DRV_ARG_MODULE, etc::etc_default::get_drv_type(type), args);
            get_node_arg(DRV_ARG_NAME, name, args);
            get_node_arg(DRV_ARG_PARAM, cfg->arguments, args);
            break;
        }
        default:
            ret = false;
            break;
        }
    } catch (std::exception &e) {
        LOGS_ERROR << "get sub node arg exception, " << e.what();
        ret = false;
    }
    return ret ? args : "";
}

bool etc_default::get_node_arg(drv_arg_type_t type, const std::string &arg, std::string &node)
{
    bool ret = true;
    switch (type)
    {
    case DRV_ARG_NODE:
        if(arg.empty()) {
            return false;
        } else {
            node += arg;
        }
        break;
    case DRV_ARG_MODULE:
        if(arg.empty()) {
            return false;
        } else {
            node += " --module " + arg;
        }
        break;
    case DRV_ARG_NAME:
        node += arg.empty()? "" : (" --name " + arg);
        break;
    case DRV_ARG_PARAM:
        node += arg.empty()? "" : (" --param " + arg);
        break;
    default:
        ret = false;
        break;
    }
    return ret;
}

bool etc_default::get_ros_arg(ros_arg_t &arg, const drv_cfg_t &cfg)
{
    bool ret = false;
    if(nullptr == cfg) {
        return false;
    }
    uint32_t major = cfg->major_type;
    switch(major) {
    case kHwCamera:
        return get_camera_arg(arg, cfg);
    case kHwLaser:
        return get_laser_arg(arg, cfg);
    default:
        break;
    }
    return ret;
}

bool etc_default::get_laser_arg(ros_arg_t &args, const drv_cfg_t &drv_cfg)
{
    bool ret = true;
    try {
        if (nullptr == drv_cfg) {
            LOGS_ERROR << "laser param object is nullptr";
            return false;
        }
        uint32_t laser_type = drv_cfg->minor_type;
        if(laser_type < kLaserRslidar) {
            ret = get_laser2d_arg(args, drv_cfg);
        } else if (laser_type < kLaserTypeMax) {
            ret = get_laser3d_arg(args, drv_cfg);
        } else {
            ret = false;
        }
    } catch(const std::exception &e) {
        LOGS_ERROR << "parse laser config exception, " << e.what();
        ret = false;
    }
    return ret;
}

bool etc_default::get_laser2d_arg(ros_arg_t &args, const drv_cfg_t &drv_cfg)
{
    auto cfg = std::dynamic_pointer_cast<laser2d_obj_t>(drv_cfg);
    if (nullptr == cfg) {
        LOGS_ERROR << "laser2d param object is nullptr";
        return false;
    }

    std::string node_name = etc::etc_default::get_laser_type(cfg->minor_type) + "_" 
        + common::get_file_name(cfg->name);
    std::string status_topic = ETC_POOL? ETC_POOL->get_value(kStatusTopic) : kStatusTopic;

    args.clear();
    args["node_name"]           = node_name;
    args["host"]                = cfg->laser_ip;
    args["port"]                = std::to_string(cfg->laser_port);
    args["topic"]               = cfg->topic;
    args["frame_id"]            = cfg->frame_id;
    args["scan_frequency"]      = std::to_string(cfg->scan_frequency);
    args["angle_resolution"]    = std::to_string(cfg->angle_increment);
    args["start_angle"]         = std::to_string(cfg->angle_min);
    args["end_angle"]           = std::to_string(cfg->angle_max);
    args["inverted"]            = cfg->inverted ? "true" : "false";
    args["intensity"]           = cfg->intensity ? "true" : "false";
    args["health"]              = status_topic.empty() ? kStatusTopic : status_topic;

    std::replace(args["node_name"].begin(), args["node_name"].end(), '-', '_');
    std::replace(args["node_name"].begin(), args["node_name"].end(), '.', '_');
    std::replace(args["node_name"].begin(), args["node_name"].end(), '/', '_');

    return true;
}

bool etc_default::get_laser3d_arg(ros_arg_t &ros_args, const drv_cfg_t &drv_cfg)
{
    auto cfg = std::dynamic_pointer_cast<laser3d_obj_t>(drv_cfg);
    if (nullptr == cfg) {
        LOGS_ERROR << "laser3d param object is nullptr";
        return false;
    }

    std::string ns = etc::etc_default::get_laser_type(cfg->minor_type) + "_" 
        + common::get_file_name(cfg->name);
    
    std::string calib_file("data");
    if(!cfg->calib_path.empty()) {
        calib_file += "/" + cfg->calib_path;
    }

    std::string status_topic = ETC_POOL? ETC_POOL->get_value(kStatusTopic) : kStatusTopic;

    ros_args.clear();

    ros_args["namespace"]       = ns;
    ros_args["calib_path"]      = calib_file;
    ros_args["model"]           = cfg->model;
    ros_args["topic"]           = cfg->topic;
    ros_args["base_frame_id"]   = cfg->base_frame_id;
    ros_args["frame_id"]        = cfg->frame_id;
    ros_args["device_port"]     = std::to_string(cfg->laser_port);
    ros_args["difop_port"]      = std::to_string(cfg->difop_port);
    ros_args["min_range"]       = std::to_string(cfg->min_range);
    ros_args["max_range"]       = std::to_string(cfg->max_range);
    ros_args["start_angle"]     = std::to_string(cfg->start_angle);
    ros_args["end_angle"]       = std::to_string(cfg->end_angle);
    ros_args["start_ring"]      = std::to_string(cfg->start_ring);
    ros_args["end_ring"]        = std::to_string(cfg->end_ring);
    ros_args["rpm"]             = std::to_string(cfg->rpm);
    ros_args["read_fast"]       = std::to_string(cfg->pcap_read_fast);
    ros_args["read_once"]       = std::to_string(cfg->pcap_read_once);
    ros_args["repeat_delay"]    = std::to_string(cfg->pcap_repeated_delay);
    if(cfg->minor_type == kLaserRslidar) {
        ros_args["resolution"] = (cfg->resolution == 10)? "1cm" : "0.5cm";
    } else {
        ros_args["resolution"] = std::to_string(cfg->resolution);
    }
    ros_args["frequency"] = std::to_string(cfg->frequency);
    ros_args["health"] = status_topic.empty()? kStatusTopic : status_topic;

    return true;
}

bool etc_default::get_camera_arg(ros_arg_t &args, const drv_cfg_t &drv_cfg)
{
    try {
        if (nullptr == drv_cfg) {
            LOGS_ERROR << "camera param object is nullptr";
            return false;
        }

        switch (drv_cfg->minor_type)
        {
        case kCameraRealsense:
            return get_realsense_arg(args, drv_cfg);
        case kCameraIfm:
            return get_ifm_arg(args, drv_cfg);
        default:
            break;
        }

    } catch(const std::exception &e) {
        std::string camera_type = etc::etc_default::get_camera_type(drv_cfg->minor_type);
        LOGS_ERROR << "parse camera(" << camera_type << ") config exception, " << e.what();
        return false;
    }
    return true;
}

bool etc_default::get_realsense_arg(ros_arg_t &args, const drv_cfg_t &drv_cfg)
{
    auto cfg = std::dynamic_pointer_cast<camera_obj_t>(drv_cfg);
    if (nullptr == cfg) {
        LOGS_ERROR << "get camera realsense param error";
        return false;
    }
    
    args.clear();
    args["camera_name"]             = common::get_file_name(cfg->name);
    args["base_frame_id"]           = cfg->base_frame_id;
    
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '-', '_');
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '.', '_');
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '/', '_');

    return true;
}

bool etc_default::get_ifm_arg(ros_arg_t &args, const drv_cfg_t &drv_cfg)
{
    auto cfg = std::dynamic_pointer_cast<camera_obj_t>(drv_cfg);
    if (nullptr == cfg) {
        LOGS_ERROR << "get camera ifm param error";
        return false;
    }

    args.clear();
    args["camera_name"] = common::get_file_name(cfg->name);
    args["ns"]          = common::get_file_name(cfg->name);
    args["frame_id"]    = cfg->frame_id;
    args["ip"]          = cfg->host_ip;
    args["xmlrpc_port"] = std::to_string(cfg->host_port);
    
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '-', '_');
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '.', '_');
    std::replace(args["camera_name"].begin(), args["camera_name"].end(), '/', '_');

    return true;
}

} // namespace etc
} // namespace comwise
