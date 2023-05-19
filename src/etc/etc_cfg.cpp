#include "etc/etc_cfg.h"
#include <iostream>
#include <fstream>
#include <json/json.h>
#include "log/log.h"
#include "etc/etc_default.h"
#include "common/time.h"
#include "common/linux_chcwd.h"

namespace comwise {
namespace etc {


etc_json::etc_json(const std::string &file)
    : etc_cfg(file)
{

}
etc_json::~etc_json()
{

}

bool etc_json::load(std::string &err /*= ""*/)
{
    bool ret = true;
    Json::Value root;
    try {
        Json::Reader reader;
        std::ifstream ifs;
        ifs.open(cfg_file_, std::ios::binary);
        if (reader.parse(ifs, root)) {
            ret = parse(root);
        } else {
            LOGP_ERROR("parse json file error, file = %s", cfg_file_.c_str());
            ret = false;
        }
        ifs.close();
    } catch (std::exception &e) {
        LOGP_ERROR("read json file(%s) exception, %s", cfg_file_.c_str(), e.what());
        ret = false;
    }
    if (ret) {
        cfg_obj_ = std::make_shared<Json::Value>(root);
    }
    return ret;
}

bool etc_json::save(bool is_format /*= true*/)
{
    bool ret = false;
    try {
        json_value_t value;
        if (!pack(value)) {
            LOGP_WARN("pack json data failed, file = %s", cfg_file_.c_str());
            return ret;
        }

        //! 1\ back old file
        std::string file_name = common::get_file_name(cfg_file_);
        std::string dir = common::get_file_path(cfg_file_) + "/bak/";
        if(!common::is_dir_exist(dir)) {
            if(!common::create_dir(dir)) {
                LOGP_WARN("create directory failed, dir = %s", dir.c_str());
            }
        }
        std::string back_name = dir + file_name + "_" + common::get_current_systime() + ".bak";
        if (std::rename(cfg_file_.c_str(), back_name.c_str())) {
            LOGP_WARN("backup config file failed, file = %s", back_name.c_str());
        }

        //! 2\ write new config
        std::string write_json;
        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = is_format? "  " : "";
        write_json = Json::writeString(builder, value);
        std::ofstream ofs;
        ofs.open(cfg_file_);
        ofs << write_json;
        ofs.close();
        ret = true;
    } catch (std::exception &e) {
        LOGP_ERROR("write json file(%s) exception, %s", cfg_file_.c_str(), e.what());
        ret = false;
    }
    return ret;
}

bool etc_json::parse(const json_value_t &value)
{
    return true;
}

bool etc_json::pack(json_value_t &value)
{
    return true;
}

etc_parse::etc_parse(const std::string &file)
    : etc_json(file)
{

}
etc_parse::~etc_parse()
{

}

bool etc_parse::parse_value(const json_value_t &value, uint32_t module, uint32_t type, etc_value_t &obj)
{
    bool ret = false;
    switch (module)
    {
    case kHwMotor:
        ret = parse_motor(value, obj); break;
    case kHwChassis:
        ret = parse_chassis(value, obj); break;
    case kHwLaser:
        ret = parse_laser(value, obj); break;
    case kHwCamera:
        ret = parse_camera(value, obj); break;
    case kHwCan:
        ret = parse_can(value, obj); break;
    case kHwNetwork:
        ret = parse_net(value, obj); break;
    case kHwIO:
        ret = parse_io(value, obj); break;
    case kHwSerial:
        ret = parse_serial(value, obj); break;
    case kHwModbus:
        ret = parse_modbus(value, obj); break;
    case kHwAudio:
        ret = parse_audio(value, obj); break;
    case kHwIMU:
    case kHwBattery:
    case kHwUltrasonic:
    case kHwGPS:
    {
        ret = parse_sensor(value, obj);
        obj->major_type = module;
        break;
    }
    case kHwJoystick:
        ret = parse_joy(value, obj); break;
    case kHwCarrier:
        ret = parse_carrier(value, obj); break;
    default:
        ret = false; break;
    }
    return ret;
}

bool etc_parse::parse_value(const json_value_t &value,
    const std::string &major, const std::string &minor, etc_value_t &obj)
{
    uint32_t module = etc::etc_default::get_drv_type(major);
    uint32_t type = 0;
    if (module == kHwLaser) {
        type = etc::etc_default::get_laser_type(minor);
    }
    return parse_value(value, module, type, obj);
}

bool etc_parse::pack_value(etc_value_t &etc, json_value_t &json)
{
    bool ret = true;
    if(nullptr == etc) {
        LOGP_WARN("param value object is nullptr");
        return false;
    }
    switch (etc->major_type)
    {
    case kHwMotor:
        ret &= pack_motor(etc, json); break;
    case kHwChassis:
        ret &= pack_chassis(etc, json); break;
    case kHwLaser:
        ret &= pack_laser(etc, json); break;
    case kHwCamera:
        ret &= pack_camera(etc, json); break;
    case kHwCan:
        ret &= pack_can(etc, json); break;
    case kHwNetwork:
        ret &= pack_net(etc, json); break;
    case kHwSerial:
        ret &= pack_serial(etc, json); break;
    case kHwModbus:
        ret &= pack_modbus(etc, json); break;
    case kHwCarrier:
        ret &= pack_carrier(etc, json); break;
    case kHwIMU:
    case kHwBattery:
    case kHwUltrasonic:
    case kHwGPS:
        ret &= pack_sensor(etc, json); break;
    case kHwJoystick:
        ret &= pack_joy(etc, json); break;
    case kHwAudio:
        ret &= pack_audio(etc, json); break;
    case kHwCalib:
        ret &= pack_tf(etc, json); break;
    default:
        ret &= false;
        break;
    }
    return ret;
}

bool etc_parse::parse_motor(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<motor_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<motor_obj_t>();
    }
    std::string type = value["type"].asString();
    obj->major_type = kHwMotor;
    obj->minor_type = etc::etc_default::get_motor_type(type);
    obj->type       = type;
    obj->name       = value["name"].asString();
    obj->node_id    = value["node"].asUInt();
    obj->channel    = value["channel"].asUInt();
    obj->position   = value["position"].asUInt();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_chassis(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<chassis_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<chassis_obj_t>();
    }
    std::string type = value["type"].asString();
    obj->major_type = kHwChassis;
    obj->minor_type = etc::etc_default::get_chassis_type(type);
    obj->type       = type;
    obj->name       = value["name"].asString();
    obj->move_option.model_type = obj->minor_type;

    if (value.isMember("bicycle_model")) {
        auto &model = value["bicycle_model"];
        auto &option = obj->move_option.steer_model_option;
        option.wheel_base         = model["wheel_base"].asDouble();
        option.max_wheel_base     = model["max_wheel_base"].asDouble();
        option.min_wheel_base     = model["min_wheel_base"].asDouble();
        option.eccentric_distance = model["eccentric_distance"].asDouble();
        option.eccentric_angle    = model["eccentric_angle"].asDouble();
        option.reduction_ratio    = model["reduction_ratio"].asDouble();
    }

    if (value.isMember("diff_model")) {
        auto &model = value["diff_model"];
        auto &option = obj->move_option.diff_model_option;
        option.wheel_track              = model["wheel_track"].asDouble();
        option.wheel_diameter           = model["wheel_diameter"].asDouble();
        option.left_motor_reduce_ratio  = model["left_reduce_ratio"].asDouble();
        option.right_motor_reduce_ratio = model["right_reduce_ratio"].asDouble();
        option.left_motor_direction     = model["left_direction"].asInt();
        option.right_motor_direction    = model["right_direction"].asInt();
    }

    etc_val = obj;

    return true;
}

bool etc_parse::parse_camera(const json_value_t &val, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<camera_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<camera_obj_t>();
    }
    obj->major_type = kHwCamera;
    obj->minor_type = 0;
    obj->name       = val["name"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_laser(const json_value_t &obj, etc_value_t &cfg)
{
    bool ret = false;
    std::string type = obj["type"].asString();
    uint32_t laser_type = etc_default::get_laser_type(type);
    if (laser_type < kLaserSick1XX || laser_type >= kLaserTypeMax) {
        LOG_STREAM_ERROR << "laser object type error, type = " << type;
        return false;
    }
    bool is_2d = (laser_type >= kLaserSick1XX) && (laser_type < kLaserRslidar);
    if (is_2d) {
        ret = parse_laser2d(obj, cfg);
    } else {
        ret = parse_laser3d(obj, cfg);
    }

    return ret;
}

bool etc_parse::parse_laser2d(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<laser2d_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<laser2d_obj_t>();
    }

    std::string type        = value["type"].asString();

    obj->major_type         = kHwLaser;
    obj->minor_type         = etc_default::get_laser_type(type);
    obj->node               = value.isMember("node")? value["node"].asString()=="ros" : true;
    obj->name               = value["name"].asString();
    obj->type               = type;
    obj->laser_ip           = value["2d_param"]["laser_ip"].asString();
    obj->laser_port         = value["2d_param"]["laser_port"].asUInt();
    obj->angle_min          = value["2d_param"]["angle_min"].asDouble();
    obj->angle_max          = value["2d_param"]["angle_max"].asDouble();
    obj->angle_increment    = value["2d_param"]["angle_increment"].asDouble();
    obj->scan_frequency     = value["2d_param"]["scan_frequency"].asDouble();
    obj->topic              = value["2d_param"]["topic"].asString();
    obj->base_frame_id      = value["base_frame_id"].asString();
    obj->frame_id           = value["frame_id"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_laser3d(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<laser3d_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<laser3d_obj_t>();
    }
    std::string type            = value["type"].asString();
    
    obj->major_type             = kHwLaser;
    obj->minor_type             = etc_default::get_laser_type(type);
    obj->node                   = value.isMember("node")? value["node"].asString()=="ros" : true;
    obj->name                   = value["name"].asString(); 
    obj->type                   = type;
    
    obj->base_frame_id          = value["base_frame_id"].asString();
    obj->frame_id               = value["frame_id"].asString();
    obj->topic                  = value["3d_param"]["topic"].asString();
    
    obj->laser_ip               = value["3d_param"]["laser_ip"].asString();
    obj->laser_port             = value["3d_param"]["laser_port"].asInt();
    obj->difop_port             = value["3d_param"]["difop_port"].asUInt();
    obj->model                  = value["3d_param"]["model"].asString();
    
    obj->rpm                    = value["3d_param"]["rpm"].asUInt();
    obj->npackets               = value["3d_param"]["npackets"].asInt();
    obj->time_offset            = value["3d_param"]["time_offset"].asUInt();

    obj->min_range              = value["3d_param"]["min_range"].asDouble();
    obj->max_range              = value["3d_param"]["max_range"].asDouble();
    obj->start_angle            = value["3d_param"]["start_angle"].asDouble();
    obj->end_angle              = value["3d_param"]["end_angle"].asDouble();
    obj->start_ring             = value["3d_param"]["start_ring"].asUInt();
    obj->end_ring               = value["3d_param"]["end_ring"].asUInt();
    obj->resolution             = value["3d_param"]["resolution"].asUInt();
    obj->frequency              = value["3d_param"]["frequency"].asDouble();

    obj->calib_path             = value["3d_param"]["calib_path"].asString();
    obj->use_pcap               = value["3d_param"]["use_pcap"].asBool();
    obj->pcap_file_name         = value["3d_param"]["pcap_file_name"].asString();
    obj->pcap_read_fast         = value["3d_param"]["pcap_read_fast"].asBool();
    obj->pcap_read_once         = value["3d_param"]["pcap_read_once"].asBool();
    obj->pcap_repeated_delay    = value["3d_param"]["pcap_repeated_delay"].asDouble();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_can(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<can_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<can_obj_t>();
    }

    obj->major_type    = kHwCan;
    obj->minor_type    = value["type"].asUInt();
    obj->name          = value["name"].asString();
    obj->type          = etc::etc_default::get_can_type(obj->minor_type);
    obj->channel       = value["channel"].asUInt();
    obj->can0_baudrate = value["baudrate"]["can0"].asUInt();
    obj->can1_baudrate = value["baudrate"]["can1"].asUInt();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_net(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<net_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<net_obj_t>();
    }

    obj->major_type    = kHwNetwork;
    obj->minor_type    = 0;
    obj->name          = value["name"].asString();
    obj->type          = value["type"].asString();
    obj->local_addr    = value["local_addr"].asString();
    obj->local_dev     = value["local_device"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_serial(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<serial_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<serial_obj_t>();
    }

    obj->major_type    = kHwSerial;
    obj->minor_type    = 0;
    obj->name          = value["name"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_modbus(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<modbus_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<modbus_obj_t>();
    }

    obj->major_type    = kHwModbus;
    obj->minor_type    = 0;
    obj->name          = value["name"].asString();
    obj->protocol      = value["type"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_io(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<io_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<io_obj_t>();
    }

    obj->major_type    = kHwIO;
    obj->minor_type    = 0;
    obj->name          = value["name"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_carrier(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<carrier_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<carrier_obj_t>();
    }

    obj->major_type    = kHwCarrier;
    obj->minor_type    = 0;
    obj->name          = value["name"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_joy(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<joy_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<joy_obj_t>();
    }

    obj->major_type        = kHwJoystick;
    obj->minor_type        = 0;
    obj->name              = value["name"].asString();
    obj->type              = value["type"].asString();
    obj->device            = value["device"].asString();
    obj->coalesce_interval = 20;
    obj->repeat_interval   = 20;

    etc_val = obj;

    return true;
}

bool etc_parse::parse_sensor(const json_value_t &val, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<sensor_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<sensor_obj_t>();
    }

    obj->name       = val["name"].asString();
    obj->type       = val["type"].asString();
    obj->major_type = kHwBattery;
    obj->minor_type = etc_default::get_battery_type(obj->type);

    // can
    obj->node_id    = val["node_id"].asInt();
    obj->channel    = val["channel"].asInt();
    obj->mode       = val["mode"].asInt();
    
    // serial
    obj->device     = val["dev"].asString();
    obj->baudrate   = val["baudrate"].asUInt();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_audio(const json_value_t &value, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<audio_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<audio_obj_t>();
    }

    obj->major_type        = kHwAudio;
    obj->minor_type        = 0;
    obj->name              = value["name"].asString();

    etc_val = obj;

    return true;
}

bool etc_parse::parse_tf(const json_value_t &val, etc_value_t &etc_val)
{
    auto obj = std::dynamic_pointer_cast<calib_obj_t>(etc_val);
    if (!obj) {
        obj = std::make_shared<calib_obj_t>();
    }

    obj->major_type         = kHwCalib;
    obj->minor_type         = 0;
    obj->name               = val["name"].asString();

    etc_val = obj;

    return true;
}

//!> pack module
bool etc_parse::pack_motor(etc_value_t &etc, json_value_t &obj)
{
    bool ret = false;
    auto cfg = std::dynamic_pointer_cast<motor_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("motor_obj_t config object is nullptr");
        return false;
    }

    obj["name"]        = cfg->name;
    obj["type"]        = cfg->type;
    obj["channel"]     = cfg->channel;
    obj["node"]        = cfg->node_id;
    obj["position"]    = cfg->position;

    return true;
}

bool etc_parse::pack_chassis(etc_value_t &etc, json_value_t &obj)
{
    bool ret = false;
    auto cfg = std::dynamic_pointer_cast<chassis_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("chassis_obj_t config object is nullptr");
        return false;
    }

    obj["name"]        = cfg->name;
    obj["type"]        = cfg->type;

    auto &option = cfg->move_option.steer_model_option;
    obj["bicycle_model"]["wheel_base"]          = option.wheel_base;
    obj["bicycle_model"]["max_wheel_base"]      = option.max_wheel_base;
    obj["bicycle_model"]["min_wheel_base"]      = option.min_wheel_base;
    obj["bicycle_model"]["eccentric_distance"]  = option.eccentric_distance;
    obj["bicycle_model"]["eccentric_angle"]     = option.eccentric_angle;
    obj["bicycle_model"]["reduction_ratio"]     = option.reduction_ratio;

	auto &diff = cfg->move_option.diff_model_option;
    obj["diff_model"]["wheel_track"]            = diff.wheel_track;
    obj["diff_model"]["wheel_diameter"]         = diff.wheel_diameter;
    obj["diff_model"]["left_reduce_ratio"]      = diff.left_motor_reduce_ratio;
    obj["diff_model"]["right_reduce_ratio"]     = diff.right_motor_reduce_ratio;
    obj["diff_model"]["left_direction"]         = diff.left_motor_direction;
    obj["diff_model"]["right_direction"]        = diff.right_motor_direction;

    return true;
}

bool etc_parse::pack_camera(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<camera_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("camera_obj_t config object is nullptr");
        return false;
    }

    obj["name"]     = cfg->name;
    obj["topic"]    = cfg->topic;
    obj["type"]     = etc_default::get_camera_type(cfg->minor_type);
    obj["features"] = cfg->features;

    return true;
}

bool etc_parse::pack_laser(etc_value_t &etc, json_value_t &obj)
{
    if(nullptr == etc) {
        LOGP_ERROR("laser config object is nullptr");
        return false;
    }

    bool is_3d = etc->minor_type >= kLaserRslidar;
    if (is_3d) {
        auto cfg = std::dynamic_pointer_cast<laser3d_obj_t>(etc);
        if (nullptr == cfg) {
            LOGP_ERROR("laser3d_obj_t object is nullptr, id = %s", etc->name.c_str());
            return false;
        }
        obj["name"]                         = cfg->name;
        obj["topic"]                        = cfg->topic;
        obj["type"]                         = etc_default::get_laser_type(cfg->minor_type);
        obj["purpose_type"]                 = cfg->purpose_type;
        obj["features"]                     = cfg->features;
        obj["3d_param"]["laser_ip"]         = cfg->laser_ip;
        obj["3d_param"]["laser_port"]       = cfg->laser_port;
        obj["3d_param"]["model"]            = cfg->model;
        obj["3d_param"]["calib_path"]       = cfg->calib_path;
        obj["3d_param"]["rpm"]              = cfg->rpm;
        obj["3d_param"]["time_offset"]      = cfg->time_offset;
        obj["3d_param"]["min_range"]        = cfg->min_range;
        obj["3d_param"]["max_range"]        = cfg->max_range;
        obj["3d_param"]["use_pcap"]         = cfg->use_pcap;
        obj["3d_param"]["npackets"]         = cfg->npackets;
        obj["3d_param"]["pcap_file_name"]   = cfg->pcap_file_name;
        obj["3d_param"]["pcap_read_fast"]   = cfg->pcap_read_fast;
        obj["3d_param"]["pcap_read_once"]   = cfg->pcap_read_once;
        obj["3d_param"]["pcap_repeated_delay"] = cfg->pcap_repeated_delay;
        obj["3d_param"]["view_direction"]   = cfg->start_angle;
        obj["3d_param"]["view_width"]       = cfg->end_angle;
        obj["3d_param"]["start_ring"]       = cfg->start_ring;
        obj["3d_param"]["end_ring"]         = cfg->end_ring;
        obj["3d_param"]["difop_port"]       = cfg->difop_port;
        obj["3d_param"]["resolution"]       = cfg->resolution;
        obj["3d_param"]["frequency"]        = cfg->frequency;
    } else {
        auto cfg = std::dynamic_pointer_cast<laser2d_obj_t>(etc);
        if (nullptr == cfg) {
            LOGP_ERROR("laser2d_obj_t object is nullptr, id = %s", etc->name.c_str());
            return false;
        }
        obj["name"]                         = cfg->name;
        obj["topic"]                        = cfg->topic;
        obj["type"]                         = etc_default::get_laser_type(cfg->minor_type);
        obj["purpose_type"]                 = cfg->purpose_type;
        obj["features"]                     = cfg->features;
        obj["2d_param"]["laser_ip"]         = cfg->laser_ip;
        obj["2d_param"]["laser_port"]       = cfg->laser_port;
        obj["2d_param"]["angle_max"]        = cfg->angle_max;
        obj["2d_param"]["angle_min"]        = cfg->angle_min;
        obj["2d_param"]["angle_increment"]  = cfg->angle_increment;
        obj["2d_param"]["scan_frequency"]   = cfg->scan_frequency;
        obj["2d_param"]["inverted"]         = cfg->inverted;
        obj["2d_param"]["intensity"]        = cfg->intensity;
        obj["base_frame_id"]                = cfg->base_frame_id;
        obj["frame_id"]                     = cfg->frame_id;
    }

    return true;
}

bool etc_parse::pack_can(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<can_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("can config object is nullptr");
        return false;
    }

    obj["name"]             = cfg->name;
    obj["type"]             = cfg->minor_type;
    obj["channel"]          = cfg->channel;
    obj["baudrate"]["can0"] = cfg->can0_baudrate;
    obj["baudrate"]["can1"] = cfg->can1_baudrate;

    return true;
}

bool etc_parse::pack_net(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<net_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("network config object is nullptr");
        return false;
    }

    obj["name"]         = cfg->name;
    obj["type"]         = cfg->type;
    obj["local_addr"]   = cfg->local_addr;
    obj["local_device"] = cfg->local_dev;

    return true;
}

bool etc_parse::pack_serial(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<serial_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("serial config object is nullptr");
        return false;
    }

    obj["name"] = cfg->name;
    obj["type"] = cfg->type;

    return true;
}

bool etc_parse::pack_modbus(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<modbus_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("modbus config object is nullptr");
        return false;
    }

    obj["name"] = cfg->name;
    obj["type"] = cfg->protocol;

    return true;
}

bool etc_parse::pack_io(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<io_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("io config object is nullptr");
        return false;
    }

    obj["name"]     = cfg->name;
    obj["type"]     = cfg->type;

    return true;
}

bool etc_parse::pack_carrier(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<carrier_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("carrier config object is nullptr");
        return false;
    }

    obj["name"]     = cfg->name;
    obj["type"]     = cfg->type;
    obj["features"] = cfg->features;

    return true;
}

bool etc_parse::pack_joy(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<joy_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("joystick config object is nullptr");
        return false;
    }

    obj["name"]                 = cfg->name;
    obj["type"]                 = cfg->type;
    obj["device"]               = cfg->device;
    obj["coalesce_interval"]    = cfg->coalesce_interval;
    obj["repeat_interval"]      = cfg->repeat_interval;

    return true;
}

bool etc_parse::pack_sensor(etc_value_t &etc, json_value_t &obj)
{
    bool ret = false;
    auto cfg = std::dynamic_pointer_cast<sensor_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("sensor config object is nullptr");
        return false;
    }

    obj["name"]      = cfg->name;
    obj["type"]      = cfg->type;

    // can
    obj["channel"]   = cfg->channel;
    obj["node_id"]   = cfg->node_id;
    obj["mode"]      = cfg->mode;
    
    // serial
    obj["dev"]       = cfg->device;
    obj["baudrate"]  = cfg->baudrate;

    return true;
}

bool etc_parse::pack_audio(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<audio_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("audio config object is nullptr");
        return false;
    }

    obj["name"]     = cfg->name;
    obj["type"]     = cfg->type;

    return true;
}

bool etc_parse::pack_tf(etc_value_t &etc, json_value_t &obj)
{
    auto cfg = std::dynamic_pointer_cast<calib_obj_t>(etc);
    if (nullptr == cfg) {
        LOGP_ERROR("calib_obj_t config object is nullptr");
        return false;
    }

    obj["name"]          = cfg->name;
    obj["type"]          = cfg->type;

    obj["base_frame_id"] = cfg->base_frame_id;
    obj["frame_id"]      = cfg->frame_id;

    pack_calib(obj, cfg->calib_param);

    return true;
}

bool etc_parse::parse_calib(const json_value_t &obj, calib_param_t &param)
{
    param.px     = obj["px"].asDouble();
    param.py     = obj["py"].asDouble();
    param.pz     = obj["pz"].asDouble();
    param.pitch  = obj["pitch"].asDouble();
    param.roll   = obj["roll"].asDouble();
    param.yaw    = obj["yaw"].asDouble();

    return true;
}

bool etc_parse::pack_calib(json_value_t &obj, const calib_param_t &param)
{
    obj["px"]       = param.px;
    obj["py"]       = param.py;
    obj["pz"]       = param.pz;
    obj["pitch"]    = param.pitch;
    obj["roll"]     = param.roll;
    obj["yaw"]      = param.yaw;
}

template <typename T>
bool etc_parse::parse(const json_value_t &json, T &val)
{
    bool ret = true;
    switch(json.type()) {
    case Json::stringValue:
    {
        if (std::is_same<T, std::string>::value){
            val = json.asString();
        }
        break;
    }
    case Json::realValue:
    {
        if (std::is_same<T, double>::value ||
            std::is_same<T, float>::value) {
            val = json.asDouble();
        }
        break;
    }
    case Json::uintValue:
    case Json::intValue:
    {
        if (std::is_same<T, int>::value ||
            std::is_same<T, unsigned int>::value ||
            std::is_same<T, long>::value ||
            std::is_same<T, unsigned long>::value ) {
            val = json.asInt();
        }
        break;
    }
    case Json::booleanValue:
    {
        if (std::is_same<T, bool>::value ||
            std::is_same<T, int>::value) {
            val = json.asBool();
        }
        break;
    }
    default:
        ret = false;
        break;
    }
    return ret;
}

} // namespace etc
} // namespace comwise