#ifndef __COMWISE_ETC__ETC_DEFAULT__H__
#define __COMWISE_ETC__ETC_DEFAULT__H__

#include "etc_arg.h"

namespace comwise {
namespace etc {

class etc_default
{
  using drv_cfg_t = std::shared_ptr<etc_obj_t>;
  using ros_arg_t = std::map<std::string, std::string>;

  enum drv_arg_type_t {
    DRV_ARG_NODE,
    DRV_ARG_MODULE,
    DRV_ARG_NAME,
    DRV_ARG_PARAM,
    DRV_ARG_URL,
  };

public:
    static std::string get_drv_type(uint32_t type);
    static uint32_t get_drv_type(const std::string &type);

    static std::string get_can_type(uint32_t type);
    static uint32_t get_can_type(const std::string &type);

    static std::string get_curtis_type(uint32_t type);
    static uint32_t get_curtis_type(const std::string &type);

    static std::string get_motor_type(uint32_t type);
    static uint32_t get_motor_type(const std::string &type);

    static std::string get_io_type(uint32_t type);
    static uint32_t get_io_type(const std::string &type);

    static std::string get_chassis_type(uint32_t type);
    static uint32_t get_chassis_type(const std::string &type);

    static std::string get_laser_type(uint32_t type);
    static uint32_t get_laser_type(const std::string &type);

    static std::string get_camera_type(uint32_t type);
    static uint32_t get_camera_type(const std::string &type);

    static std::string get_carrier_type(uint32_t type);
    static uint32_t get_carrier_type(const std::string &type);

    static std::string get_battery_type(uint32_t type);
    static uint32_t get_battery_type(const std::string &type);

    static std::string get_imu_type(uint32_t type);
    static uint32_t get_imu_type(const std::string &type);

    static std::string get_drv_node(uint32_t type);
    static std::string get_drv_url(uint32_t type);

    static etc_value_t get_drv_default(uint32_t type);
    static etc_value_t get_drv_default(const std::string &module, const std::string &type = kEmptyStr);
    static etc_param_t get_drv_default();
    static bool get_drv_default(etc_value_t obj);

    static bool get_camera_rs_default(etc_value_t obj);
    static bool get_camera_ifm_default(etc_value_t obj);

    //! get xml
    static const std::string get_xml(uint32_t major_type, uint32_t minor_type);
    //! get node arg
    static std::string get_node_arg(const drv_cfg_t param);
    //! get ros arg
    static bool get_ros_arg(ros_arg_t &arg, const drv_cfg_t& cfg);

private:

  //! get node arg
  static bool get_node_arg(drv_arg_type_t type, const std::string &arg, std::string &node);

  //! get ros arg
  static bool get_laser_arg(ros_arg_t &args, const drv_cfg_t &cfg);
  static bool get_camera_arg(ros_arg_t &args, const drv_cfg_t &cfg);

  //! get laser arg
  static bool get_laser2d_arg(ros_arg_t &args, const drv_cfg_t &cfg);
  static bool get_laser3d_arg(ros_arg_t &args, const drv_cfg_t &cfg);

  //! get depth camera arg
  static bool get_realsense_arg(ros_arg_t &args, const drv_cfg_t &cfg);
  static bool get_ifm_arg(ros_arg_t &args, const drv_cfg_t &cfg);
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_DEFAULT__H__
