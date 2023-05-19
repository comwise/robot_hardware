#ifndef __COMWISE__VAR_CODE__H__
#define __COMWISE__VAR_CODE__H__

namespace comwise { 

enum hw_code_t {

    // common 0 ~ 1000
    RET_OK                                          = 0,
    RET_ERROR                                       = 1,
    RET_EXCEPTION                                   = 2,
    
    RET_OBJECT_IS_NULL                              = 11,
    RET_OBJECT_IS_NOINIT                            = 14,
    RET_OBJECT_IS_INIT                              = RET_OK,
    RET_OBJECT_IS_NO_FOUND                          = 15,

    RET_PARAM_OBJECT_IS_NULL                        = 21,
    RET_PARAM_OBJECT_IS_ERROR                       = 22,
    RET_PARSE_PARAM_ERROR                           = 23,
    RET_PARSE_PARAM_EXCEPTION                       = 24,
    RET_SET_PARAM_ERROR                             = 25,
    RET_SET_CONFIG_ERROR                            = 26,
    RET_SET_NETWORK_ERROR                           = 27,

    RET_INIT_ERROR                                  = 30,
    RET_UNINIT_ERROR                                = 31,
    RET_START_ERROR                                 = 32,
    RET_STOP_ERROR                                  = 33,
    
    RET_RUN_SUB_PROCESS_ERROR                       = 50,
    RET_RUN_THREAD_ERROR                            = 51,
    RET_RUN_THREAD_EXCEPTION                        = 52,
    RET_WAITING_FOR_END_PROCESS_ERROR               = 53,
    RET_STOP_SUB_PROCESS_ERROR                      = 54,
    RET_EXIT_SUB_PROCESS_ERROR                      = 55,

    // drv_chassis
    RET_DRV_CHASSIS_INIT_PARAM_OBJECT_IS_NULL       = 200,
    RET_DRV_CHASSIS_INIT_IMP_OBJECT_IS_NULL         = 201,
    RET_DRV_CHASSIS_INIT_IMP_OBJECT_ERROR           = 202,
    RET_DRV_CHASSIS_START_THREAD_ERROR              = 210,
    RET_DRV_CHASSIS_START_THREAD_EXCEPTION          = 211,
    RET_DRV_CHASSIS_READ_REQUEST_OBJECT_IS_NULL     = 220,
    RET_DRV_CHASSIS_READ_REPLY_OBJECT_IS_NULL       = 221,
    RET_DRV_CHASSIS_WRITE_REQUEST_OBJECT_IS_NULL    = 230,
    RET_DRV_CHASSIS_WRITE_REPLY_OBJECT_IS_NULL      = 231,
    
    // drv_laser
    RET_DRV_LASER_INIT_CREATE_LASER_OBJECT_ERROR    = 300,
    RET_DRV_LASER_INIT_SET_CONFIG_ERROR             = 301,
    RET_DRV_LASER_START_CONFIG_OBJECT_IS_NULL       = 310,
    RET_DRV_LASER_START_CREATE_ROS_OBJECT_ERROR     = 311,
    RET_DRV_LASER_START_SET_XML_ARG_ERROR           = 312,
    RET_DRV_LASER_START_RUN_ROSLAUNCH_ERROR         = 313,
    RET_DRV_LASER_START_RUN_ROS_SERVICE_ERROR       = 314,
    RET_DRV_LASER_START_EXIT_ROS_NODE_ERROR         = 315,
    RET_DRV_LASER_START_THREAD_ERROR                = 320,
    RET_DRV_LASER_START_THREAD_EXCEPTION            = 321,

    // hardware_interface
    RET_SRV_HW_INTERFACE_PARAM_OBJECT_ERROR         = 500,
    RET_SRV_HW_INTERFACE_GET_OBJECT_ERROR           = 501,
    RET_SRV_HW_INTERFACE_READ_ERROR                 = 502,
    RET_SRV_HW_INTERFACE_WRITE_ERROR                = 503,

    // runner_service
    RET_SRV_HW_SERVICE_INIT_PARAM_OBJECT_IS_NULL    = 600,
    RET_SRV_HW_SERVICE_INIT_SUB_PARAM_OBJECT_IS_NULL = 601,
    RET_SRV_HW_SERVICE_INIT_CREATE_DRV_OBJECT_ERROR = 602,
    RET_SRV_HW_SERVICE_INIT_DRV_OBJECT_ERROR        = 603,
    RET_SRV_HW_SERVICE_GET_DRV_OBJECT_IS_NULL       = 610,
    RET_SRV_HW_SERVICE_GET_MAJOR_TYPE_ERROR         = 611,
    RET_SRV_HW_SERVICE_START_OBJECT_ERROR           = 620,
    RET_SRV_HW_SERVICE_UNINIT_OBJECT_ERROR          = 630,

    // runner_manager
    RET_SRV_HW_MANAGER_CREATE_OBJECT_ERROR          = 700,
    RET_SRV_HW_MANAGER_INIT_OBJECT_ERROR            = 701,

    // etc_calib
    RET_ETC_ETC_CALIB_INIT_ERROR                    = 800,
    RET_ETC_ETC_CALIB_UNINIT_ERROR                  = 801,
    RET_ETC_ETC_CALIB_OBJECT_IS_NULL                = 802,
    RET_ETC_ETC_PARSE_JSON_ERROR                    = 820,
    RET_ETC_ETC_PARSE_SUB_OBJECT_ERROR              = 821,
    RET_ETC_ETC_PARSE_JSON_EXCEPTION                = 822,
    RET_ETC_ETC_PARSE_WRITE_JSON_TO_FILE_ERROR      = 830,
    RET_ETC_ETC_PARSE_UPDATE_CALIB_PRAM_ERROR       = 831,

};

} // namespace comwise

#endif // __COMWISE__VAR_CODE__H__
