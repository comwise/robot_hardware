#ifndef __COMWISE__VAR_COMMON__H__
#define __COMWISE__VAR_COMMON__H__

#include <cstdint>
#include <string>
#include <map>

namespace comwise {

enum hw_type_t
{
    kHwIO,
    kHwMotor,
    kHwChassis,
    kHwIMU,
    kHwLaser,
    kHwCamera,
    kHwBattery,
    kHwUltrasonic,
    kHwJoystick,
    kHwGPS,
    kHwAudio,
    kHwRfid,
    kHwCan,
    kHwNetwork,
    kHwSerial,
    kHwModbus,
    kHwPlc,
    kHwCarrier,
    kHwCalib,
    kHwMax,
};

enum chassis_type_t
{
    kChassisNone     = 0,
    kChassisSteer    = 1,
    kChassisDiff     = 2,
    kChassisSMSR     = 3,
    kChassisMecanum  = 4,
    kChassisMax,
};

enum laser_type_t
{
    kLaserSick1XX,
    kLaserSickTim240,
    kLaserSickNanoscan3,
    kLaserUam05xl,
    kLaserLslidarN301,
    kLaserPfR2000,
    kLaserRslidar,
    kLaserVelodyne,
    kLaserPandar,
    kLaserTypeMax,
};

enum camera_type_t
{
    kCamera2D,
    kCameraRealsense,
    kCameraIfm,
    kCameraMax
};

enum can_type_t
{
    kCanNone    = 0,
    kCanZLG     = 1,
    kCanEMUC    = 2,
    kCanLike    = 3,
    kCanMax,
};

enum motor_type_t
{
    kMotorNone           = 0,
    kMotorTest1,
    kMotorTest2,
    kMotorCurtis,
    kMotorMax,
};

enum battery_type_t 
{
    kBatteryNone    = 0,
    kBatteryTest1   = 1,
    kBatteryTest2   = 2,
    kBatteryMax,
};

enum imu_type_t {
  kIMUNone,
  kIMUSerial,
  kIMUCan,
  kIMUMax,
};

enum io_type_t
{
    kIOCommon,
    kIOCollision,
    kIOPower,
    kIOStop,
    kIOLcd,
    kIOMax,
};

enum carrier_type_t
{
    kCarrierRoller,
    kCarrierLift,
    kCarrierRotate,
    kCarrierMax,
};

enum param_type_t
{
    kParamTypeNone,
    kParamTypeReadConfig,
    kParamTypeWriteConfig,
    kParamTypeReadSpeed,
    kParamTypeWriteSpeed,
    kParamTypeReadData,
    kParamTypeWriteData,
    kParamTypeReadStatus,
    kParamTypeWriteStatus,
};

// common const type define
static const char *kDrvType[] =
{
    "io"        /*kHwIO*/,
    "motor"     /*kHwMotor*/,
    "chassis"   /*kHwChassis*/,
    "imu"       /*kHwIMU*/,
    "laser"     /*kHwLaser*/,
    "camera"    /*kHwCamera*/,
    "battery"   /*kHwBattery*/,
    "ultrasonic"/*kHwUltrasonic*/,
    "joystick"  /*kHwJoystick*/,
    "gps"       /*kHwGPS*/,
    "audio"     /*kHwAudio*/,
    "rfid"      /*kHwRfid*/,
    "can"       /*kHwCan*/,
    "network"   /*kHwNetwork*/,
    "serial"    /*kHwSerial*/,
    "modbus"    /*kHwModbus*/,
    "plc"       /*kHwPlc*/,
    "carrier"   /*kHwCarrier*/,
};

static const char *kCameraType[] =
{
    "2d" /*kCamera2D*/,
    "realsense" /*kCameraRealsense*/,
    "ifm" /*kCameraIfm*/,
};

static const char *kCanType[] =
{
    "",
    "zlg",
    "emuc",
    "like",
};

static const char *kMotorType[] =
{
    ""                 /*kMotorNone*/,
    "test1"            /*kMotorTest1*/,
    "test2"            /*kMotorTest2*/,
    "curtis"           /*kMotorCurtis*/,
    ""                 /**/,
};

static const char *kIOType[] =
{
    "close"     /*kIOCollision*/,
    "pwr"       /*kIOPower*/,
    "stop"      /*kIOStop*/,
    "lcd"       /*kIOLcd*/,
    ""
};

static const char *kChassisType[] =
{
    ""          /**/,
    "steer"     /**/,
    "diff"      /**/,
    "smsr"      /**/,
    "mecanum"   /**/,
};

static const char *kLaserType[] =
{
    "sick_1xx"      /*kLaserSick1XX*/,
    "sick_tim240"   /*kLaserSickTim240*/,
    "sick_nanoscan3" /*kLaserSickNanoscan3*/,
    "uam_05xl"      /*kLaserUam05xl*/,
    "lslidar_n301"  /*kLaserLslidarN301*/,
    "pf_r2000"      /*kLaserPfR2000*/,
    "rslidar"       /*kLaserRslidar*/,
    "velodyne"      /*kLaserVelodyne*/,
    "pandar"         /*kLaserPandar*/,
    ""
};

static const char *kDrvNodeName[] =
{
    ""      /*kHwIO*/,
    ""      /*kHwChassis*/,
    ""      /*kHwLaser*/,
    ""      /*kHwCamera*/,
    ""      /*kHwBattery*/,
    ""      /*kHwJoystick*/,
    ""      /*kHwGPS*/,
    ""      /*kHwAudio*/,
    ""      /*kHwRfid*/,
    ""      /*kHwCan*/,
    ""      /*kHwNetwork*/,
    ""      /*kHwSerial*/,
    ""      /*kHwModbus*/,
    ""      /*kHwPlc*/,
    ""      /*kHwCarrier*/,
};

static const char *kCarriertype[] =
{
    "roller" /*kCarrierRoller*/,
    "lift"   /*kCarrierLift*/,
    "rotate" /*kCarrierRotate*/,
    "",
};

static const char* kBatteryType[] =
{
    ""     ,
    "test1",
    "test2",
};

static const char* kIMUType[] =
{
    "",
    "serial",
    "can",
    "",
};

} // namespace comwise

#endif // __COMWISE__VAR_COMMON__H__
