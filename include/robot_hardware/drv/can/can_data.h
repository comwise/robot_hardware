#ifndef __COMWISE_CAN__CAN_DATA__H__
#define __COMWISE_CAN__CAN_DATA__H__

#include <cstdint>
#include <memory>
#include <string>

namespace comwise {
namespace can {

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

enum can_enable_t {
  kCanEnableNone = 0,                               // 0-不使用can
  kCanEnableCan0 = 1<<kCanChannel0,                 // 1-使用CAN0
  kCanEnableCan1 = 1<<kCanChannel1,                 // 2-使用CAN1
  kCanEnableBoth = kCanEnableCan0 | kCanEnableCan1, // 3-同时使用CAN0 CAN1
};

enum can_send_type_t {
  kCanSendNormal    = 0, // 0-正常发送
  kCanSendTSingle   = 1, // 1-单次发送
  kCanSendTR        = 2, // 2-自发自收
  kCanSendTRSingle  = 3, // 3-单次自发自收
};

enum can_debug_t {
  CAN_NO_DEV_DEBUG    = 0x01, // no can device debug
  CAN_T_DATA_DEBUG    = 0x02, // can transmit data debug
  CAN_R_DATA_DEBUG    = 0x04, // can read data debug
  CAN_ERROR_DEBUG     = 0x08, // can error debug
  CAN_TIMESTAMP_DEBUG = 0x10, // can timestamp debug
};

struct can_frame_t {
  uint32_t id;          // frame id
  uint32_t time_stamp;  // time stamp
  uint8_t time_flag;    // time flag
  uint8_t send_type;    // send type(can_send_type_t)
  uint8_t remote_flag;  // remote frame
  uint8_t extern_flag;  // extern frame
  uint8_t size;         // data size
  uint8_t data[8];      // data
  uint8_t reserved[3];  // reserved data
  uint8_t channel;      // channel
};

struct can_param_t {
  uint8_t  type{0};              // can type(can_type_t): 0-none 1-zlg 2-emuc 3-like
  uint8_t  channel{0};           // can channel(can_enable_t): 0-none 1-can0 2-can1 3-both
  uint32_t can0_baudrate{125};   // can0 baudrate
  uint32_t can1_baudrate{1000};  // can1 baudrate
  uint32_t timeout{20};          // timeout(ms)
  uint32_t debug{0};             // debug flag
};

struct node_param_t {
  int32_t node_id{0};             // node id
  int32_t channel{0};             // can channel(can_channel_t): 0-can0 1-can1
  int32_t mode{0};                // mode
};

} // namespace can
} // namespace comwise

#endif  // __COMWISE_CAN__CAN_DATA__H__
