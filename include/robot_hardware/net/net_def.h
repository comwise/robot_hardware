/*!
  \file     net_def.h
  \brief    nef define
  \author   lichanglin
  \version  v1.0
*/
#ifndef __COMWISE_NET__NET_DEF__H__
#define __COMWISE_NET__NET_DEF__H__

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__)
#define NET_WINDOWS_OS

#if defined(_MSC_VER)
#define NET_MSC
#endif

#elif defined(__linux__)
#define NET_LINUX_OS

#else
#define NET_MAC_OS
#endif

#if defined __cplusplus
#include <cstdint>
#define STD_C extern "C"
#else
#include <stdint.h>
#define STD_C
#endif

#if !defined STD_CALL
#if WIN32
#define STD_CALL __stdcall
#else
#define STD_CALL
#endif
#endif

#define net_interface (_Ty) STD_C _Ty STD_CALL

typedef enum _net_protoc_t {
  NET_PROTOC_NONE,
  NET_PROTOC_TCP,
  NET_PROTOC_UDP,
} net_protoc_t;

typedef enum net_event_def_t {
  EVENT_CREATED             = 0x0001,     /* created */
  EVENT_PRE_CLOSE           = 0x0002,     /* will close */
  EVENT_CLOSED              = 0x0003,     /* closed */
  EVENT_SENDDATA            = 0x0004,     /* send data */
  EVENT_RECEIVEDATA         = 0x0005,     /* receive data */
  EVENT_EXCEPTION           = 0xFFFF,     /* exception */

  EVENT_TCP_ACCEPTED        = 0x0013,     /* accepted */
  EVENT_TCP_CONNECTED       = 0x0014,     /* connected */
  EVENT_TCP_DISCONNECTED    = 0x0015,     /* disconnected(host disconnect auto) */

} net_event_def_t;

typedef uint32_t net_link_t;
typedef struct _net_event_t {
    int event; 
    union {
        net_link_t tcp;
        net_link_t udp;
    } link;
} net_event_t;
typedef void( *net_callback_t)(const net_event_t *net_event, const void *param);

#endif // __COMWISE_NET__NET_DEF__H__
