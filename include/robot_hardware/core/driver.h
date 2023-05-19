#ifndef __COMWISE_CORE__DRIVER__H__
#define __COMWISE_CORE__DRIVER__H__

#include <cstdint>
#include <memory>
#include <string>
#include "error.h"
#include "state.h"
#include "type.h"
#include "event.h"

namespace core {

template <typename Param>
class driver_core : public hw_type, public state, public error_code<int>
{
public:
    driver_core(const hw_id_t &id, int type = 0) : hw_type(id, type) { }
    virtual ~driver_core() { }

    //>! init/deinit driver
    virtual int init(const Param &param) { return -1; }
    virtual int deinit() { return -1; }

    //>! start/stop driver
    virtual int start() { return -1; }
    virtual int stop() { return -1; }
};

template <typename Param, typename Request, typename Response>
class driver : public driver_core<Param>
{
public:
    driver(const hw_type::hw_id_t &id, int type = 0) : driver_core<Param>(id, type) { }
    virtual ~driver() { }

    //>! init/deinit driver
    virtual int init(const Param &param) { return -1; }
    virtual int deinit() { return -1; }

    //>! start/stop driver
    virtual int start() { return -1; }
    virtual int stop() { return -1; }

    //>! read/write driver
    virtual int read(const Request &req, Response res) { return -1; }
    virtual int write(const Request &req, Response res) { return -1; }
};

template <typename Param, typename Request, typename Response>
class driver_event : public driver<Param, Request, Response>, public event
{
public:
    driver_event(const hw_type::hw_id_t &id, int type = 0) 
        : driver<Param, Request, Response>(id, type) { }
    virtual ~driver_event() { }
};

} // namespace core
 
#endif // __COMWISE_CORE__DRIVER__H__
