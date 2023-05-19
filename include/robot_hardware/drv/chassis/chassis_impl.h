#ifndef __COMWISE__CHASSIS_IMPL__H__
#define __COMWISE__CHASSIS_IMPL__H__

#include "chassis_base.h"

namespace std {
    class thread;
}

namespace comwise {
namespace motor {
    class motor_driver;
}
namespace chassis {

class move_model;
class chassis_impl : public chassis_base
{
protected:
    using motor_driver_t = motor::motor_driver;
    using motor_driver_ptr = std::shared_ptr<motor_driver_t>;

public:
    explicit chassis_impl(const chassis_id_t &id, 
                chassis_type_t type = kChassisNone);
    virtual ~chassis_impl();

    //!> init and deinit drvier
    virtual int init(const chassis_param_t &param = nullptr) override;
    virtual int deinit() override;

    //!> start/stop driver
    virtual int start() override;
    virtual int stop() override;

    //!> set/get speed
    virtual int set_speed(const vel_t &vel) override;
    virtual int get_speed(vel_t &vel) override;

protected:
    virtual void work_handler();

protected:
    chassis_param_t param_{nullptr};
    std::map<int, motor_driver_ptr> motor_;
    std::shared_ptr<move_model> move_model_{nullptr};
    MoveModelControl move_control_option_;

    int32_t period_{20};
    std::shared_ptr<std::thread> thr_{nullptr};
    bool is_loop_{false};
    bool start_thread_{false};

    std::chrono::steady_clock::time_point last_time_;

    vel_t cmd_vel_;
    vel_t feeback_vel_;

    pose_t pos_;
};

} // namespace chassis
} // namespace comwise
 
#endif // __COMWISE__CHASSIS_IMPL__H__
