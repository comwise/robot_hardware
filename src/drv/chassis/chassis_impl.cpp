#include "chassis/chassis_impl.h"
#include <thread>
#include "log/log.h"
#include "chassis/chassis_model.h"

namespace comwise {
namespace chassis {

chassis_impl::chassis_impl(const chassis_id_t &id, chassis_type_t type)
    : chassis_base(id, type)
{

}

chassis_impl::~chassis_impl()
{
    deinit();
}

int chassis_impl::init(const chassis_param_t &param)
{
    param_ = param;
    if (param_) {
        move_control_option_ = param_->move_option.movemode_control_option;
        move_model_ = move_model_factory::create_model(get_type(), param_->move_option);
    }

    is_loop_ = true;
    thr_ = std::make_shared<std::thread>([&]() {
        work_handler();
    });
    return 0;
}

int chassis_impl::deinit()
{
    is_loop_ = false;
    if(thr_ && thr_->joinable()) {
        thr_->join();
    }
    thr_ = nullptr;
    return 0;
}

int chassis_impl::start()
{
    last_time_ = std::chrono::steady_clock::now();
    start_thread_ = true;
    return 0;
}

int chassis_impl::stop()
{
    start_thread_ = false;
    return 0;
}

void chassis_impl::work_handler()
{
    while (is_loop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(period_));
        if (!start_thread_) {
            continue;
        }
    }
}

int chassis_impl::set_speed(const vel_t &vel)
{
    cmd_vel_ = vel;
    last_time_ = std::chrono::steady_clock::now();
    return 0;
}

int chassis_impl::get_speed(vel_t &vel)
{
    vel = feeback_vel_;
    return 0;
}

} // namespace chassis
} // namespace comwise
