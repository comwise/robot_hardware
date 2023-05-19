#include "chassis/chassis_steer.h"
#include <cmath>
#include "log/log.h"
#include "chassis/chassis_model.h"
#include "motor/motor_factory.h"

namespace comwise {
namespace chassis {

struct curtis_data_t { };

chassis_steer::chassis_steer(const chassis_id_t &id, chassis_type_t type)
    : chassis_impl(id, type)
{

}

chassis_steer::~chassis_steer()
{
    deinit();
}

int chassis_steer::move_cmd(const vel_t &vel)
{
    auto curtis_obj = MOTOR_FACTORY->find(motor::kMotorCurtis);
    if (nullptr == curtis_obj) {
        return 0;
    }

    curtis_obj->SetHeartBeat();
    //curtis_obj->SetConfig(curtis::kAcceleration, curtis::kDeceleration,
    //                   curtis::kSteeringLimit);

    std::tuple<double, double> cmd_move{0.0, 0.0};
    if (move_model_) {
        cmd_move = move_model_->set_move(vel);
    }
    //curtis_obj->SetControl(
    //    std::get<0>(cmd_move), std::get<1>(cmd_move),
    //    curtis::AtomicActionType::NONE);
    
    return 0;
}

int chassis_steer::move_feedback(vel_t &vel)
{
    auto curtis_obj = MOTOR_FACTORY->find(motor::kMotorCurtis);
    curtis_data_t data;
    if (curtis_obj && curtis_obj->is_ready()) {
        curtis_obj->get_data(&data);
    } else {
        //data.actual_driver_velocity = 0.0;
        //data.actual_steering_angle = 0.0;
    }

    //vel.v = data.actual_driver_velocity;
    //vel.angle = data.actual_steering_angle;

    return 0;
}

int chassis_steer::move_odom(const vel_t &cmd, double delta_t)
{
    double delta_angle = common::normal_theta(cmd.w * delta_t);
    double angle = pos_.angle + delta_angle;
    angle = common::normal_theta(angle);

    double delta_x = cmd.v * delta_t * cos(angle);
    double delta_y = cmd.v * delta_t * sin(angle);

    pos_.x += delta_x;
    pos_.y += delta_y;
    pos_.angle = angle;

#ifdef ODOM_DEBUG
    std::cout << "v(" << cmd.v<< ", " << cmd.w*180/PI
        << ") delta(" << delta_x << ", " << delta_y << ", " << delta_angle*180/PI
        << ") pos(" << pos_.x << ", " << pos_.y << ", " << pos_.angle*180/PI 
        << ")" << std::endl;
#endif
    return 0;
}

void chassis_steer::work_handler()
{
    while(is_loop_) {

        std::this_thread::sleep_for(std::chrono::milliseconds(period_));
        if (!start_thread_) {
            continue;
        }

        vel_t cmd(cmd_vel_);
        auto now = std::chrono::steady_clock::now();
        double delta_t = std::chrono::duration_cast<
            std::chrono::milliseconds>(now - last_time_).count();
        if (delta_t > 100) {
            cmd = vel_t();
        }

        move_cmd(cmd);
        move_feedback(feeback_vel_);
    }
}

} // namespace chassis
} // namespace comwise
