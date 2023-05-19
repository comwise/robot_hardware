#include "chassis/chassis_diff.h"
#include <cmath>
#include <iostream>
#include "common/algo.h"
#include "motor/motor_factory.h"
#include "chassis/chassis_model.h"

namespace comwise {
namespace chassis {

namespace {

static constexpr double PI = 3.1415926;

//#define DEBUG_DIFF_DATA
//#define DEBUG_DIFF_STATE
//#define DEBUG_DIFF_ODOM

}

using namespace motor;

chassis_diff::chassis_diff(const chassis_id_t &id, chassis_type_t type)
    : chassis_impl(id, type)
{

}

chassis_diff::~chassis_diff()
{
    deinit();
}

int chassis_diff::move_cmd(const vel_t &vel)
{
    auto left_motor = MOTOR_FACTORY->find(kMotorLeft);
    auto right_motor = MOTOR_FACTORY->find(kMotorRight);
    if (!left_motor || !right_motor) {
        return -1;
    }
    if (left_motor->get_state() != kStateRunning ||
        right_motor->get_state() != kStateRunning ) {
        return -2;
    }
    if (manual_mode_ == false) { // auto mode
        if (last_manual_mode_) {
            left_motor->SetAccelAndDecelControl(100, 80);
            right_motor->SetAccelAndDecelControl(100, 80);
        }
    } else { // manual mode
        if (!last_manual_mode_) {
            left_motor->SetAccelAndDecelControl(800, 1000);
            right_motor->SetAccelAndDecelControl(800, 1000);
        }
    }
    last_manual_mode_ = manual_mode_;

    std::tuple<double, double> cmd_move{0.0, 0.0};
    if (move_model_) {
        cmd_move = move_model_->set_move(vel);
    }

    left_motor->set_control(std::get<0>(cmd_move));
    right_motor->set_control(std::get<1>(cmd_move));

#ifdef DEBUG_DIFF_DATA
    printf("[move_cmd] (v, w) = (%lf, %lf)->(v1, v2)=(%lf, %lf)\n",
        vel.v, vel.w, std::get<0>(cmd_move), std::get<1>(cmd_move));
#endif

    return 0;
}

int chassis_diff::move_feedback(vel_t &vel)
{
    auto left_motor = MOTOR_FACTORY->find(kMotorLeft);
    auto right_motor = MOTOR_FACTORY->find(kMotorRight);
    if (!left_motor || !right_motor) {
        return -1;
    }

    motor_data_t left_data, right_data;
    if (left_motor->get_state() >= kStateRunning) {
        left_motor->get_data(&left_data);
#ifdef DEBUG_DIFF_STATE
        printf("left motor current: %d, rpm: %d\n",
                left_data.current, left_data.speed);
#endif
        set_code(get_code() & (~1 << kMotorLeft));
    } else {
        LOGP_ERROR("left motor error, code = %d(%s)", 
                left_motor->get_code(), left_motor->get_detail().c_str());
        set_code(get_code() & (1 << kMotorLeft));
    }
    if (right_motor->get_state() >= kStateRunning) {
        right_motor->get_data(&right_data);
#ifdef DEBUG_DIFF_STATE
        printf("left motor current: %d, rpm: %d\n",
                right_data.current, right_data.speed);
#endif
        set_code(get_code() & (~1 << kMotorRight));
    } else {
        LOGP_ERROR("right motor error, code = %d(%s)",
                right_motor->get_code(), right_motor->get_detail().c_str());
        set_code(get_code() & (1 << kMotorRight));
    }

    if (move_model_) {
        vel = move_model_->get_move(left_data.speed, right_data.speed);
    }

#ifdef DEBUG_DIFF_DATA
    printf("[feedback] velocity: %lf, omega: %lf\n", vel.v, vel.w);
#endif

    return 0;
}

int chassis_diff::move_odom(const vel_t &cmd, double delta_t)
{
    double delta_angle = common::normal_theta(cmd.w * delta_t);
    double angle = pos_.angle + delta_angle;
    angle = common::normal_theta(angle);

    double delta_x = cmd.v * delta_t * cos(angle);
    double delta_y = cmd.v * delta_t * sin(angle);

    pos_.x += delta_x;
    pos_.y += delta_y;
    pos_.angle = angle;

#ifdef DEBUG_DIFF_ODOM
    std::cout << "v(" << cmd.v<< ", " << cmd.w*180/PI
        << ") delta(" << delta_x << ", " << delta_y << ", " << delta_angle*180/PI
        << ") pos(" << pos_.x << ", " << pos_.y << ", " << pos_.angle*180/PI 
        << ")" << std::endl;
#endif
    return 0;
}

void chassis_diff::work_handler()
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

        emit("move_feedback", feeback_vel_);
    }
}

} // namespace chassis
} // namespace comwise
