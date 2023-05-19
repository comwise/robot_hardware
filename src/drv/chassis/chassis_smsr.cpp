#include "chassis/chassis_smsr.h"
#include <cmath>
#include <iostream>
#include "log/log.h"
#include "chassis/chassis_model.h"
#include "motor/motor_factory.h"

namespace comwise {
namespace chassis {

namespace {
// 行走电机移动 1: 正  -1：负
constexpr int kSpeedDirection = 1;
// 转向电机移动 1: 左正右负  -1: 左负右正
constexpr int kAngleDirection = 1;

static const double PI = 3.1415926;
}

using namespace motor;

chassis_smsr::chassis_smsr(const chassis_id_t &id, chassis_type_t type)
    : chassis_impl(id, type)
{

}

chassis_smsr::~chassis_smsr()
{

}

int chassis_smsr::move_cmd(const vel_t &vel)
{
    auto front_left_motor  = MOTOR_FACTORY->find(motor::kMotorFrontLeft);
    auto front_right_motor = MOTOR_FACTORY->find(motor::kMotorFrontRight);
    auto back_left_motor   = MOTOR_FACTORY->find(motor::kMotorBackLeft);
    auto back_right_motor  = MOTOR_FACTORY->find(motor::kMotorBackRight);

    if (!front_left_motor || !front_right_motor ||
        !back_left_motor || !back_right_motor) {
        return -1;
    }

    if (manual_mode_ == false) { // 非手动模式
        if (last_manual_mode_ == true) {
            front_left_motor->SetAccelAndDecelControl(10, 10);
            front_right_motor->SetAccelAndDecelControl(10, 10);
            back_left_motor->SetAccelAndDecelControl(10, 10);
            back_right_motor->SetAccelAndDecelControl(10, 10);
            LOGP_WARN("auto mode, set accel 10s, decel 10s");
        }
        last_manual_mode_ = manual_mode_;
    } else { // 手动模式
        if (last_manual_mode_ == false) {
            front_left_motor->SetAccelAndDecelControl(30, 30);
            front_right_motor->SetAccelAndDecelControl(30, 30);
            back_left_motor->SetAccelAndDecelControl(30, 30);
            back_right_motor->SetAccelAndDecelControl(30, 30);
            LOGP_WARN("manul mode, set accel 80s, decel 100s");
        }
        last_manual_mode_ = true;
    }

    std::tuple<double, double> cmd_vel;
    if (move_model_) {
        cmd_vel = move_model_->set_move(vel);
    }
    front_left_motor->set_control(std::get<0>(cmd_vel));
    front_right_motor->set_control(std::get<1>(cmd_vel));
    back_left_motor->set_control(std::get<0>(cmd_vel));
    back_right_motor->set_control(std::get<1>(cmd_vel));

    return 0;
}

int chassis_smsr::move_feedback(vel_t &vel)
{
    auto front_left_motor  = MOTOR_FACTORY->find(motor::kMotorFrontLeft);
    auto front_right_motor = MOTOR_FACTORY->find(motor::kMotorFrontRight);
    auto back_left_motor   = MOTOR_FACTORY->find(motor::kMotorBackLeft);
    auto back_right_motor  = MOTOR_FACTORY->find(motor::kMotorBackRight);

    motor_data_t fl_data;
    motor_data_t fr_data;
    motor_data_t bl_data;
    motor_data_t br_data;

    if (!front_left_motor || !front_right_motor ||
        !back_left_motor || !back_right_motor) {
        return -1;
    }

    auto state_cb = [&](std::shared_ptr<motor_driver_t> motor,
                        motor_data_t &data, uint8_t state_bit) {
        if (!motor) {
            return;
        }
        if (motor->is_ready()) {
            motor->get_data(&data);
            LOGP_DEBUG("motor() current: %d, rpm: %d, status(%d)",
                       data.current, data.speed, data.status);
            auto code = get_code();
            set_code(code &= (~1 << state_bit), "motor error");
        } else {
            // 48V低功耗不打印
            if (!(power_mode_ & 0x01)) {
                LOGP_ERROR("[feedback] motor() error code = %d", 0);
            }
            auto code = get_code(); 
            set_code(code &= (~1 << state_bit), "motor is not ready");
        }
    };

    state_cb(front_left_motor,  fl_data, 1 << kMotorFrontLeft);
    state_cb(front_right_motor, fr_data, 1 << kMotorFrontRight);
    state_cb(back_left_motor,   bl_data, 1 << kMotorBackLeft);
    state_cb(back_right_motor,  br_data, 1 << kMotorBackRight);

    vel_t feedback_vel;
    if (move_model_) {
        feedback_vel = move_model_->get_move(fl_data.speed, fr_data.speed);
    }

    vel.v  = kSpeedDirection * feedback_vel.v;
    vel.vv = kSpeedDirection * feedback_vel.vv;
    vel.w  = kAngleDirection * feedback_vel.w;

#ifdef DEBUG_SMSR_DATA
    printf("[feedback] velocity: %lf, omega: %lf", vel.v, vel.w);
#endif

    return 0;
}

int chassis_smsr::move_odom(const vel_t &cmd, double delta_t)
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

void chassis_smsr::work_handler()
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

        emit("vel_feedback", feeback_vel_);
        emit("odom_feedback", pos_);
    }
}

} // namespace chassis
} // namespace comwise