#include "joy_ctl_forklift.h"
#include <cmath>

namespace comwise {
namespace joy {

namespace {
    static constexpr double kScaleLiner{0.5};
    static constexpr double kScaleLinerBoost{1.2};
    static constexpr double kPi{3.1415926};
    static constexpr double kScaleAxesLimit{0.2};
}

joy_ctl_forklift::joy_ctl_forklift(const std::string &name)
    : joy_ctl_base(name)
    , is_send_{false}
{

}

joy_ctl_forklift::~joy_ctl_forklift()
{
    deinit();
}

void joy_ctl_forklift::set_joy_data(const joy_data &joy)
{
    if(joy.axes.size() < 8 || joy.buttons.size() < 8) {
        return;
    }
    bool btn_brake   = joy.buttons[BUTTON_B];
    bool low_enable  = joy.axes[AXES_BUTTON_LT] < -0.5;
    bool high_enable = joy.axes[AXES_BUTTON_RT] < -0.5;
    float axes_lx    = joy.axes[AXES_LEFT_STICK_UD];
    float axes_ly    = joy.axes[AXES_RIGHT_STICK_LR];
    bool btn_enable  = low_enable || high_enable;

    vel_cmd_t cmd_vel;
    cmd_vel.type     = kChassisSteer;
    cmd_vel.priority = kVelPriorityJoystick;
    cmd_vel.vel = vel_list_t(1, vel_t());
    if (btn_brake) {
        emit("move_brake", cmd_vel);
        is_send_ = false;
    } else if (btn_enable) {
        double ax = axes_lx * kScaleLiner;
        double ay = axes_ly * kPi/2;
        ax = high_enable? 
            axes_lx * kScaleLinerBoost : axes_lx * kScaleLiner;

#if 0
        if (std::fabs(ax) > kScaleAxesLimit) {
            if (ax > kScaleAxesLimit) {
                ay = ay;
            } else if (ax < -1 * kScaleAxesLimit) {
                ay *= -1;
            } else {
                ay = 0;
            }
        }
#endif

        cmd_vel.vel[0].v = ax;
        cmd_vel.vel[0].w = ay;
        cmd_vel.vel[0].angle = 0;

        printf("(v, w) = %lf, %lf\n", ax, ay);

        emit("move_cmd", cmd_vel);
        is_send_ = false;
    } else {
        // When enable button is released, immediately send brake command
        // in order to stop the robot
        if (!is_send_) {
            cmd_vel.vel[0] = vel_t();
            emit("move_cmd", cmd_vel);
            is_send_ = true;
        }
    }
}

} // namespace joy
} // namespace comwise