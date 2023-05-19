#include "joy_stick.h"
#include <fcntl.h>
#include <zconf.h>
#include <thread>
#include <linux/joystick.h>
#include "common/make_thread.h"
#include "log/log.h"

namespace comwise {
namespace joy {

joy_stick::joy_stick(const std::string &dev_name_,
    int coalesce_interval, int repeat_interval)
    : dev_name_(dev_name_)
    , joy_fd_(0)
    , fail_count_(0)
    , joy_thr_(nullptr)
    , is_run_(false)
    , deadzone_(0.03)
    , coalesce_interval_(coalesce_interval/1000.0)
    , autorepeat_rate_(1000.0 / repeat_interval)
{

}

joy_stick::~joy_stick()
{
    stop();
}

void joy_stick::set_config(const std::string &dev,int coalesce, int repeat)
{
    dev_name_ = dev;
    coalesce_interval_ = coalesce/1000.0;
    autorepeat_rate_ = 1000.0/repeat;
}

void joy_stick::set_config(const joy_config &cfg)
{
    dev_name_ = cfg.device;
    coalesce_interval_ = cfg.coalesce_interval/1000.0;
    autorepeat_rate_ = 1000.0/cfg.repeat_interval;
}

void joy_stick::get_config(joy_config &cfg)
{
    cfg.device = dev_name_;
    cfg.repeat_interval = round(1000.0/autorepeat_rate_);
    cfg.coalesce_interval = round(1000.0*coalesce_interval_);

}

void joy_stick::set_deadzone(double deadzone)
{
    deadzone_ = deadzone;
}

double joy_stick::get_deadzone()
{
    return deadzone_;
}

double joy_stick::get_orignal_deadzone()
{ 
    return 32767.0 * deadzone_; 
}

double joy_stick::get_scale_data()
{ 
    return -1.0 / ((1.0 - deadzone_) * 32767.0); 
}

void joy_stick::set_data_callback(data_cb_t cb)
{
    data_cb_ = cb;
}

bool joy_stick::start()
{
    stop();
    joy_thr_ = std::move(common::make_thread("joy_worker", [&]() {
        is_run_ = true;
        worker();
    }));
    LOGP_INFO("start joystick thread ok");
    return true;
}

bool joy_stick::stop()
{
    is_run_ = false;
    if(joy_thr_ && joy_thr_->joinable()) {
        joy_thr_->join();
    }
    joy_thr_ = nullptr;
    close_joy();
    LOGP_INFO("stop joystick thread ok");
    return true;
}

void joy_stick::worker()
{
    fd_set set;
    js_event event = {};
    struct timeval tv = {};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    bool is_tv_set_ok = false;
    bool update_pending = false;
    while (is_run_) {
        bool is_open = open_joy();
        if (is_open) {
            LOGP_WARN("open joy succeed");
            break;
        } else {
            LOGP_WARN("open joy failed");
        }
    }

    unsigned int last_sec = 0, cur_sec;
    while (is_run_)
    {
        bool update_now = false;
        bool update_soon = false;

        FD_ZERO(&set);
        FD_SET(joy_fd_, &set);
        int select_out = select(joy_fd_ + 1, &set, NULL, NULL, &tv);
        if (select_out == -1) {
            tv.tv_sec = 0;
            tv.tv_usec = 0;
            continue;
        }

        if (FD_ISSET(joy_fd_, &set)) {
            ssize_t nread;
            if ((nread = read(joy_fd_, &event, sizeof(js_event))) == -1 && errno != EAGAIN) {
                LOGP_WARN("read joy data failed, joystick is probadly close");
                worker();
            }

            if (nread != sizeof(js_event)) {
                LOGP_WARN("unexpected bytes from joystick");
                continue;
            }
            handler(event, update_now, update_soon);
        } else if (is_tv_set_ok) { 
            // Assume that the timer has expired.
            update_now = true;
        }

        if (update_now) {
            // Assume that all the JS_EVENT_INIT messages have arrived already.
            // This should be the case as the kernel sends them along as soon as
            // the device opens.
            if (data_cb_) {
                data_cb_(joy_msg_);
            }

            update_now = false;
            update_soon = false;
            is_tv_set_ok = false;
            update_pending = false;

            auto cur_sec = std::chrono::system_clock::now().time_since_epoch().count();
            if ((cur_sec - last_sec >= 5) || (last_sec == 0))
            {
                last_sec = cur_sec;
            }
        }

        // If an axis event occurred, start a timer to combine with other events.
        if (!update_pending && update_soon)
        {
            //20ms
            tv.tv_sec = trunc(coalesce_interval_);
            tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
            update_pending = true;
            is_tv_set_ok = true;
        }

        // If nothing is going on, start a timer to do autorepeat.
        if (!is_tv_set_ok && autorepeat_rate_ > 0)
        {
            //20ms
            double autorepeat_interval = 1.0 / autorepeat_rate_;
            tv.tv_sec = trunc(autorepeat_interval);
            tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
            is_tv_set_ok = true;
            //ROS_INFO("Autorepeat pending... %i %i", tv.tv_sec, tv.tv_usec);
        }

        if (!is_tv_set_ok) {
            //1s
            tv.tv_sec = 1;
            tv.tv_usec = 0;
        }
    }
    close_joy();
}

bool joy_stick::open_joy()
{
    while (is_run_) {
        joy_fd_ = open(dev_name_.c_str(), O_RDONLY | O_NONBLOCK);
        if (joy_fd_ != -1) {
            // There seems to be a bug in the driver
            // where the initial events about initial state of the
            // joystick are not the values of the joystick when it was opened
            // but rather the values of the joystick when it was last closed.
            // Opening then closing and opening again is a hack to get more
            // accurate initial state data.
            close(joy_fd_);
            joy_fd_ = open(dev_name_.c_str(), O_RDONLY | O_NONBLOCK);
        }
        if (joy_fd_ != -1) {
            LOGP_INFO("open joystick ok");
            fail_count_ = 0;
            return true;
        }
        if (fail_count_++ > 10) {
            LOGP_WARN("open joy stick failed");
            fail_count_ = 0;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds((fail_count_ + 1) * 100));
    }
}

bool joy_stick::close_joy()
{
    close(joy_fd_);
    return true;
}

bool joy_stick::handler(const js_event &event, bool &update_now, bool &update_soon)
{
    bool ret = true;
    joy_msg_.time = event.time;
    uint8_t type = event.type & ~JS_EVENT_INIT;
    switch (type) {
    case JS_EVENT_BUTTON:
    {
        if (event.number+1 >= joy_msg_.buttons.size()) {
            joy_msg_.buttons.resize(event.number+1);
        }
        joy_msg_.buttons[event.number] = event.value ? 1 : 0;
        update_now = true;
        break;
    }
    case JS_EVENT_AXIS:
    {
        if (event.number+1 >= joy_msg_.axes.size()) {
            joy_msg_.axes.resize(event.number+1);
        }
        double val = event.value;
        // allow deadzone to be "smooth"
        double deadzone_val = get_orignal_deadzone();
        if (val > deadzone_val) {
            val -= deadzone_val;
        } else if (val < -1*deadzone_val) {
            val += deadzone_val;
        } else {
            val = 0;
        }
        double scale_val = get_scale_data();
        double new_val = val*scale_val;
#ifdef JOY_DEBUG  
        std::cout << "axis:(" << (int)event.number << "," << event.value << "), const(" 
            << deadzone_val << "," << scale_val << "," << new_val << ")"<< std::endl;
#endif
        joy_msg_.axes[event.number] = new_val;
        update_soon = true;
        break;
    }
    default:
    {
        LOGP_WARN("Unknown event type(%d)\n", type);
        ret = false;
        break;
    }
    }
    return ret;
}

} // namespace joy
} // namespace comwise