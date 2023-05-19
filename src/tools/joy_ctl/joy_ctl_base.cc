#include "joy_ctl_base.h"
#include <thread>
#include <iostream>

namespace comwise {
namespace joy {

#define SPD_INFO  printf
#define SPD_ERROR printf

joy_ctl_base::joy_ctl_base(const msg_id_t &name, uint32_t type)
    : core::driver_event<msg_cfg_t, msg_data_t, msg_data_t>(name, type)
{
    SPD_INFO("create joy_ctl_base(%s) ok\n", get_id().c_str());
}

joy_ctl_base::~joy_ctl_base()
{
    deinit();
    SPD_INFO("destroy joy_ctl_base(%s) ok\n", get_id().c_str());
}

int joy_ctl_base::init(const msg_cfg_t &cfg)
{
    if (get_state() >= kStateReady && get_state() <= kStatePause) {
        return 0;
    }

    cfg_ = cfg;
    
    set_state(kStateReady);

    SPD_INFO("init app(%s) ok\n", get_id().c_str());

    return 0;
}

int joy_ctl_base::deinit()
{
    _stop();
    set_state(kStateExit);

    SPD_INFO("deinit app(%s) ok\n", get_id().c_str());

    return 0;
}

int joy_ctl_base::start()
{
    int ret = 0;
    work_thr_ = std::move(std::make_shared<std::thread>([&]() {
        is_loop_ = true;
        while (is_loop_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            try {
                switch (get_state()) {
                case kStateIdle:
                {
                    int init_ret = init(cfg_);
                    SPD_INFO("init app(%s) return %d\n", get_id().c_str(), init_ret);
                    if (0 == init_ret) {
                        ret = 0;
                        set_state(kStateReady);
                    } else {
                        ret = -10;
                    }
                    break;
                }
                case kStateReady:
                {
                    SPD_INFO("start app(%s) ...\n", get_id().c_str());
                    int start_ret = _start();
                    if (0 == start_ret) {
                        SPD_INFO("start app(%s) success\n", get_id().c_str());
                        set_state(kStateRunning);
                    } else {
                        SPD_ERROR("start app(%s) error, return = %d\n", get_id().c_str(), start_ret);
                    }
                    ret = start_ret;
                    break;
                }
                case kStateRunning:
                    break;
                case kStatePause:
                case kStateExit:
                {
                    return 0;
                }
                default:
                {
                    SPD_ERROR("app(%s) have unknown status: %d\n", get_id().c_str(), get_state());
                    break;
                }
                }
            } catch (const std::exception &e) {
                SPD_ERROR("app(%s) exception, %s\n", get_id().c_str(), e.what());
                ret = -12;
            }
        }
        ret = 0;
        SPD_INFO("thread(%s) quit loop normal\n", get_id().c_str());
    }));

    SPD_INFO("start thread(%s) success\n", get_id().c_str());

    return ret;
}

int joy_ctl_base::stop()
{
    return _stop();
}

int joy_ctl_base::_start()
{
    return 0;
}

int joy_ctl_base::_stop()
{
    set_state(kStatePause);
    is_loop_ = false;
    if(work_thr_ && work_thr_->joinable()) {
        work_thr_->join();
    }
    work_thr_ = nullptr;
    SPD_INFO("stop app(%s) ok\n", get_id().c_str());
    return 0;
}

int joy_ctl_base::read(const msg_data_t &request, msg_data_t reply)
{
    return -1;
}

int joy_ctl_base::write(const msg_data_t &request, msg_data_t reply)
{
    return -1;
}

void joy_ctl_base::set_joy_data(const joy_data &data)
{

}

} // namespace joy
} // namespace comwise
