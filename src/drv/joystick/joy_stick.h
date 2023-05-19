#ifndef __COMWISE_JOY__JOY_STICK__H__
#define __COMWISE_JOY__JOY_STICK__H__

#include <cstdint>
#include <atomic>
#include <memory>
#include <vector>
#include <functional>
#include "var/joy_data.h"

struct js_event;

namespace std {
    class thread;
}

namespace comwise {
namespace joy {

class joy_stick
{
public:
    using joy_data = comwise::joy_data;
    using data_cb_t = std::function<void(const joy_data&)>;
public:
    joy_stick(const std::string &dev,
        int coalesce_interval = 20/*ms*/,
        int repeat_interval = 20/*ms*/);
    virtual ~joy_stick();

    bool start();
    bool stop();

    void set_config(const std::string &dev,int coalesce = 20, int repeat = 20);
    void set_config(const joy_config &cfg);
    void get_config(joy_config &cfg);

    void set_deadzone(double x = 0.30);
    double get_deadzone();
    double get_orignal_deadzone();

    void set_data_callback(data_cb_t cb);

private:
    bool open_joy();
    bool close_joy();

    void worker();

    bool handler(const js_event &event, bool &update_now, bool &update_soon);

    double get_scale_data();

private:
    std::string dev_name_; //device name
    double deadzone_{0.03}; //percentage
    double coalesce_interval_{0.02}; //s
    double autorepeat_rate_{50.0}; //hz
    int joy_fd_{0};
    int fail_count_{0}; // failed count

    std::unique_ptr<std::thread> joy_thr_{nullptr};
    std::atomic<bool> is_run_{false};

    joy_data joy_msg_;
    data_cb_t data_cb_{nullptr};
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_STICK__H__
