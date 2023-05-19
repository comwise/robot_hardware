#ifndef __COMWISE_DRV__DRV_JOYSTICK__H__
#define __COMWISE_DRV__DRV_JOYSTICK__H__

#include "drv/drv_impl.h"

namespace comwise {
struct joy_data;
namespace joy {
    class joy_stick;
}
namespace drv {

class drv_joystick : public drv_impl
{
protected:
    using joy_data_t = comwise::joy_data;
    using joy_stick_t = std::shared_ptr<joy::joy_stick>;

public:
    drv_joystick(drv_id_t id, drv_type_t type = comwise::kHwJoystick);
    virtual ~drv_joystick() override;

protected:
    //! update param
    virtual int update(const drv_param_t &cfg, bool is_init = true) override;

    //! start/stop driver
    virtual int _start() override;
    virtual int _stop() override;

    void joy_data(const joy_data_t& data);

private:
    joy_stick_t joy_stick_{nullptr};
};

} // namespace drv
} // namespace comwise
 
#endif // __COMWISE_DRV__DRV_JOYSTICK__H__
