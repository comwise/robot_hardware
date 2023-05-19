

#ifndef __COMWISE_JOY__JOY_KEY__H__
#define __COMWISE_JOY__JOY_KEY__H__


namespace comwise {
namespace joy {

enum AxesKey {
    AXES_LEFT_STICK_LR  = 0,
    AXES_LEFT_STICK_UD  = 1,
    AXES_BUTTON_LT      = 2,
    AXES_RIGHT_STICK_LR = 3,
    AXES_RIGHT_STICK_UD = 4,
    AXES_BUTTON_RT      = 5,
    AXES_LEFT_BUTTON_LR = 6,
    AXES_LEFT_BUTTON_UD = 7,
};

enum ButtonKey {
    BUTTON_A            = 0,
    BUTTON_B            = 1,
    BUTTON_X            = 2,
    BUTTON_Y            = 3,
    BUTTON_LB           = 4,
    BUTTON_RB           = 5,
    BUTTON_BACK         = 6,
    BUTTON_START        = 7,
    BUTTON_POWER        = 8,
    BUTTON_OTHER1       = 9,
    BUTTON_OTHER2       = 10,
};

enum Button3DKey {
    kButtonKeyLevel1    = 0,
    kButtonKeyLevel2    = 1,
    kButtonKeyLevel3    = 2,
    kButtonKey1         = 3,
    kButtonKey2         = 4,
    kButtonKey3         = 5,
    kButtonKey4         = 6,
    kButtonKey5         = 7,
    kButtonKey6         = 8,
    kButtonKey7         = 9,
    kButtonKey8         = 10,
    kButtonKeyWin       = 11,
    kButtonKey9         = 12,
    kButtonKey0         = 13,
    kButtonKeyCamera    = 14,
};

} // namespace joy
} // namespace comwise

#endif //__COMWISE_JOY__JOY_KEY__H__
