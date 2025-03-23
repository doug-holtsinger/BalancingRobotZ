

#ifndef __MOTOR_DRIVER_CMD_H__
#define __MOTOR_DRIVER_CMD_H__

#include <cstdint>
#include "PIDCmd.h"

enum class MOTOR_DRIVER_CMD_t : uint8_t
{
    NOCMD = 0,
    TOGGLE_POWER,
    TOGGLE_DISPLAY,
    PWM_CLK_UP,
    PWM_CLK_DOWN,
    CMD_MIN = NOCMD,
    CMD_MAX = PWM_CLK_DOWN,
};

#endif


