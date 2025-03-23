
#ifndef __PID_CMD__H
#define __PID_CMD__H

enum class PID_CMD_t : uint8_t
{
    NOCMD = 0,
    CMD_MIN = NOCMD, 
    PID_KP_UP,
    PID_KP_DOWN,
    PID_KI_UP,
    PID_KI_DOWN,
    PID_KD_UP,
    PID_KD_DOWN,
    PID_SP_UP,
    PID_SP_DOWN,
    PID_PARAMS_SAVE,
    PID_PARAMS_ERASE,
    CMD_MAX = PID_PARAMS_ERASE
};


#endif
