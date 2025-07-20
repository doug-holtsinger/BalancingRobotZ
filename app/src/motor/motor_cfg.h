
#ifndef __MOTOR_CFG__H
#define __MOTOR_CFG__H

typedef int16_t pid_ctrl_t;
typedef uint16_t pwm_top_value_t;
typedef uint16_t pwm_seq_value_t;


// disable the motor past this Roll angle
#if 1
constexpr float MOTOR_DISABLE_ROLL_ANGLE = 25.0;
#else
constexpr float MOTOR_DISABLE_ROLL_ANGLE = 999.0;
#endif
constexpr pwm_top_value_t MOTOR_DRIVER_TOP_VALUE = 8192;
constexpr pwm_top_value_t MOTOR_DRIVER_MAX_VALUE = 3500;

#endif
