
#ifndef __MOTOR_DRIVER__H
#define __MOTOR_DRIVER__H

#include "pid.h"
#include "motorcmd.h"

typedef int16_t pid_ctrl_t;
typedef uint16_t pwm_top_value_t;
typedef uint16_t pwm_seq_value_t;

constexpr pwm_top_value_t MOTOR_DRIVER_TOP_VALUE = 8192;
constexpr int16_t PID_CONTROL_SETTING_MAX = MOTOR_DRIVER_TOP_VALUE-1;
constexpr uint16_t MOTOR_CONTROL_SETTING_MASK = 0x7FFF;
// Setpoint default of PID controller
constexpr float MOTOR_DRIVER_SP_DEFAULT = 0.0;
constexpr pwm_seq_value_t PWM_POL_FALLING_EDGE = 0x8000;
constexpr pwm_seq_value_t PWM_POL_RISING_EDGE = 0x0000;

constexpr float MOTOR_PID_KP = PID_CONTROL_SETTING_MAX / 18.6;
constexpr float MOTOR_PID_KI = 100.0;
constexpr float MOTOR_PID_KD = 0.0;
constexpr float MOTOR_PID_SP = 0.0;
constexpr float MOTOR_PID_KP_INCR = 2.0;
constexpr float MOTOR_PID_KI_INCR = 2.0;
constexpr float MOTOR_PID_KD_INCR = 2.0;
constexpr float MOTOR_PID_SP_INCR = 0.05;

// disable the motor past this Roll angle
constexpr float MOTOR_DISABLE_ROLL_ANGLE = 35.0;

class MotorDriver {
    public:
        MotorDriver();
        void init();
	void cmd_internal(const MOTOR_DRIVER_CMD_t i_cmd);
	// FIXME -- change to MOTOR_DRIVER_CMD_t instead of uint8_t
	void cmd(const uint8_t i_cmd);
	// FIXME -- change to PID_CMD_t instead of uint8_t
        void PIDCmd(const uint8_t i_cmd);
        void setValues(pid_ctrl_t driver0, pid_ctrl_t driver1);
        void getValues(pid_ctrl_t& driver0, pid_ctrl_t& driver1);
        pid_ctrl_t getValue(void);
        void setActualRollAngle(float i_roll);
        void setDesiredRollAngle(float i_roll);
	void setSP(float setPoint);
	void send_all_client_data();
    private:
        PID<pid_ctrl_t> pidCtrl;
	bool motor_enabled;
	bool display_enabled;
        pid_ctrl_t drv_ctrla, drv_ctrlb; 
        void pwm_base_clock_modify(const bool up);
};

#endif
