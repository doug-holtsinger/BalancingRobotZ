
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

// FIXME - put these into the DTS
// PWM port pin connections on nRF52 board
// Direction pin
//constexpr uint8_t MOTOR_DRIVER_APHASE = 20;
// PWM pin
//constexpr uint8_t MOTOR_DRIVER_AENBL = 17;
// Direction pin
//constexpr uint8_t MOTOR_DRIVER_BPHASE = 24;
// PWM pin
//constexpr uint8_t MOTOR_DRIVER_BENBL = 22;

/** @brief PWM base clock periods in integer nanoseconds. */
constexpr uint32_t PWM_CLK_PERIOD_16MHz  = 62;
constexpr uint32_t PWM_CLK_PERIOD_8MHz   = 125;
constexpr uint32_t PWM_CLK_PERIOD_4MHz   = (PWM_CLK_PERIOD_8MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_2MHz   = (PWM_CLK_PERIOD_4MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_1MHz   = (PWM_CLK_PERIOD_2MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_500kHz = (PWM_CLK_PERIOD_1MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_250kHz = (PWM_CLK_PERIOD_500kHz*2);
constexpr uint32_t PWM_CLK_PERIOD_125kHz = (PWM_CLK_PERIOD_250kHz*2);

class MotorDriver {
    public:
        MotorDriver();
        int init();
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
	uint32_t pwm_base_clock;

	struct pwm_dt_spec pwm_motor[2];

        void pwm_base_clock_modify(const bool up);
};

#endif
