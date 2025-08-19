
#ifndef __MOTOR__H
#define __MOTOR__H

#include "pid.h"
#include "motorcmd.h"
#include "motor_cfg.h"

constexpr int16_t PID_CONTROL_SETTING_MAX = MOTOR_DRIVER_TOP_VALUE-1;
constexpr uint16_t MOTOR_CONTROL_SETTING_MASK = 0x7FFF;

/* 
 * Number of nanoseconds in one 16Mhz clock period 
 * The 16Mhz clock is built-in to the PWM driver at
 * drivers/src/pwm_nrfx.c
 */ 
constexpr uint16_t NANOSECS_PER_CLK_PERIOD = (uint16_t)(1000/16);
/* The max PWM count in the NRFX PWM code */
constexpr uint16_t PWM_TOP_COUNT = 8192;
constexpr uint32_t pwm_period_ns = PWM_TOP_COUNT * NANOSECS_PER_CLK_PERIOD;

// Setpoint default of PID controller
constexpr float MOTOR_DRIVER_SP_DEFAULT = 0.0;

constexpr float MOTOR_PID_KP = PID_CONTROL_SETTING_MAX / 18.6;
constexpr float MOTOR_PID_KI = 100.0;
constexpr float MOTOR_PID_KD = 0.0;
constexpr float MOTOR_PID_SP = 0.0;
constexpr float MOTOR_PID_KP_INCR = 5.0;
constexpr float MOTOR_PID_KI_INCR = 5.0;
constexpr float MOTOR_PID_KD_INCR = 2.0;
constexpr float MOTOR_PID_SP_INCR = 0.05;

/** @brief PWM base clock periods in integer nanoseconds. */
constexpr uint32_t PWM_CLK_PERIOD_16MHz  = 62;
constexpr uint32_t PWM_CLK_PERIOD_8MHz   = 125;
constexpr uint32_t PWM_CLK_PERIOD_4MHz   = (PWM_CLK_PERIOD_8MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_2MHz   = (PWM_CLK_PERIOD_4MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_1MHz   = (PWM_CLK_PERIOD_2MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_500kHz = (PWM_CLK_PERIOD_1MHz*2);
constexpr uint32_t PWM_CLK_PERIOD_250kHz = (PWM_CLK_PERIOD_500kHz*2);
constexpr uint32_t PWM_CLK_PERIOD_125kHz = (PWM_CLK_PERIOD_250kHz*2);

#if 0
constexpr bool MOTOR_ENABLE_DEFAULT = false;
#else
constexpr bool MOTOR_ENABLE_DEFAULT = true;
#endif
constexpr bool MOTOR_DISPLAY_DEFAULT = true;

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

	struct pwm_dt_spec pwm_motor[2];
	struct gpio_dt_spec motor_direction[2];

};

#endif
