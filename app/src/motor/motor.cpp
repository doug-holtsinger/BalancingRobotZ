/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(MOTOR, CONFIG_SENSOR_LOG_LEVEL);

#include "thread.h"

#include "notify.h"
#include "logdef.h"
#include "ble_svcs.h"
#include "app_demux.h"
#include "imu.h"

#include "motor.h"
#include "motorcmd.h"
#include "param_store_ids.h"
#include "pid.h"
#include "pid_num.h"

#define MOTOR_DRIVER_THREAD_STACK_SIZE 2048

constexpr float SPEED_PID_KP = 0.02;
constexpr float SPEED_PID_KI = 0.0;
constexpr float SPEED_PID_KD = 0.0;
constexpr float SPEED_PID_SP = 0.0;
constexpr float SPEED_PID_KP_INCR = 0.005;
constexpr float SPEED_PID_KI_INCR = 0.005;
constexpr float SPEED_PID_KD_INCR = 0.005;
constexpr float SPEED_PID_SP_INCR = 0.05;
constexpr float SPEED_PID_CTRL_MAX = 1.0;
constexpr bool SPEED_PID_REVERSE_OUTPUT = true;
constexpr bool SPEED_PID_LOW_PASS_FILTER = true;

#if 1
// extern FIXME
int32_t wheel_encoder;
#endif

void motor_driver_thread(void *, void *, void *)
{
    float speedControlSP = 0.0;
    int debug_cnt = 0;
    PID speedControlPID = PID({SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_SP},
           {SPEED_PID_KP_INCR, SPEED_PID_KI_INCR, SPEED_PID_KD_INCR, SPEED_PID_SP_INCR},
            SPEED_PID_CTRL_MAX, SPEED_PID_RECORD_KEY, SPEED_PID_NUM,
            SPEED_PID_REVERSE_OUTPUT, SPEED_PID_LOW_PASS_FILTER);
    MotorDriver md = MotorDriver();

    // Register command handler for Motor Driver
    appDemuxRegisterHandler(
        std::bind( &MotorDriver::cmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(MOTOR_DRIVER_CMD_t::CMD_MAX) );

    // Register command handler for Motor Driver PID
    appDemuxRegisterHandler(
        std::bind( &MotorDriver::PIDCmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );

    // Add command handler for Speed Control PID
    // FIXME -- why is this different from the others?
    typedef void (PID<float>::*member_func_ptr)(const APP_CMD_t);
    member_func_ptr f = (member_func_ptr)&PID<float>::cmd;
    appDemuxRegisterHandler(
        std::bind( f,
                   std::ref(speedControlPID),
                   std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );

    /* set priority to normal after initializing command handler */
    k_thread_priority_set(k_current_get(), NORMAL_THREAD_PRIORITY);

    // Initialize the Motor Driver
    if ( md.init() != 0 )
    {
        return;
    }

    while (true)
    {
        md.setActualRollAngle(get_imu_roll());
        speedControlSP = speedControlPID.update(static_cast<float>(wheel_encoder));
        md.setDesiredRollAngle(speedControlSP);
	if ((debug_cnt & 0x3FF) == 0)
	{
            md.send_all_client_data();
	}

	if ((debug_cnt & 0x3FF) == 0)
	{
            LOG_DBG("cnt %d", debug_cnt);
    	    k_msleep(100);
	}
	debug_cnt++;
	// DSH4
	k_yield();
    }
}

K_THREAD_DEFINE(motor_driver_tid, MOTOR_DRIVER_THREAD_STACK_SIZE,
                motor_driver_thread, NULL, NULL, NULL,
                MOTOR_DRIVER_THREAD_PRIORITY, K_ESSENTIAL|K_FP_REGS, 0);

MotorDriver::MotorDriver() :
    pidCtrl({MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_SP},
       {MOTOR_PID_KP_INCR, MOTOR_PID_KI_INCR, MOTOR_PID_KD_INCR, MOTOR_PID_SP_INCR},
        PID_CONTROL_SETTING_MAX, MOTOR_PID_RECORD_KEY, MOTOR_PID_NUM),
    motor_enabled(true),
    display_enabled(true),
    drv_ctrla(0), 
    drv_ctrlb(0),
    pwm_base_clock(PWM_CLK_PERIOD_4MHz)
{
}

void MotorDriver::setActualRollAngle(float i_roll)
{
    drv_ctrla = pidCtrl.update(i_roll);
    drv_ctrlb = -drv_ctrla;
    if (abs(i_roll) > MOTOR_DISABLE_ROLL_ANGLE)
    {
        drv_ctrla = drv_ctrlb = 0;
    }
    this->setValues(drv_ctrla, drv_ctrlb);
}

void MotorDriver::setDesiredRollAngle(float i_roll)
{
    pidCtrl.setSP(i_roll);
}

void MotorDriver::getValues(pid_ctrl_t& driver0, pid_ctrl_t& driver1)
{
    driver0 = drv_ctrla;
    driver1 = drv_ctrlb;
}

pid_ctrl_t MotorDriver::getValue()
{
    return drv_ctrla;
}

void MotorDriver::setValues(pid_ctrl_t driver0, pid_ctrl_t driver1)
{
    if (!motor_enabled)
    {
        driver0 = driver1 = 0;
    }

    // FIXME drive GPIO pins direction and PWM

}

int MotorDriver::init()
{
    pidCtrl.init();

    // static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#if 0
    pwm_motor[0] = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
    pwm_motor[1] = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#else
    pwm_motor[0] = PWM_DT_SPEC_GET(DT_ALIAS(motor0));
    pwm_motor[1] = PWM_DT_SPEC_GET(DT_ALIAS(motor1));
#endif

    if (!pwm_is_ready_dt(&pwm_motor[0])) {
        LOG_ERR("PWM device %s is not ready\n", pwm_motor[0].dev->name);
        return 1;
    }
    if (!pwm_is_ready_dt(&pwm_motor[1])) {
        LOG_ERR("PWM device %s is not ready\n", pwm_motor[1].dev->name);
        return 1;
    }

    // Configure GPIO pins for Motor direction -- FIXME

    // Configure PWM pins for Motor PWM -- FIXME

    //DSH4  base_clock=125,  period = 62
#if 0
    LOG_DBG("PWM0 clk %u\n", pwm_base_clock);
    pwm_set_dt(&pwm_motor[0], pwm_base_clock, pwm_base_clock / 2U);
    LOG_DBG("PWM1 clk %u\n", pwm_base_clock);
    pwm_set_dt(&pwm_motor[1], pwm_base_clock, pwm_base_clock / 2U);
#else
    /* period in terms of the number of 16Mhz clocks , originally was period in 8 Mhz clocks (TOP = 8192  , 8 Mhz clock) */
#define CLK_MULT (1000/16)
// #define TOP_COUNT 8192
#define TOP_COUNT 1000
    // 8192 * 1000 / 16 nanoseconds =  512,000 ns = 512ms
    // 1 16Mhz clock = 62.5 nanoseconds
    // base clock is the number of nanoseconds for 8192 clocks at 16Mhz
    pwm_base_clock = TOP_COUNT * CLK_MULT;
    // 1500 slow turn of the one motor
    // 2000 slow turn of both motors in opposite directions (fast if 2 channels)
    // 2500 moderate turn of both motors in opposite directions
    // uint32_t pwm_pulse_cycles = 1500 * CLK_MULT;       
    // uint32_t pwm_pulse_cycles = pwm_base_clock / 4;
    uint32_t pwm_pulse_cycles = pwm_base_clock / 6;
    // uint32_t pwm_pulse_cycles = 2500 * CLK_MULT;          
    //  b0 89   b0 89  00 00  00 00   x3089 = 12425 decimal
    LOG_DBG("PWM0 clk %u %u\n", pwm_base_clock, pwm_pulse_cycles);
    pwm_set_dt(&pwm_motor[0], pwm_base_clock, pwm_pulse_cycles );
#if 1
    LOG_DBG("PWM1 clk %u\n", pwm_base_clock);
    pwm_set_dt(&pwm_motor[1], pwm_base_clock, pwm_pulse_cycles);
#endif

#endif

#if 0
#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

    LOG_DBG("Calibrating for channel %d...\n", pwm_motor.channel);
    max_period = MAX_PERIOD;
    while (pwm_set_dt(&pwm_motor, max_period, max_period / 2U)) {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD)) {
                LOG_ERR("Error: PWM device "
                       "does not support a period at least %lu\n",
                       4U * MIN_PERIOD);
                return 0;
        }
    }
#endif

    return 0;

}

void MotorDriver::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    if ( !ble_svcs_connected() )
    {
        return;
    }
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_DISPLAY, display_enabled); 
    send_client_data(s);

    if ( !display_enabled )
    {
	return;
    }

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_ENABLED, motor_enabled); 
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_DRIVER, drv_ctrla);
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                PWM_CLOCK, pwm_base_clock); 
    send_client_data(s);

    pidCtrl.send_all_client_data();
}

void MotorDriver::pwm_base_clock_modify(const bool up)
{
    if (up)
    {
        switch (pwm_base_clock)
        {
            case PWM_CLK_PERIOD_125kHz:
                pwm_base_clock = PWM_CLK_PERIOD_250kHz;
                break;
            case PWM_CLK_PERIOD_250kHz:
                pwm_base_clock = PWM_CLK_PERIOD_500kHz;
                break;
            case PWM_CLK_PERIOD_500kHz:
                pwm_base_clock = PWM_CLK_PERIOD_1MHz;
                break;
            case PWM_CLK_PERIOD_1MHz:
                pwm_base_clock = PWM_CLK_PERIOD_2MHz;
                break;
            case PWM_CLK_PERIOD_2MHz:
                pwm_base_clock = PWM_CLK_PERIOD_4MHz;
                break;
            case PWM_CLK_PERIOD_4MHz:
                pwm_base_clock = PWM_CLK_PERIOD_8MHz;
                break;
            case PWM_CLK_PERIOD_8MHz:
                pwm_base_clock = PWM_CLK_PERIOD_16MHz;
                break;
            case PWM_CLK_PERIOD_16MHz:
            default: break;
        }
    } else 
    {
        switch (pwm_base_clock)
        {
            case PWM_CLK_PERIOD_125kHz:
                break;
            case PWM_CLK_PERIOD_250kHz:
                pwm_base_clock = PWM_CLK_PERIOD_125kHz;
                break;
            case PWM_CLK_PERIOD_500kHz:
                pwm_base_clock = PWM_CLK_PERIOD_250kHz;
                break;
            case PWM_CLK_PERIOD_1MHz:
                pwm_base_clock = PWM_CLK_PERIOD_500kHz;
                break;
            case PWM_CLK_PERIOD_2MHz:
                pwm_base_clock = PWM_CLK_PERIOD_1MHz;
                break;
            case PWM_CLK_PERIOD_4MHz:
                pwm_base_clock = PWM_CLK_PERIOD_2MHz;
                break;
            case PWM_CLK_PERIOD_8MHz:
                pwm_base_clock = PWM_CLK_PERIOD_4MHz;
                break;
            case PWM_CLK_PERIOD_16MHz:
                pwm_base_clock = PWM_CLK_PERIOD_8MHz;
                break;
            default: break;
        }
    }
    pwm_set_dt(&pwm_motor[0], pwm_base_clock, pwm_base_clock / 2U);
}

void MotorDriver::cmd_internal(const MOTOR_DRIVER_CMD_t i_cmd)
{
    switch (i_cmd)
    {
        case MOTOR_DRIVER_CMD_t::TOGGLE_POWER:
            motor_enabled = !motor_enabled;
            break;
        case MOTOR_DRIVER_CMD_t::TOGGLE_DISPLAY:
            display_enabled = !display_enabled;
            break;
        case MOTOR_DRIVER_CMD_t::PWM_CLK_UP:
	    // pwm_base_clock_modify(true);
            break;
        case MOTOR_DRIVER_CMD_t::PWM_CLK_DOWN:
	    // pwm_base_clock_modify(false);
            break;
        default: break;
    }
}

void MotorDriver::cmd(const uint8_t i_cmd)
{
    LOG_DBG("cmd %d", i_cmd);
    cmd_internal(static_cast<MOTOR_DRIVER_CMD_t>(i_cmd));
}

void MotorDriver::PIDCmd(const uint8_t i_cmd)
{
    LOG_DBG("pcmd %d", i_cmd);
    pidCtrl.cmd(static_cast<PID_CMD_t>(i_cmd));
}

