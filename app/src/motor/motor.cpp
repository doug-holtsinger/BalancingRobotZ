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
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(MOTOR, CONFIG_SENSOR_LOG_LEVEL);

#include "thread.h"

#include "notify.h"
#include "logdef.h"
#include "ble_svcs.h"
#include "ble_cfg.h"
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
// extern FIXME -- connect to QDEC
int32_t wheel_encoder = 0;
#endif


void motor_driver_thread(void *, void *, void *)
{
    float speedControlSP = 0.0;
    int ble_send_cnt = 0;
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
	if ((ble_send_cnt++ & BLE_SEND_NOTIFICATION_INTERVAL) == 0)
	{
            md.send_all_client_data();
	}

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
    motor_enabled(MOTOR_ENABLE_DEFAULT),
    display_enabled(MOTOR_DISPLAY_DEFAULT),
    drv_ctrla(0), 
    drv_ctrlb(0)
{
}

void MotorDriver::setActualRollAngle(float i_roll)
{
    drv_ctrla = pidCtrl.update(i_roll);
    // motors spin in opposite directions
    drv_ctrlb = -drv_ctrla;
    if (motor_enabled && abs(i_roll) > MOTOR_DISABLE_ROLL_ANGLE)
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
    uint32_t pwm_pulse0_ns, pwm_pulse1_ns;

    // Drive GPIO pins direction
    if (driver0 >= 0)
    {
        gpio_pin_set_dt(&motor_direction[0], 1);
    } else {
        gpio_pin_set_dt(&motor_direction[0], 0);
    }
    if (driver1 >= 0)
    {
        gpio_pin_set_dt(&motor_direction[1], 1);
    } else {
        gpio_pin_set_dt(&motor_direction[1], 0);
    }

    pwm_pulse0_ns = abs(driver0) * pwm_period_ns / MOTOR_DRIVER_TOP_VALUE;
    pwm_pulse1_ns = abs(driver1) * pwm_period_ns / MOTOR_DRIVER_TOP_VALUE;

    if (!motor_enabled)
    {
        pwm_pulse0_ns = pwm_pulse1_ns = 0;
    }
    pwm_set_dt(&pwm_motor[0], pwm_period_ns, pwm_pulse0_ns );
    pwm_set_dt(&pwm_motor[1], pwm_period_ns, pwm_pulse1_ns );

}

int MotorDriver::init()
{
    int ret;

    pidCtrl.init();

    pwm_motor[0] = PWM_DT_SPEC_GET(DT_ALIAS(motor0));
    pwm_motor[1] = PWM_DT_SPEC_GET(DT_ALIAS(motor1));

    motor_direction[0] = GPIO_DT_SPEC_GET(DT_ALIAS(motordir0), gpios);
    motor_direction[1] = GPIO_DT_SPEC_GET(DT_ALIAS(motordir1), gpios);

    for (int i=0 ; i < 2; i++)
    {
        if (!pwm_is_ready_dt(&pwm_motor[i])) {
            LOG_ERR("PWM device %s is not ready\n", pwm_motor[i].dev->name);
            return 1;
        }
        if (!gpio_is_ready_dt(&motor_direction[i])) {
            LOG_ERR("GPIO device %s is not ready\n", motor_direction[i].port->name);
            return 1;
        }
    }

    for (int i=0 ; i < 2; i++)
    {
        ret = gpio_pin_configure_dt(&motor_direction[i], GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("GPIO device %s config issue %d\n", motor_direction[i].port->name, ret);
            return ret;
        }
    }

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

    pidCtrl.send_all_client_data();
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

