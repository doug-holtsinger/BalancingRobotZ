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
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */


#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(PERF, CONFIG_SENSOR_LOG_LEVEL);

#include "notify.h"
#include "logdef.h"
#include "ble_svcs.h"

#include "perf.h"

void Perf::update(float roll, pid_ctrl_t pidCtrlValue, int32_t wheel_encoder_speed)
{

    if (roll > roll_max)
    {
        roll_normalized = 1.0f;
    } else
    {
        roll_normalized = roll / roll_max;
    }

    pid_ctrl_normalized = pidCtrlValue / (float)MOTOR_DRIVER_MAX_VALUE;

    if (abs(wheel_encoder_speed) > wheel_speed_max)
    {
        wheel_speed_max = abs(wheel_encoder_speed);
    }

    if (wheel_speed_max != 0)
    {
        wheel_speed_normalized = wheel_encoder_speed / (float)wheel_speed_max;
    }

}

void Perf::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    if ( !ble_svcs_connected() ) {
        return;
    }

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 ,
            LAG,
            PRINTF_FLOAT_VALUE2(roll_normalized),
            PRINTF_FLOAT_VALUE2(pid_ctrl_normalized),
            PRINTF_FLOAT_VALUE2(wheel_speed_normalized)
    );
    send_client_data(s);

}

