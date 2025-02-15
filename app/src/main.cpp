/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <app_version.h>

#include <zephyr/bluetooth/services/nus.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include "imu.h"
#include "qdec.h"
#include "ble_svcs.h"
#include "app_demux.h"
#include "param_store_ids.h"

/* The devicetree node identifier for the "led0" alias. */
/* led2 is the Green LED */
#define LED0_NODE DT_ALIAS(led2)
#define SLEEP_TIME_MS   1000
#define DEBUG_LED 1

#ifdef DEBUG_LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

void main_thread(void *, void *, void *)
{
    int ret;
#ifdef DEBUG_LED
    bool led_state = true;
#endif
    QDEC qdec = QDEC(DEVICE_DT_GET_ONE(nordic_nrf_qdec));

    LOG_INF("AHRS Started.\n");

#ifdef DEBUG_LED
    if (!gpio_is_ready_dt(&led)) {
	LOG_DBG("GPIO is not ready");
    	return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
	LOG_DBG("Failed to initialize GPIO");
    	return;
    }
#endif

    ret = qdec.init();
    if (ret < 0) {
	LOG_ERR("Failed to initialize QDEC");
    	return;
    }

    ret = ble_svcs_init();
    if (ret < 0) {
	LOG_ERR("Failed to initialize BLE");
    	return;
    }

#if 0
    //DSH4 FIXME

    appDemuxRegisterHandler(
        std::bind( &IMU::cmd, std::ref(imu), std::placeholders::_1),
        appDemuxCmdType(IMU_CMD_t::CMD_MAX) );
#endif

#if 0
    // FIXME
    // Add command handler for Motor Driver
    appDemuxAddHandler(
        std::bind( &MotorDriver::cmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(MOTOR_DRIVER_CMD_t::CMD_MAX) );

    // Add command handler for Motor Driver PID
    appDemuxAddHandler(
        std::bind( &MotorDriver::PIDCmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );

    // Add command handler for Speed Control PID
    typedef void (PID<float>::*member_func_ptr)(const APP_CMD_t);
    member_func_ptr f = (member_func_ptr)&PID<float>::cmd;
    appDemuxAddHandler(
        std::bind( f,
                   std::ref(speedControlPID),
                   std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );
#endif

    ble_svcs_register(&appDemuxExecHandler);

    while (true) {

#ifdef DEBUG_LED
    	ret = gpio_pin_toggle_dt(&led);
    	if (ret < 0) {
	    LOG_DBG("Failed to toggle GPIO");
            return;
    	}

    	led_state = !led_state;
    	k_msleep(SLEEP_TIME_MS);
#endif


    }

}

#define MAIN_THREAD_STACK_SIZE 2048
#define MAIN_THREAD_PRIORITY 4
K_THREAD_DEFINE(main_tid, MAIN_THREAD_STACK_SIZE,
                main_thread, NULL, NULL, NULL,
                MAIN_THREAD_PRIORITY, K_ESSENTIAL|K_FP_REGS, 0);

