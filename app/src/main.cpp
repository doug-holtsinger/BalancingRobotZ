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

#include "thread.h"

#include "imu.h"
//FIXME -- move qdec out of here
#include "qdec.h"
#include "ble_svcs.h"
#include "app_demux.h"
#include "param_store_ids.h"

/* The devicetree node identifier for the "led0" alias. */
/* led2 is the Green LED */
#define LED0_NODE DT_ALIAS(led2)
#define SLEEP_TIME_MS   2000
#define LED_THREAD_STACK_SIZE 2048

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    int ret;

    LOG_INF("AHRS Started.\n");
    QDEC qdec = QDEC(DEVICE_DT_GET_ONE(nordic_nrf_qdec));

    ret = qdec.init();
    if (ret < 0) {
	LOG_ERR("Failed to initialize QDEC");
    	return 1;
    }

    ret = ble_svcs_init();
    if (ret < 0) {
	LOG_ERR("Failed to initialize BLE");
    	return 1;
    }

    ble_svcs_register(&appDemuxExecHandler);

    return 0;
}

void led_thread(void *, void *, void *)
{
    int ret;
    bool led_state = true;

    if (!gpio_is_ready_dt(&led)) {
	LOG_DBG("GPIO is not ready");
    	return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
	LOG_DBG("Failed to initialize GPIO");
    	return;
    }

    while (true) {
    	ret = gpio_pin_toggle_dt(&led);
    	if (ret < 0) {
	    LOG_DBG("Failed to toggle GPIO");
            return;
    	}

    	led_state = !led_state;
    	k_msleep(SLEEP_TIME_MS);
    }

}

K_THREAD_DEFINE(led_tid, LED_THREAD_STACK_SIZE,
                led_thread, NULL, NULL, NULL,
                LED_THREAD_PRIORITY, K_ESSENTIAL|K_FP_REGS, 0);

