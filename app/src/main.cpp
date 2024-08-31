/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <app_version.h>

#include <zephyr/bluetooth/services/nus.h>

#include "imu.h"

extern int bt_nus_init();

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* The devicetree node identifier for the "led0" alias. */
/* led2 is the Green LED */
#define LED0_NODE DT_ALIAS(led2)
#define SLEEP_TIME_MS   1000

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    int ret;
    bool led_state = true;
    IMU imu = IMU();

    printk("Sample - Bluetooth Peripheral NUS\n");

    if (!gpio_is_ready_dt(&led)) {
	LOG_DBG("GPIO is not ready");
    	return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
	LOG_DBG("Failed to initialize GPIO");
    	return 0;
    }

    ret = bt_nus_init();
    if (ret < 0) {
	LOG_DBG("Failed to initialize BLE");
    	return 0;
    }

    ret = imu.init();
    if (ret < 0) {
	LOG_DBG("Failed to initialize IMU");
    	return 0;
    }

    while (true) {
    	const char *hello_world = "Hello World!\n";

    	ret = gpio_pin_toggle_dt(&led);
    	if (ret < 0) {
	    LOG_DBG("Failed to toggle GPIO");
            return 0;
    	}

    	led_state = !led_state;
    	k_msleep(SLEEP_TIME_MS);

    	ret = bt_nus_send(NULL, hello_world, strlen(hello_world));
    	printk("Data send - Result: %d\n", ret);

    	if (ret < 0 && (ret != -EAGAIN) && (ret != -ENOTCONN)) {
	    LOG_DBG("Failed to send BLE data");
            return ret;
    	}

        imu.update();
    }


    return 0;
}

