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
#include "ble_svcs.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* The devicetree node identifier for the "led0" alias. */
/* led2 is the Green LED */
#define LED0_NODE DT_ALIAS(led2)
#define SLEEP_TIME_MS   1000
#define DEBUG_LED 1

#ifdef DEBUG_LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

int main(void)
{
    int ret;
#ifdef DEBUG_LED
    bool led_state = true;
#endif
    float roll, pitch, yaw;
    int16_t roll_i, pitch_i, yaw_i;
    IMU imu = IMU(DEVICE_DT_GET_ONE(st_lsm6ds3tr_c), DEVICE_DT_GET_ONE(st_lis3mdl_magn));

    printk("AHRS Started.\n");

#ifdef DEBUG_LED
    if (!gpio_is_ready_dt(&led)) {
	LOG_DBG("GPIO is not ready");
    	return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
	LOG_DBG("Failed to initialize GPIO");
    	return 0;
    }
#endif

    ret = ble_svcs_init();
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

#ifdef DEBUG_LED
    	ret = gpio_pin_toggle_dt(&led);
    	if (ret < 0) {
	    LOG_DBG("Failed to toggle GPIO");
            return 0;
    	}

    	led_state = !led_state;
    	k_msleep(SLEEP_TIME_MS);
#endif

        imu.update();
        imu.compute();
        imu.get_angles(roll, pitch, yaw);

        roll_i = (int16_t)roll;
        pitch_i = (int16_t)pitch;
        yaw_i = (int16_t)yaw;
        ble_svcs_send_euler_angles(roll_i, pitch_i, yaw_i);
    }


    return 0;
}

