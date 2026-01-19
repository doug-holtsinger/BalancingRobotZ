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

#include "ble_svcs.h"
#include "app_demux.h"
#include "param_store_ids.h"

#include "datalog.h"

/* The devicetree node identifier for the "led0" alias. */
/* led2 is the Green LED */
#define LED0_NODE DT_ALIAS(led2)
#define SLEEP_TIME_MS   2000
#define LED_THREAD_STACK_SIZE 2048

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
#ifdef DATALOG_ENABLED
	datalog_trigger_dump();
#else
	LOG_DBG("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
#endif
}

int main(void)
{
    int ret;

    LOG_INF("AHRS Started.\n");

    /* 
     * Setup and configure button
     */
    if (!gpio_is_ready_dt(&button)) 
    {
        LOG_ERR("Button device %s is not ready\n", button.port->name);
        return 1;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Error %d: failed to configure %s pin %d\n",
               ret, button.port->name, button.pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button,
                          GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
            ret, button.port->name, button.pin);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    /* 
     * Setup and configure BLE
     */
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

