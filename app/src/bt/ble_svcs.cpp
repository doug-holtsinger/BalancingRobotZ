/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *   For the NUS code, originally copied from:
 *      zephyr/samples/bluetooth/peripheral_nus/src/main.c
 *   With some copies also from the following for the BLE connection handling:
 *      nrf/applications/connectivity_bridge/src/modules/ble_handler.c 
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

#include "app_demux.h"

LOG_MODULE_REGISTER(BLE, CONFIG_SENSOR_LOG_LEVEL);

#define DEVICE_NAME        CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN        (sizeof(DEVICE_NAME) - 1)

/* Advertising data */
#define NUM_EULER_ANGLES 3
#define NUM_MANUFACTURER_DATA (NUM_EULER_ANGLES+1)
#define ADV_INT_MIN 0x01e0 /* 300 ms */
#define ADV_INT_MAX 0x0260 /* 380 ms */
static int16_t manufacturer_data[1+NUM_EULER_ANGLES];
static bool notifications_enabled = false;
static bool ble_connected = false;
static struct bt_conn *current_conn;
static void (*nus_data_handler_cb)(const APP_CMD_t data) = NULL;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manufacturer_data, sizeof(int16_t) * NUM_MANUFACTURER_DATA)
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static void notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);
    notifications_enabled = enabled;
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    uint8_t message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);

    memcpy(message, data, MIN(sizeof(message) - 1, len));
    if (len == 1)
    {
	APP_CMD_t cmd = static_cast<APP_CMD_t>(message[0]);
        LOG_DBG("cmd %d pri %d", cmd,  k_thread_priority_get(k_current_get() ));
        nus_data_handler_cb(cmd);
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_WRN("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    ble_connected = true;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    ble_connected = false;
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s (reason %u)", addr, reason);
}

struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

static void start_adv(void)
{
    int err;

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err != 0) 
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
}

int ble_svcs_init(void)
{
    int err = 0;
    // struct bt_le_adv_param bt_adv_param[1] = BT_LE_ADV_CONN_FAST_2;

    // FIXME: use a non-standard manufacturer ID
    manufacturer_data[0] = -1;

    static struct bt_conn_cb conn_callbacks = {
        .connected    = connected,
        .disconnected = disconnected,
        .recycled = start_adv,
    };

    err = bt_nus_cb_register(&nus_listener, NULL);
    if (err) {
        LOG_ERR("Failed to register NUS callback: %d\n", err);
        return err;
    }

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Failed to enable bluetooth: %d\n", err);
        return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Failed to start advertising: %d\n", err);
        return err;
    }

    LOG_DBG("init complete\n");

    return 0;
}

/**@brief Function for sending Euler Angles over BLE in the Advertising Channel
 */
int ble_svcs_send_euler_angles(int16_t& roll, int16_t& pitch, int16_t& yaw)
{
    int err = 0;

    if (ble_connected)
    {
	    return 0;
    }

    manufacturer_data[1] = roll;
    manufacturer_data[2] = pitch;
    manufacturer_data[3] = yaw;

    err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
#if 0
    if (err) 
    { 
        LOG_ERR("Failed to update advertising: %d\n", err);
    }
#endif

    return err;
}

/**@brief Function for sending data to client over BLE using notifications
 */
int send_client_data(char* p_str)
{
    int err = 0;
    if (ble_connected)
    {
        uint8_t *p = (uint8_t *)p_str;
        err = bt_nus_send(current_conn, p, strlen(p_str));
    }
    return err;
}

/**@brief register callback function for NUS data handler function
 */
void ble_svcs_register(void (*data_handler_fn)(const APP_CMD_t data))
{
    nus_data_handler_cb = data_handler_fn;
}

bool ble_svcs_connected()
{
    return ble_connected;
}
