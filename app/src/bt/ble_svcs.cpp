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

LOG_MODULE_REGISTER(BLE, CONFIG_SENSOR_LOG_LEVEL);

#define DEVICE_NAME        CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN        (sizeof(DEVICE_NAME) - 1)

/* Advertising data */
#define NUM_EULER_ANGLES 3
#define NUM_MANUFACTURER_DATA (NUM_EULER_ANGLES+1)
int16_t manufacturer_data[1+NUM_EULER_ANGLES];
bool notifications_enabled = false;
bool ble_connected = false;

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
    // printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    char message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);

    memcpy(message, data, MIN(sizeof(message) - 1, len));
    printk("%s() - Len: %d, Message: %s\n", __func__, len, message);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_WRN("Connection failed (err %u)", err);
        return;
    }
    ble_connected = true;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    ble_connected = false;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s (reason %u)", addr, reason);
}

struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

int ble_svcs_init(void)
{
    int err = 0;

    manufacturer_data[0] = -1;
    bt_le_adv_param bt_adv_param[1] = BT_LE_ADV_CONN_ONE_TIME;
    static struct bt_conn_cb conn_callbacks = {
        .connected    = connected,
        .disconnected = disconnected,
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

    err = bt_le_adv_start(bt_adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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
    if (err) 
    { 
        LOG_ERR("Failed to update advertising: %d\n", err);
    }

    return err;
}

