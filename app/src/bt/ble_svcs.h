/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_H__
#define __BLE_SVCS_H__

#include "app_demux.h"
// #include "ble_svcs_cmd.h"


int ble_svcs_init(void);
int ble_svcs_send_euler_angles(int16_t& roll, int16_t& pitch, int16_t& yaw);
int send_client_data(char* p_str, const size_t len);
typedef void (*nus_data_handler_t)(const APP_CMD_t data);
void ble_svcs_register(nus_data_handler_t data_handler_fn);
bool ble_svcs_connected();
#if 0
void ble_svcs_cmd(BLE_CMD_t ble_cmd, uint16_t data);
void ble_svcs_event_handler(bsp_event_t event);
void ble_svcs_advertising_start(void);
#endif

#endif
