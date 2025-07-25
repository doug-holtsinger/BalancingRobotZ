/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_H__
#define __BLE_SVCS_H__

#include "app_demux.h"


int ble_svcs_init(void);
int ble_svcs_send_euler_angles(int16_t& roll, int16_t& pitch, int16_t& yaw);
int send_client_data(char* p_str);
typedef void (*nus_data_handler_t)(const APP_CMD_t data);
void ble_svcs_register(nus_data_handler_t data_handler_fn);
bool ble_svcs_connected();

#endif
