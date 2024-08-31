#ifndef __SIMPLEAHRS_H__
#define __SIMPLEAHRS_H__

#include "AHRS.h"
#include "imu_cmd.h"

class SimpleAHRS : public AHRS
{
public:
    SimpleAHRS() {}
    ~SimpleAHRS() {}
    void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void ComputeAngles(float& roll, float& pitch, float& yaw);
#ifdef BLE_CONSOLE_AVAILABLE
    void send_all_client_data(const bool *display_data, const bool settings_display);
#endif
    void cmd(const IMU_CMD_t cmd);
private:
    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az); 

};

#endif
