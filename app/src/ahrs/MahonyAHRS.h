//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef __MAHONYAHRS_H__
#define __MAHONYAHRS_H__

#include "AHRS.h"
#include "imu_cmd.h"

//---------------------------------------------------------------------------------------------------
// Definitions
#define twoKpDef    (2.0f * 0.5f)    // 2 * proportional gain 0.5f original
#define twoKiDef    (2.0f * 0.0f)    // 2 * integral gain

class MahonyAHRS : public AHRS
{
public:
    MahonyAHRS() {}
    ~MahonyAHRS() {}
    void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#ifdef BLE_CONSOLE_AVAILABLE
    void send_all_client_data(const bool *display_data, const bool settings_display);
#endif
    void cmd(const IMU_CMD_t cmd);

private:
    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az); 

    float twoKp = twoKpDef;    // 2 * proportional gain (Kp)
    float twoKi = twoKiDef;    // 2 * integral gain (Ki)

    float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

};

#endif
