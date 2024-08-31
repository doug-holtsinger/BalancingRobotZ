//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef __MADGWICKAHRS_H__
#define __MADGWICKAHRS_H__

#include "AHRS.h"
#include "imu_cmd.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define betaDef		0.1f		// 2 * proportional gain

class MadgwickAHRS : public AHRS
{
public:
    MadgwickAHRS() {}
    ~MadgwickAHRS() {}
    void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#ifdef BLE_CONSOLE_AVAILABLE
    void send_all_client_data(const bool *display_data, const bool settings_display);
#endif
    void cmd(const IMU_CMD_t cmd);

private:
    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az); 

    float beta = betaDef;				// algorithm gain

};

#endif
