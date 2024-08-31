
//---------------------------------------------------------------------------------------------------
// Header files

#include <stdio.h>
#include <math.h>

#include "AHRS.h"
#include "SimpleAHRS.h"
#include "imu_cmd.h"
#include "logdef.h"
#include "notify.h"

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------

void SimpleAHRS::UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
}

void SimpleAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az, false);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;     
	axN = ax;
	ayN = ay;
	azN = az;
    }
}

void SimpleAHRS::ComputeAngles(float& roll, float& pitch, float& yaw) 
{
    float arg = ayN;
    pitch = yaw = 0.0;

    if (arg > 1.0f)
    {
        arg = 1.0f;
    } else if (arg < -1.0f)
    {
        arg = -1.0f;
    }
    roll = asinf(arg);
    roll = l_roll = roll * 57.29578f;
}

#ifdef BLE_CONSOLE_AVAILABLE
void SimpleAHRS::send_all_client_data(const bool *display_data, const bool settings_display)
{
    AHRS::send_all_client_data(display_data, settings_display);
}
#endif

void SimpleAHRS::cmd(const IMU_CMD_t cmd)
{
    AHRS::cmd(cmd);
}
