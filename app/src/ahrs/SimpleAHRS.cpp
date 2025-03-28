
//---------------------------------------------------------------------------------------------------
// Header files

#include <cstdio>
#include <math.h>

#include "AHRS.h"
#include "SimpleAHRS.h"
#include "imu_cmd.h"
#include "logdef.h"
#include "notify.h"

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------

void SimpleAHRS::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
}

void SimpleAHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
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

void SimpleAHRS::compute_angles(float& roll, float& pitch, float& yaw) 
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

void SimpleAHRS::send_all_client_data(const bool *display_data, const bool settings_display)
{
    AHRS::send_all_client_data(display_data, settings_display);
}

void SimpleAHRS::cmd(const IMU_CMD_t cmd)
{
    AHRS::cmd(cmd);
}
