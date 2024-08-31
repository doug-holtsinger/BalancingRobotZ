
#include <string.h>
#include <math.h>

#include "AHRS.h"
#include "logdef.h"
#include "notify.h"

#ifdef BLE_CONSOLE_AVAILABLE
#include "bsp.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"
#endif

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//---------------------------------------------------------------------------------------------------
float AHRS::invSqrt(float x, bool accurate_calc) {
    float y = x;

    if (accurate_calc)
    {
        y = 1.0 / sqrt(x);
    } else {
        float halfx = 0.5f * x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
    }

    return y;
}

#ifdef BLE_CONSOLE_AVAILABLE
void AHRS::send_client_data(char *p)
{
    uint8_t *p_data = (uint8_t *)p;
    ble_svcs_send_client_notification(p_data, strlen(p));
}

void AHRS::send_all_client_data(const bool *display_data, const bool settings_display)
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    // Check if BLE connected, otherwise return.
    if ( !ble_svcs_connected() ) {
            return;
    }

    // Accelerometer
    if (display_data[IMU_ACCELEROMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , ACCELEROMETER_NORMAL, PRINTF_FLOAT_VALUE2(axN), PRINTF_FLOAT_VALUE2(ayN), PRINTF_FLOAT_VALUE2(azN) );
        send_client_data(s);
    }

    // Gyroscope
    if (display_data[IMU_GYROSCOPE])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_X, PRINTF_FLOAT_VALUE3(gxN) );
        send_client_data(s);
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_Y, PRINTF_FLOAT_VALUE3(gyN) );
        send_client_data(s);
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_Z, PRINTF_FLOAT_VALUE3(gzN) );
        send_client_data(s);
    }

    // Magnetometer
    if (display_data[IMU_MAGNETOMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , MAGNETOMETER_NORMAL, PRINTF_FLOAT_VALUE2(mxN), PRINTF_FLOAT_VALUE2(myN), PRINTF_FLOAT_VALUE2(mzN) );
        send_client_data(s);
    }

    // Quaternion
    if (display_data[IMU_AHRS])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q0, PRINTF_FLOAT_VALUE4(q0X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q1, PRINTF_FLOAT_VALUE4(q1X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q2, PRINTF_FLOAT_VALUE4(q2X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q3, PRINTF_FLOAT_VALUE4(q3X) );
        send_client_data(s);
    }

    if (settings_display)
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , SAMPLE_FREQ, PRINTF_FLOAT_VALUE2(sampleFreq) );
        send_client_data(s);
    }

}
#endif

void AHRS::ComputeAngles(float& roll, float& pitch, float& yaw) 
{
    // calculate roll in degrees
    float y = q0 * q1 + q2 * q3;
    float x = 0.5f - q1 * q1 - q2 * q2;
    if (y != 0.0f && x != 0.0f)
    {
        roll = l_roll = atan2f(y, x) * 57.29578f;
    } else 
    {
        roll = l_roll;
    }

    // attitude = asin(2*qx*qy + 2*qz*qw)
    // calculate pitch in degrees
    float arg = -2.0f * (q1 * q3 - q0 * q2);
    if (arg > 1.0f)
    {
        arg = 1.0f;
    } else if (arg < -1.0f)
    {
        arg = -1.0f;
    }
    pitch = asinf(arg);
    pitch = l_pitch = pitch * 57.29578f;

    // calculate yaw in degrees
    y = q1 * q2 + q0 * q3;
    x = 0.5f - q2 * q2 - q3 * q3;
    if (y != 0.0f && x != 0.0f)
    {
        // FIXME: Are there any issues in this calculation when x and/or y get close to 0? 
        yaw = l_yaw = atan2f(y, x) * 57.29578f + 180.0f;
    } else 
    {
        yaw = l_yaw;
    }
}

// Cython doesn't work with references passed back.
float AHRS::GetAngle(const EULER_ANGLE_SELECT_t angle_select) const
{
    switch (angle_select)
    {
        case ROLL: return l_roll; break; 
        case PITCH: return l_pitch; break; 
        case YAW: return l_yaw; break; 
	default: break;
    }
    return 0.0;
}

float AHRS::GetQuaternion(const QUATERNION_SELECT_t quaternion_select) const
{
    switch (quaternion_select)
    {
        case Q0: return q0; break; 
        case Q1: return q1; break; 
        case Q2: return q2; break; 
        case Q3: return q3; break; 
	default: break;
    }
    return 0.0;
}

void AHRS::GetNormalizedVectors(const IMU_SENSOR_t sensor, float& o_x, float& o_y, float& o_z) const
{
    switch(sensor)
    {
        case IMU_ACCELEROMETER:
	    o_x = axN; o_y = ayN ; o_z = azN; break;
        case IMU_GYROSCOPE:
	    o_x = gxN; o_y = gyN ; o_z = gzN; break;
        case IMU_MAGNETOMETER:
	    o_x = mxN; o_y = myN ; o_z = mzN; break;
	default: break;
    }

}

void AHRS::cmd(const IMU_CMD_t cmd)
{
    switch (cmd)
    {
	    case IMU_CMD_t::AHRS_SAMPLE_FREQ_UP:
            sampleFreq += 32.0f;
            break;
	    case IMU_CMD_t::AHRS_SAMPLE_FREQ_DOWN:
            sampleFreq -= 32.0f;
            break;
	default: break;
    }
}


