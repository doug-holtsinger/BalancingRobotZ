
#ifndef __NOTIFY_H__
#define __NOTIFY_H__

#define NOTIFY_PRINT_STR_MAX_LEN (size_t)256

typedef enum
{
    EULER_ANGLES = 0,

    ACCELEROMETER_NORMAL,
    ACCELEROMETER_CAL,
    ACCELEROMETER_UNCAL,
    ACCELEROMETER_MIN_THRESHOLD,

    MAGNETOMETER_NORMAL,
    MAGNETOMETER_CAL,
    MAGNETOMETER_UNCAL,
    MAGNETOMETER_MIN_THRESHOLD,

    QUATERNION_Q0,
    QUATERNION_Q1,
    QUATERNION_Q2,
    QUATERNION_Q3,

    GYROSCOPE_NORMAL_X,
    GYROSCOPE_NORMAL_Y,
    GYROSCOPE_NORMAL_Z,
    GYROSCOPE_CAL,
    GYROSCOPE_UNCAL,
    GYROSCOPE_MIN_THRESHOLD,

    GYRO_CORRECTION, 
    BIT_FLAGS,

    PROP_GAIN, 
    INTEG_GAIN,
    SAMPLE_FREQ,
    AHRS_ALGORITHM,

    BETA_GAIN,
    ODR_HZ_ACCELEROMETER,
    ODR_HZ_GYRO,
    ODR_HZ_MAGNETOMETER,

    ACCELEROMETER_NOISE_THRESHOLD_MULT,
    MAGNETOMETER_NOISE_THRESHOLD_MULT,
    GYROSCOPE_NOISE_THRESHOLD_MULT, /* 31 */

    NOTIFY_BASE_MAX = GYROSCOPE_NOISE_THRESHOLD_MULT

} DATA_NOTIFY_t;


typedef enum
{
    MAGNETOMETER_STABILITY = 0,
    GYROSCOPE_ENABLE,
    DATA_HOLD_ACCELEROMETER,
    DATA_HOLD_MAGNETOMETER,
    DATA_HOLD_GYROSCOPE,
    IDEAL_DATA_ACCELEROMETER,
    IDEAL_DATA_MAGNETOMETER,
    IDEAL_DATA_GYROSCOPE,
    DISPLAY_DATA_AHRS,
    DISPLAY_DATA_ACCELEROMETER,
    DISPLAY_DATA_MAGNETOMETER,
    DISPLAY_DATA_GYROSCOPE,
    UNCALIBRATED_DISPLAY,
    SETTINGS_DISPLAY,
    IDEAL_DATA_ODR, 
    DISPLAY_DATA_ODR, 
    DISPLAY_DATA_IMU_ATAN2F, 
} DATA_NOTIFY_BIT_FLAGS_t;

#endif
