/**
 * @brief Header file for IMU
 *
 */


#ifndef __IMU_HW_H__
#define __IMU_HW_H__

// Output Data Rate in Hz
#define AHRS_SAMPLE_FREQ	416
#define GYROSCOPE_ODR        AHRS_SAMPLE_FREQ
#define ACCELEROMETER_ODR    AHRS_SAMPLE_FREQ

// Range is in G
#define ACCELEROMETER_RANGE  2
// Gyroscope FS in units of dps
#define GYROSCOPE_RANGE	     2000

#endif
