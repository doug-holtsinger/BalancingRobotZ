/**
 * @brief Header file for IMU
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "AHRS.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
#include "SimpleAHRS.h"
#include <zephyr/device.h>

#ifndef __IMU_H__
#define __IMU_H__

#define AHRS_ALGORITHM_DEFAULT AHRS_MAHONY

typedef enum {
        IMU_CALIBRATE_DISABLED = 0,
        IMU_CALIBRATE_MIN = IMU_CALIBRATE_DISABLED,
        IMU_CALIBRATE_ZERO_OFFSET,
        IMU_CALIBRATE_MAGNETOMETER,
        IMU_CALIBRATE_GYROSCOPE,
        IMU_CALIBRATE_MAX = IMU_CALIBRATE_MAGNETOMETER
} IMU_CALIBRATE_t;

typedef enum {
	AHRS_MAHONY,
	AHRS_MADGWICH,
	AHRS_SIMPLE
} AHRS_ALGORITHM_t;

class IMU {
    public:
        IMU(const struct device *const dev_accelerometer_gyroscope, const struct device *const dev_magnetometer);
        int init();
        int update();
        void compute();
        void get_angles(float& roll, float& pitch, float& yaw);
    private:
        int set_sampling_freq();
        const struct device *const dev_accel_gyro; 
        const struct device *const dev_magn;
        struct sensor_value accelerometer_uncal[3];
        struct sensor_value gyroscope_uncal[3];
        struct sensor_value magnetometer_uncal[3];
        AHRS* AHRSptr;

	float accel[3];
	float magn[3];
	float gyro[3];

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_ALGORITHM_DEFAULT;
};

#endif

