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
#include "imu_cal.h"

#ifndef __IMU_H__
#define __IMU_H__

#define AHRS_ALGORITHM_DEFAULT AHRS_MAHONY

#define DEGREES_PER_RADIAN 57.2957795f
#define MILLIDEGREES_PER_DEGREE 1000

#define NOISE_THRESHOLD_MULTIPLIER 2.0
#ifndef NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER
#define NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER 2.0
#endif
#define NOISE_THRESHOLD_MULTIPLIER_MAX 2.0
#define NOISE_THRESHOLD_MULTIPLIER_MIN 0.0
#define NOISE_THRESHOLD_MULTIPLIER_INCR 0.2

typedef enum {
        IMU_CALIBRATE_DISABLED = 0,
        IMU_CALIBRATE_MIN = IMU_CALIBRATE_DISABLED,
        IMU_CALIBRATE_ZERO_OFFSET,
        IMU_CALIBRATE_MAGNETOMETER,
        IMU_CALIBRATE_GYROSCOPE,
        IMU_CALIBRATE_MAX = IMU_CALIBRATE_GYROSCOPE
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
        void update();
        void get_angles(float& roll, float& pitch, float& yaw);
    private:
        int set_sampling_freq();
        void reset_calibration(void);
        void reset_calibration_threshold(void);
        void calibrate_data(void);
        void calibrate_zero_offset(void);

        int read_sensors();
        void compute_angles();

        const struct device *const dev_accel_gyro; 
        const struct device *const dev_magn;

        AHRS* AHRSptr;

	imu_calibration_params_t cp;				// local copy of calibration params

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_ALGORITHM_DEFAULT;

        bool display_data[IMU_SENSOR_MAX+1];
        float noise_threshold_mult[IMU_SENSOR_MAX+1]; // initialized in imu.cpp 
        IMU_CALIBRATE_t calibrate_enable = IMU_CALIBRATE_DISABLED;

        struct sensor_value accelerometer_sens[3];
        struct sensor_value gyroscope_sens[3];
        struct sensor_value magnetometer_sens[3];

        uncalibrated_t accelerometer_uncal[3];
        calibrated_t accelerometer_cal[3];

        uncalibrated_t gyroscope_uncal[3];
        int32_t gyroscope_cal_before_correction[3];
        uint32_t gyroscope_cal_before_correction_abs[3];
        float gyroscope_cal[3];

        uncalibrated_t magnetometer_uncal[3];
        calibrated_t magnetometer_cal[3];

};

#endif

