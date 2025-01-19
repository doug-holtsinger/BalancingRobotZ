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
#include "imu_cal.h"
#include "param_store.h"

#ifndef __IMU_H__
#define __IMU_H__

#define AHRS_ALGORITHM_DEFAULT AHRS_MAHONY

#define DEGREES_PER_RADIAN 57.2957795f
#define MILLIDEGREES_PER_DEGREE 1000
#define MICRO10DEGREES_PER_DEGREE 100000
#define MICRODEGREES_PER_DEGREE 1000000

#define NOISE_THRESHOLD_MULTIPLIER 2.0f
#ifndef NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER
#define NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER 2.0f
#endif
#define NOISE_THRESHOLD_MULTIPLIER_MAX 2.0f
#define NOISE_THRESHOLD_MULTIPLIER_MIN 0.0f
#define NOISE_THRESHOLD_MULTIPLIER_INCR 0.2f

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

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
        IMU(const struct device *const dev_accelerometer_gyroscope, const struct device *const dev_magnetometer, 
			const uint16_t param_store_id);
        int init();
        void update();
        void get_angles(float& roll, float& pitch, float& yaw);
        void cmd(const uint8_t i_cmd);
        void send_all_client_data();
    private:
        int set_sampling_freq();
        void reset_calibration(void);
        void reset_calibration_threshold(void);
        void calibrate_data(void);
        void calibrate_zero_offset(void);
        void calibrate_magnetometer(void);
        void init_params(imu_calibration_params_t params);

        int read_sensors();
        void compute_angles();
        int32_t sensor_ms2_to_mg(const struct sensor_value* const val_ms2);
        float sensor_gauss_to_mgauss(const struct sensor_value* const val);

	void cmd_internal(const IMU_CMD_t i_cmd);
        void params_save();

        const struct device *const dev_accel_gyro; 
        const struct device *const dev_magn;

        AHRS* AHRSptr;

	imu_calibration_params_t cp;				// local copy of calibration params
	ParamStore<imu_calibration_params_t> param_store;       // parameter storage object

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_ALGORITHM_DEFAULT;

        float noise_threshold_mult[IMU_SENSOR_MAX+1]; // initialized in imu.cpp 

	unsigned int odr_select = 0;
        bool timestamp_valid = false;
        bool timestamp_odr_valid = false;
        int32_t timestamp = 0;
        int32_t timestamp_prev = 0;
        float odr_hz[3];
        int odr_update_cnt = 0;
        bool new_data_odr = false;

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

	IMU_SENSOR_t sensor_select = IMU_AHRS;

        bool ideal_data[IMU_SENSOR_MAX+1] = { false };
        bool data_hold[IMU_SENSOR_MAX+1] = { false };
        bool display_data[IMU_SENSOR_MAX+1];    // initialized in imu.cpp
        bool fixed_data = false;
        bool uncalibrated_display = false;
        bool settings_display = true;
};

#endif

