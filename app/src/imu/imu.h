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
	IMU();
        int init();
        int update();
        void get_angles(float& roll, float& pitch, float& yaw);
    private:
        int set_sampling_freq();
        void AHRSCompute();

        const struct device *const dev_lsm6; 
        const struct device *const dev_lis3; 
        struct sensor_value accelerometer_uncal[3];
        struct sensor_value gyroscope_uncal[3];
        struct sensor_value magnetometer_uncal[3];
        AHRS* AHRSptr;

        float gx = 0.0, gy = 0.0, gz = 0.0, ax = 0.0, ay = 0.0, az = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_ALGORITHM_DEFAULT;
};

#if 0
#include "TwoWire.h"
#include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"
#include "imu_cal.h"
#include "imu_cmd.h"
#include "param_store.h"
#include "AHRS.h"
#include "logdef.h"
#include "app_config.h"

#define NOISE_THRESHOLD_MULTIPLIER 2.0
#ifndef NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER
#define NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER 2.0
#endif
#define NOISE_THRESHOLD_MULTIPLIER_MAX 2.0
#define NOISE_THRESHOLD_MULTIPLIER_MIN 0.0
#define NOISE_THRESHOLD_MULTIPLIER_INCR 0.2

#define DEGREES_PER_RADIAN 57.2957795
#define MILLIDEGREES_PER_DEGREE 1000
// FIXME: move this into the LSM6DS3 code
#define SECONDS_PER_TIMER_TICK 0.000025
#define TIMER_TICKS_PER_SECOND 40000

#define ODR_UPDATE_INTERVAL 200

#define AHRS_ALGORITHM_DEFAULT AHRS_MAHONY
#ifdef BALANCING_ROBOT_CONFIG
#define DISABLE_MEASURE_ODR
#endif

#define IMU_RECORD_KEY 0x7010

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
        IMU();
        IMU(const IMU& x) = delete;
        IMU& operator=(const IMU& x) = delete;
        void update(void);
        void init(void);
        void get_params(imu_calibration_params_t& params);
        void cmd_internal(const IMU_CMD_t i_cmd);
	// FIXME -- use template
        void cmd(const uint8_t i_cmd);
        void send_all_client_data();
        void send_client_data(char*);
        void get_angles(float& roll, float& pitch, float& yaw);
        TwoWire* dev_i2c;
    private:
        void sensor_init(void);
        void calibrate_zero_offset(void);
        void calibrate_magnetometer(void);
        void calibrate_gyroscope(void);
        void reset_calibration(void);
        void reset_calibration_threshold(void);
        void calibrate_data(void);
        void AHRSCompute(void);
        void init_params(imu_calibration_params_t params);
        void params_save();
        void params_print(imu_calibration_params_t& params);
	void MeasureODR();
        bool read_sensor_values();

        ParamStore<imu_calibration_params_t> param_store;	// parameter storage object

        LSM6DS3Sensor* AccGyr;
        LIS3MDLSensor* Magneto;
	AHRS* AHRSptr;

        IMU_CALIBRATE_t calibrate_enable = IMU_CALIBRATE_DISABLED;
        bool calibrate_reset = false;
        bool show_pitch = false;
        bool show_yaw   = false;
        bool show_roll  = false;
        bool ideal_data[IMU_SENSOR_MAX+1] = { false };
        bool data_hold[IMU_SENSOR_MAX+1] = { false };
        bool display_data[IMU_SENSOR_MAX+1]; 	// initialized in imu.cpp
	bool fixed_data = false;
	bool uncalibrated_display = false;
	bool settings_display = true;

        float roll = 0.0, pitch = 0.0, yaw = 0.0;
        float yaw_last_cal = 0.0;

	imu_calibration_params_t cp;				// local copy of calibration params

        int32_t accelerometer_uncal[3];
        int32_t accelerometer_cal[3];

        int32_t gyroscope_uncal[3];
        int32_t gyroscope_cal_before_correction[3];
        uint32_t gyroscope_cal_before_correction_abs[3];
        float gyroscope_cal[3];

        int32_t magnetometer_uncal[3];
        int32_t magnetometer_cal[3];

        float noise_threshold_mult[IMU_SENSOR_MAX+1]; // initialized in imu.cpp 

	unsigned int odr_select = 0;
        bool timestamp_valid = false;
        bool timestamp_odr_valid = false;
        int32_t timestamp = 0;
        int32_t timestamp_prev = 0;
	float odr_hz[3];
	int odr_update_cnt = 0;
        bool new_data_odr = false;

        IMU_SENSOR_t sensor_select = IMU_AHRS;
        uint32_t show_input_ahrs = 0;
        float gx = 0.0, gy = 0.0, gz = 0.0, ax = 0.0, ay = 0.0, az = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_ALGORITHM_DEFAULT;
};

#endif
#endif

