/**
 * @brief Header file for IMU_CAL
 *
 */


#ifndef __IMU_CAL_H__
#define __IMU_CAL_H__

typedef struct {
    int32_t accelerometer_min[3] = {{0}};
    int32_t accelerometer_max[3] = {{0}};
    uint32_t accelerometer_min_threshold[3] = {{0}};

    int32_t gyroscope_min[3] = {{0}};
    int32_t gyroscope_max[3] = {{0}};
    uint32_t gyroscope_min_threshold[3] = {{0}};
    float gyroscope_correction = 1.0f;
    bool gyroscope_enabled = true;
    bool magnetometer_stability = true;

    int32_t magnetometer_min[3] = {{0}};
    int32_t magnetometer_max[3] = {{0}};
    uint32_t magnetometer_min_threshold[3] = {{0}};
    int32_t magnetometer_uncal_last[3] = {{0}};
} imu_calibration_params_t;

#endif
