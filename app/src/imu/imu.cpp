#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "imu.h"

IMU::IMU() :
    dev_lsm6(DEVICE_DT_GET_ONE(st_lsm6dso)),
    dev_lis3(DEVICE_DT_GET_ONE(st_lis3mdl_magn))
{
    if (AHRSalgorithm == AHRS_MAHONY)
    {
        AHRSptr = new MahonyAHRS();
    } else if (AHRSalgorithm == AHRS_MADGWICH) 
    {
        AHRSptr = new MadgwickAHRS();
    } else
    {
        AHRSptr = new SimpleAHRS();
    }
}

int IMU::init()
{
    int rc = 0;

    if (!device_is_ready(dev_lsm6)) {
        return rc;
    }

    if (!device_is_ready(dev_lis3)) {
        return rc;
    }

    if (set_sampling_freq() != 0) {
        return rc;
    }
    return rc;
}

int IMU::update()
{
    int rc = 0;

    /* lsm6dso accel */
    rc = sensor_sample_fetch_chan(dev_lsm6, SENSOR_CHAN_ACCEL_XYZ);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_lsm6, SENSOR_CHAN_ACCEL_XYZ, accelerometer_uncal);

    /* lsm6dso gyro */
    rc = sensor_sample_fetch_chan(dev_lsm6, SENSOR_CHAN_GYRO_XYZ);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_lsm6, SENSOR_CHAN_GYRO_XYZ, gyroscope_uncal);

    /* lis3mdl magnetometer */
    rc = sensor_sample_fetch_chan(dev_lis3, SENSOR_CHAN_ALL);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_lis3, SENSOR_CHAN_MAGN_XYZ, magnetometer_uncal);

    return rc; 
}

void IMU::AHRSCompute()
{
    ax = sensor_value_to_float(&accelerometer_uncal[0]);
    ay = sensor_value_to_float(&accelerometer_uncal[1]);
    az = sensor_value_to_float(&accelerometer_uncal[2]);

    gx = sensor_value_to_float(&gyroscope_uncal[0]);
    gy = sensor_value_to_float(&gyroscope_uncal[1]);
    gz = sensor_value_to_float(&gyroscope_uncal[2]);

    mx = sensor_value_to_float(&magnetometer_uncal[0]);
    my = sensor_value_to_float(&magnetometer_uncal[1]);
    mz = sensor_value_to_float(&magnetometer_uncal[2]);

    AHRSptr->Update(gx, gy, gz, ax, ay, az, mx, my, mz);
    AHRSptr->ComputeAngles(roll, pitch, yaw);
}

void IMU::get_angles(float& o_roll, float& o_pitch, float& o_yaw)
{
    o_roll = roll;
    o_pitch = pitch;
    o_yaw = yaw;
}

int IMU::set_sampling_freq()
{
    int rc = 0;
    struct sensor_value odr_attr;

    /* set accel/gyro sampling frequency to 12.5 Hz */
    odr_attr.val1 = 12.5;
    odr_attr.val2 = 0;

    rc = sensor_attr_set(dev_lsm6, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    if (rc != 0) {
        return rc;
    }

    rc = sensor_attr_set(dev_lsm6, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    if (rc != 0) {
        return rc;
    }

    return rc;
}

#if 0

#ifdef BLE_CONSOLE_AVAILABLE
#include "bsp.h"
#endif
#include "imu.h"
#include "imu_cal.h"
#include "imu_cmd.h"
#include "AHRS.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
#include "SimpleAHRS.h"
#include "notify.h"
#ifdef BLE_CONSOLE_AVAILABLE
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// Nordic I2C
#include "nrfx_twi.h"

#include "param_store.h"

#ifdef __cplusplus
extern "C" {
#endif
extern void fds_evt_handler_C(fds_evt_t const * p_evt);
#ifdef __cplusplus
}
#endif


IMU::IMU() :
    param_store(IMU_RECORD_KEY)
{
    std::fill(display_data, display_data + IMU_SENSOR_MAX, true);
    std::fill(noise_threshold_mult, noise_threshold_mult + IMU_SENSOR_MAX, NOISE_THRESHOLD_MULTIPLIER);
    for (int i=0 ; i < 3 ; i++)
    {
        accelerometer_uncal[i] = 0;
        accelerometer_cal[i] = 0;
        gyroscope_uncal[i] = 0;
        gyroscope_cal_before_correction[i] = 0;
        gyroscope_cal_before_correction_abs[i] = 0;
        gyroscope_cal[i] = 0.0;
        magnetometer_uncal[i] = 0;
        magnetometer_cal[i] = 0;
    }

    noise_threshold_mult[IMU_ACCELEROMETER] = NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER;
    dev_i2c = new TwoWire();
    AccGyr = new LSM6DS3Sensor(dev_i2c, TWI_ADDRESS_LSM6DS3);
    if (AHRSalgorithm == AHRS_MAHONY)
    {
        AHRSptr = new MahonyAHRS();
    } else if (AHRSalgorithm == AHRS_MADGWICH) 
    {
        AHRSptr = new MadgwickAHRS();
    } else
    {
        AHRSptr = new SimpleAHRS();
    }
    Magneto = new LIS3MDLSensor(dev_i2c, TWI_ADDRESS_LIS3MDL);
}

void IMU::init(void)
{
    imu_calibration_params_t imu_cal_params;
    sensor_init();
    reset_calibration();
    // Initialize AHRS algorithm
    // Initialize param storage device
    param_store.init(&cp);
    // Read parameters from storage and initialize local copy
    imu_cal_params = param_store.get();
    // params_print(imu_cal_params);
    init_params(imu_cal_params);
}

void IMU::params_save()
{
    // params_print(cp);
    param_store.set(&cp);
}

void IMU::params_print(imu_calibration_params_t& params)
{
    NRF_LOG_INFO("IMU Parameters");
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Mag min %d 0x%04x", i, params.magnetometer_min[i]);
        //NRF_LOG_INFO("  Mag max %d 0x%04x", i, params.magnetometer_max[i]);
        NRF_LOG_INFO("  Mag min %d thresh 0x%04x", i, params.magnetometer_min_threshold[i]);
    }
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Gyr min %d 0x%04x", i, params.gyroscope_min[i]);
        NRF_LOG_INFO("  Gyr max %d 0x%04x", i, params.gyroscope_max[i]);
        // NRF_LOG_INFO("  Gyr min thresh %d %s" PRINTF_FLOAT_FORMAT PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[i]) );
    }
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Acc min %d 0x%04x", i, params.accelerometer_min[i]);
        //NRF_LOG_INFO("  Acc max %d 0x%04x", i, params.accelerometer_max[i]);
        NRF_LOG_INFO("  Acc min thresh %d 0x%04x", i, params.accelerometer_min_threshold[i]);
    }
}

void IMU::init_params(imu_calibration_params_t params)
{
    // FIXME -- reset AHRS settings, like gyro sensitivity, as well
    cp.gyroscope_correction = params.gyroscope_correction;
    cp.gyroscope_enabled = params.gyroscope_enabled;
    cp.magnetometer_stability = params.magnetometer_stability;
    for (int i = 0 ; i < 3 ; i++)
    {
        cp.magnetometer_min[i] = params.magnetometer_min[i];
        cp.magnetometer_max[i] = params.magnetometer_max[i];
        cp.magnetometer_min_threshold[i] = params.magnetometer_min_threshold[i];
        cp.magnetometer_uncal_last[i] = params.magnetometer_uncal_last[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.gyroscope_min[i] = params.gyroscope_min[i];
        cp.gyroscope_max[i] = params.gyroscope_max[i];
        cp.gyroscope_min_threshold[i] = params.gyroscope_min_threshold[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.accelerometer_min[i] = params.accelerometer_min[i];
        cp.accelerometer_max[i] = params.accelerometer_max[i];
        cp.accelerometer_min_threshold[i] = params.accelerometer_min_threshold[i];
    }
}

void IMU::get_params(imu_calibration_params_t& params)
{

    for (int i = 0 ; i < 3 ; i++)
    {
        params.magnetometer_min[i] = cp.magnetometer_min[i];
        params.magnetometer_max[i] = cp.magnetometer_max[i];
        params.magnetometer_min_threshold[i] = cp.magnetometer_min_threshold[i];
        params.magnetometer_uncal_last[i] = cp.magnetometer_uncal_last[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        params.gyroscope_min[i] = cp.gyroscope_min[i];
        params.gyroscope_max[i] = cp.gyroscope_max[i];
        params.gyroscope_min_threshold[i] = cp.gyroscope_min_threshold[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        params.accelerometer_min[i] = cp.accelerometer_min[i];
        params.accelerometer_max[i] = cp.accelerometer_max[i];
        params.accelerometer_min_threshold[i] = cp.accelerometer_min_threshold[i];
    }
}

// FIXME -- pull this into BLE library?
void IMU::send_client_data(char *p)
{
#ifdef BLE_CONSOLE_AVAILABLE
    {
        uint8_t *p_data = (uint8_t *)p;
        ble_svcs_send_client_notification(p_data, strlen(p));
    }
#endif
#ifdef SERIAL_CONSOLE_AVAILABLE
    {
        char s[NOTIFY_PRINT_STR_MAX_LEN];
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%s\r\n", p);
        puts(s);
    }
#endif
}

void IMU::calibrate_gyroscope(void)
{
    uint32_t timer_diff;
    float rotation_rate;

    //
    //  gyroscope calibration -- done while rotating in Z axis
    //

    timer_diff = timestamp - timestamp_prev;
    if (timestamp_valid && timer_diff != 0 && timestamp_prev != 0)
    {
        // estimated rotation rate in milli-degrees per second
	rotation_rate = (1000.0/timer_diff) * TIMER_TICKS_PER_SECOND * abs(yaw - yaw_last_cal); 
        // Estimated rotation rate mdps = gyro uncal (mdps)  * Correction Factor (unitless)
	if (gyroscope_cal_before_correction_abs[2] != 0)
	{
            float gyro_correct = rotation_rate / gyroscope_cal_before_correction_abs[2];
	    //FIXME -- how to set this right?
	    if (gyro_correct > 0.0 && gyro_correct < cp.gyroscope_correction)
	    {
                cp.gyroscope_correction = gyro_correct;
	    }
	}
    }

    if (timestamp_valid)
    {
        yaw_last_cal = yaw;
        timestamp_prev = timestamp;
    }
    timestamp_valid = false;

}

void IMU::calibrate_magnetometer(void)
{
    //
    //  magnetometer calibration -- done while rotating in X / Y /  Z axis
    //
    for (int i = 0 ; i < 3 ; i++) {
        if (cp.magnetometer_min[i] == 0 && cp.magnetometer_max[i] == 0)
        {
            cp.magnetometer_min[i] = cp.magnetometer_max[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] < cp.magnetometer_min[i])
        {
            cp.magnetometer_min[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] > cp.magnetometer_max[i])
        {
            cp.magnetometer_max[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        }
    }
}

void IMU::calibrate_zero_offset(void)
{
    uint32_t noise_threshold;
    int32_t accelerometer_bias;

    //
    // magnetometer calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++)
    {
        noise_threshold = noise_threshold_mult[IMU_MAGNETOMETER] * abs( magnetometer_uncal[i] - cp.magnetometer_uncal_last[i]);
        if (noise_threshold > cp.magnetometer_min_threshold[i])
        {
            cp.magnetometer_min_threshold[i] = noise_threshold;
        }
    }

    //
    // gyroscope calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++)
    {
        if (cp.gyroscope_min[i] == 0 && cp.gyroscope_max[i] == 0)
        {
            cp.gyroscope_min[i] = cp.gyroscope_max[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] < cp.gyroscope_min[i])
        {
            cp.gyroscope_min[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] > cp.gyroscope_max[i])
        {
            cp.gyroscope_max[i] = gyroscope_uncal[i];
        }
        noise_threshold = noise_threshold_mult[IMU_GYROSCOPE] * abs ( gyroscope_uncal[i] - ((cp.gyroscope_max[i] + cp.gyroscope_min[i]) / 2) ); 
        if (noise_threshold > cp.gyroscope_min_threshold[i])
        {
            cp.gyroscope_min_threshold[i] = noise_threshold;
        }
    }

    //
    // accelerometer calibration -- done while motionless with
    // IMU Z axis pointing up.
    //
    for (int i = 0 ; i < 3 ; i++) {
        if (i == 2)
        {
            accelerometer_bias = 1000;    // 1G bias in Z direction
        } else
        {
            accelerometer_bias = 0;
        }
        if (cp.accelerometer_min[i] == 0 && cp.accelerometer_max[i] == 0)
        {
            cp.accelerometer_min[i] = cp.accelerometer_max[i] = (accelerometer_uncal[i] - accelerometer_bias);
        } else if ((accelerometer_uncal[i] - accelerometer_bias) < cp.accelerometer_min[i])
        {
            cp.accelerometer_min[i] = accelerometer_uncal[i] - accelerometer_bias;
        } else if ((accelerometer_uncal[i] - accelerometer_bias) > cp.accelerometer_max[i])
        {
            cp.accelerometer_max[i] = accelerometer_uncal[i] - accelerometer_bias;
        }
        noise_threshold = noise_threshold_mult[IMU_ACCELEROMETER] * abs( accelerometer_uncal[i] - ((cp.accelerometer_max[i] + cp.accelerometer_min[i]) / 2) - accelerometer_bias);
        if (noise_threshold > cp.accelerometer_min_threshold[i])
        {
            cp.accelerometer_min_threshold[i] = noise_threshold;
        }
    }
}

void IMU::reset_calibration_threshold(void)
{
    for (int i = 0 ; i < 3 ; i++)
    {
        cp.accelerometer_min_threshold[i] = cp.magnetometer_min_threshold[i] = cp.gyroscope_min_threshold[i] = 0;
    }
}

void IMU::reset_calibration(void)
{
    // FIXME -- reset AHRS settings, like gyro sensitivity, as well
    for (int i = 0 ; i < 3 ; i++)
    {
        cp.magnetometer_min[i] = cp.magnetometer_max[i] = cp.magnetometer_min_threshold[i] = 0;
        cp.magnetometer_uncal_last[i] = 0;
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.gyroscope_min[i] = cp.gyroscope_max[i] = cp.gyroscope_min_threshold[i] = 0;
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.accelerometer_min[i] = cp.accelerometer_max[i] = cp.accelerometer_min_threshold[i] = 0;
    }
    calibrate_reset = false;
}

void IMU::calibrate_data(void)
{
    uint32_t magnetometer_diff = 0;

    // calibrate raw data values using zero offset and Min thresholds.
    for (int i = 0 ; i < 3 ; i++) {
        accelerometer_cal[i] = accelerometer_uncal[i] - ((cp.accelerometer_max[i] + cp.accelerometer_min[i]) / 2);
        if ((uint32_t)abs(accelerometer_cal[i]) < cp.accelerometer_min_threshold[i])
        {
            accelerometer_cal[i] = 0;
        }
        gyroscope_cal_before_correction[i] = ( gyroscope_uncal[i] - ((cp.gyroscope_max[i] + cp.gyroscope_min[i]) / 2) );
	gyroscope_cal_before_correction_abs[i] = abs(gyroscope_cal_before_correction[i]);
        if ( gyroscope_cal_before_correction_abs[i] < cp.gyroscope_min_threshold[i] )
        {
            gyroscope_cal[i] = 0;
        } else {
	    // Convert from milldegrees per second to radians per second and apply correction factor.
            gyroscope_cal[i] = gyroscope_cal_before_correction[i] * cp.gyroscope_correction / ( DEGREES_PER_RADIAN * MILLIDEGREES_PER_DEGREE);
	}

        magnetometer_diff = abs(magnetometer_uncal[i] - cp.magnetometer_uncal_last[i]);
        if (cp.magnetometer_stability && magnetometer_diff < cp.magnetometer_min_threshold[i])
        {
            magnetometer_cal[i] = cp.magnetometer_uncal_last[i] - ((cp.magnetometer_max[i] + cp.magnetometer_min[i]) / 2);
        } else
        {
            magnetometer_cal[i] = magnetometer_uncal[i] - ((cp.magnetometer_max[i] + cp.magnetometer_min[i]) / 2);
        }
        cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];

    }

}

void IMU::get_angles(float& o_roll, float& o_pitch, float& o_yaw)
{
    o_roll = roll;
    o_pitch = pitch;
    o_yaw = yaw;
}

void IMU::AHRSCompute()
{
    if (cp.gyroscope_enabled)
    {
        gx = gyroscope_cal[0];
        gy = gyroscope_cal[1];
        gz = gyroscope_cal[2];
    } else
    {
        gx = gy = gz = 0.0f;
    }
    ax = (float)accelerometer_cal[0];
    ay = (float)accelerometer_cal[1];
    az = (float)accelerometer_cal[2];
    mx = (float)magnetometer_cal[0];
    my = (float)magnetometer_cal[1];
    mz = (float)magnetometer_cal[2];
    if (ideal_data[IMU_ACCELEROMETER])
    {
	float axN, ayN, azN;
        AHRSptr->GetNormalizedVectors(IMU_ACCELEROMETER, axN, ayN, azN);
	if ( abs(axN) < 0.1 && abs(ayN) < 0.1 )
	{
            ax = ay = 0.0f;
	    if (az < 0.0)
	    {
                az = -1000.0f;
	    } else {
                az = 1000.0f;
	    }
	}
	if ( abs(ayN) < 0.1 && abs(azN) < 0.1 )
	{
            ay = az = 0.0f;
	    if (ax < 0.0)
	    {
                ax = -1000.0f;
	    } else {
                ax = 1000.0f;
	    }
	}
	if ( abs(axN) < 0.1 && abs(azN) < 0.1 )
	{
            ax = az = 0.0f;
	    if (ay < 0.0)
	    {
                ay = -1000.0f;
	    } else {
                ay = 1000.0f;
	    }
	}
    }
    if (ideal_data[IMU_GYROSCOPE])
    {
        gx = 0.0f; gy = 0.0f ; gz = 0.0f;
    }
    if (ideal_data[IMU_MAGNETOMETER])
    {
        mx = 400.0f; my = 0.0f ; mz = 0.0f;
    }
    AHRSptr->Update(gx, gy, gz, ax, ay, az, mx, my, mz);
    AHRSptr->ComputeAngles(roll, pitch, yaw);
    if (fixed_data)
    {
        roll = 45.0;
        pitch = 90.0;
        yaw = 180.0;
    }
}

void IMU::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];
    uint32_t bit_flags = 0;

#if defined(BLE_CONSOLE_AVAILABLE) && !defined(SERIAL_CONSOLE_AVAILABLE)
    // Check if BLE connected, otherwise return.
    if ( !ble_svcs_connected() ) {
            return;
    }
#endif


    // Euler Angles
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMATI PRINTF_FLOAT_FORMATI , EULER_ANGLES, PRINTF_FLOAT_VALUE2(roll), PRINTF_FLOAT_VALUEI(pitch), PRINTF_FLOAT_VALUEI(yaw));
    send_client_data(s);

    // Accelerometer
    if (display_data[IMU_ACCELEROMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
             ACCELEROMETER_CAL,
            (int)accelerometer_cal[0],
            (int)accelerometer_cal[1],
            (int)accelerometer_cal[2]
            );
        send_client_data(s);

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                 ACCELEROMETER_UNCAL,
                (int)accelerometer_uncal[0],
                (int)accelerometer_uncal[1],
                (int)accelerometer_uncal[2]
                );
            send_client_data(s);
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                 ACCELEROMETER_MIN_THRESHOLD,
                (int)cp.accelerometer_min_threshold[0],
                (int)cp.accelerometer_min_threshold[1],
                (int)cp.accelerometer_min_threshold[2]
                );
            send_client_data(s);
        }
    }


    // Gyroscope
    if (display_data[IMU_GYROSCOPE])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 ,
            GYROSCOPE_CAL,
            PRINTF_FLOAT_VALUE2(gyroscope_cal[0]),
            PRINTF_FLOAT_VALUE2(gyroscope_cal[1]),
            PRINTF_FLOAT_VALUE2(gyroscope_cal[2])
        );
        send_client_data(s);

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                GYROSCOPE_UNCAL,
                (int)gyroscope_uncal[0],
                (int)gyroscope_uncal[1],
                (int)gyroscope_uncal[2]
                );
            send_client_data(s);
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT ,
                GYROSCOPE_MIN_THRESHOLD,
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[0]),
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[1]),
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[2])
            );
            send_client_data(s);
        }
    }


    if (settings_display)
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT7 ,
            GYRO_CORRECTION,
            PRINTF_FLOAT_VALUE7(cp.gyroscope_correction)
            );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %u", AHRS_ALGORITHM, static_cast<int>(AHRSalgorithm));
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT ,
            ACCELEROMETER_NOISE_THRESHOLD_MULT,
            PRINTF_FLOAT_VALUE(noise_threshold_mult[IMU_ACCELEROMETER]));
        send_client_data(s);

	// FIXME -- send other noise thresholds for gyro and magnetometer

    }

    if (display_data[IMU_ODR])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%u " PRINTF_FLOAT_FORMAT2 ,
            ODR_HZ_ACCELEROMETER + odr_select,
            PRINTF_FLOAT_VALUE2(odr_hz[odr_select])
            );
        send_client_data(s);
    }


    // Magnetometer
    if (display_data[IMU_MAGNETOMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
            MAGNETOMETER_CAL,
            (int)magnetometer_cal[0],
            (int)magnetometer_cal[1],
            (int)magnetometer_cal[2]
            );
        send_client_data(s);

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                MAGNETOMETER_UNCAL,
                (int)magnetometer_uncal[0],
                (int)magnetometer_uncal[1],
                (int)magnetometer_uncal[2]
                );
            send_client_data(s);
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                MAGNETOMETER_MIN_THRESHOLD,
                (int)cp.magnetometer_min_threshold[0],
                (int)cp.magnetometer_min_threshold[1],
                (int)cp.magnetometer_min_threshold[2]
                );
            send_client_data(s);
        }
    
    }


    bit_flags |= (cp.magnetometer_stability     ? 1 << MAGNETOMETER_STABILITY : 0);
    bit_flags |= (cp.gyroscope_enabled          ? 1 << GYROSCOPE_ENABLE : 0);
    bit_flags |= (data_hold[IMU_ACCELEROMETER]  ? 1 << DATA_HOLD_ACCELEROMETER : 0);
    bit_flags |= (data_hold[IMU_MAGNETOMETER]   ? 1 << DATA_HOLD_MAGNETOMETER : 0);
    bit_flags |= (data_hold[IMU_GYROSCOPE]      ? 1 << DATA_HOLD_GYROSCOPE : 0);
    bit_flags |= (ideal_data[IMU_ACCELEROMETER] ? 1 << IDEAL_DATA_ACCELEROMETER : 0);
    bit_flags |= (ideal_data[IMU_MAGNETOMETER]  ? 1 << IDEAL_DATA_MAGNETOMETER : 0);
    bit_flags |= (ideal_data[IMU_GYROSCOPE]     ? 1 << IDEAL_DATA_GYROSCOPE : 0);
    bit_flags |= (display_data[IMU_AHRS]          ? 1 << DISPLAY_DATA_AHRS: 0);
    bit_flags |= (display_data[IMU_ACCELEROMETER] ? 1 << DISPLAY_DATA_ACCELEROMETER : 0);
    bit_flags |= (display_data[IMU_MAGNETOMETER]  ? 1 << DISPLAY_DATA_MAGNETOMETER : 0);
    bit_flags |= (display_data[IMU_GYROSCOPE]     ? 1 << DISPLAY_DATA_GYROSCOPE : 0);
    bit_flags |= (uncalibrated_display            ? 1 << UNCALIBRATED_DISPLAY : 0); 
    bit_flags |= (settings_display                ? 1 << SETTINGS_DISPLAY : 0); 
    bit_flags |= (ideal_data[IMU_ODR]             ? 1 << IDEAL_DATA_ODR : 0); 
    bit_flags |= (display_data[IMU_ODR]           ? 1 << DISPLAY_DATA_ODR : 0); 

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %lu",
            BIT_FLAGS, bit_flags
            );
    send_client_data(s);

    // AHRS sends data to client
    AHRSptr->send_all_client_data(display_data, settings_display);

}

void IMU::MeasureODR()
{
    bool timestamp_overflow;
    if (odr_update_cnt == ODR_UPDATE_INTERVAL)
    {
        LSM6DS3StatusTypeDef lsm_err_code = AccGyr->Read_Timestamp(&timestamp, &timestamp_overflow);
        APP_ERROR_CHECK(lsm_err_code);
	if (timestamp_overflow) 
	{
	    // Reset timestamp
            LSM6DS3StatusTypeDef lsm_err_code = AccGyr->Reset_Timestamp();
            APP_ERROR_CHECK(lsm_err_code);
	    odr_update_cnt = 0;
	    timestamp_odr_valid = false;
	} else if (timestamp_odr_valid)
        {
            // FIXME  LSM6DS3 timestamp sometimes runs backwards. Bug?
            if (timestamp > timestamp_prev) 
            {
                odr_hz[odr_select] = 1.0f / (SECONDS_PER_TIMER_TICK * (timestamp - timestamp_prev));
	    }
	    odr_update_cnt = 0;
	    timestamp_odr_valid = false;
	    odr_select = odr_select + 1;
	    if (odr_select == 3) 
	    {
                odr_select = 0;
	    }
        } else {
            timestamp_prev = timestamp;
	    timestamp_odr_valid = true;
        }
    } else {
        odr_update_cnt++;
    }
}

bool IMU::read_sensor_values()
{
    LSM6DS3StatusTypeDef lsm_err_code = LSM6DS3_STATUS_OK;
    LIS3MDLStatusTypeDef lis_err_code = LIS3MDL_STATUS_OK;
    uint8_t reg_data = 0;
    bool new_data_avail = false;

    new_data_odr = false;
    timestamp_valid = false;

    //
    // Read LSM6DS3
    //
    lsm_err_code = AccGyr->ReadReg(LSM6DS3_ACC_GYRO_STATUS_REG, &reg_data);
    APP_ERROR_CHECK(lsm_err_code);

    // FIXME -- can combine status of Accel and Gyro according to app note
    // Gyro status lags Accel status.
    if (!data_hold[IMU_ACCELEROMETER] && (reg_data & LSM6DS3_ACC_GYRO_XLDA_MASK) == LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL)
    {
        new_data_avail = true;
#ifndef DISABLE_MEASURE_ODR
	if (odr_select == 0)
	{
            new_data_odr = true;
	}
#endif
        lsm_err_code = AccGyr->Get_X_Axes(accelerometer_uncal);
        APP_ERROR_CHECK(lsm_err_code);
    }
    if (!data_hold[IMU_GYROSCOPE] && (reg_data & LSM6DS3_ACC_GYRO_GDA_MASK) == LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL)
    {
        new_data_avail = true;
#ifndef DISABLE_MEASURE_ODR
	if (odr_select == 1)
	{
            new_data_odr = true;
	}
#endif
        lsm_err_code = AccGyr->Get_G_Axes(gyroscope_uncal);
        APP_ERROR_CHECK(lsm_err_code);
        if (calibrate_enable == IMU_CALIBRATE_GYROSCOPE)
        {
            lsm_err_code = AccGyr->Read_Timestamp(&timestamp, nullptr);
            APP_ERROR_CHECK(lsm_err_code);
            timestamp_valid = true;
        }
    }

    //
    // Read LIS3MDL
    //
    if (!data_hold[IMU_MAGNETOMETER])
    {
	bool magNewData;
        lis_err_code = Magneto->NewDataAvailable(&magNewData);
        APP_ERROR_CHECK(lis_err_code);
	if (magNewData)
	{
            lis_err_code = Magneto->GetAxes(magnetometer_uncal);
            APP_ERROR_CHECK(lis_err_code);
#ifndef DISABLE_MEASURE_ODR
	    if (odr_select == 2)
	    {
                new_data_odr = true;
	    }
#endif
	}
    }

    return new_data_avail;
}

void IMU::update()
{
    if (calibrate_reset)
    {
        reset_calibration();
    } else if (read_sensor_values())
    {
        if (calibrate_enable == IMU_CALIBRATE_ZERO_OFFSET)
        {
            calibrate_zero_offset();
        }
        if (calibrate_enable == IMU_CALIBRATE_MAGNETOMETER)
        {
            calibrate_magnetometer();
        }
        if (calibrate_enable == IMU_CALIBRATE_GYROSCOPE)
        {
            calibrate_gyroscope();
        }
#ifndef DISABLE_MEASURE_ODR
        if (new_data_odr)
	{
	   MeasureODR();
	}	
#endif
	if (!ideal_data[IMU_ODR]) 
	{
            calibrate_data();
            AHRSCompute();
	}
    }

}

/**
 */
void IMU::sensor_init(void)
{
    AccGyr->Enable_X();
    AccGyr->Enable_G();
    AccGyr->Reset_Timestamp();
    AccGyr->Enable_Timestamp();
    // FIXME -- what to do here?
    // AccGyr->Enable_G_Filter(LSM6DS3_ACC_GYRO_HPCF_G_16Hz32);
    // AccGyr->Enable_6D_Orientation();

    Magneto->Enable();
}

void IMU::cmd(const uint8_t i_cmd)
{
    cmd_internal(static_cast<IMU_CMD_t>(i_cmd));
}

void IMU::cmd_internal(const IMU_CMD_t i_cmd)
{
    switch (i_cmd)
    {
        case IMU_CMD_t::SELECT_MAGNETOMETER:
            // magnetometer
            sensor_select = IMU_MAGNETOMETER;
            break;
	case IMU_CMD_t::SELECT_GYROSCOPE:
            // gyro
            sensor_select = IMU_GYROSCOPE;
            break;
	case IMU_CMD_t::SELECT_ACCELEROMETER:
            // accelerometer
            sensor_select = IMU_ACCELEROMETER;
            break;
	case IMU_CMD_t::SELECT_AHRS:
            sensor_select = IMU_AHRS;
            break;
	case IMU_CMD_t::SELECT_ODR:
            sensor_select = IMU_ODR;
            break;
	case IMU_CMD_t::AHRS_INPUT_TOGGLE:
            show_input_ahrs = ( show_input_ahrs + 1 ) % 4;
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_NORMALIZED:
            calibrate_enable = IMU_CALIBRATE_DISABLED; 
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_ZERO_OFFSET:
            calibrate_enable = IMU_CALIBRATE_ZERO_OFFSET; 
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_MAGNETOMETER:
            calibrate_enable = IMU_CALIBRATE_MAGNETOMETER; 
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_GYROSCOPE:
            calibrate_enable = IMU_CALIBRATE_GYROSCOPE; 
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_RESET:
            // reset calibration values
            calibrate_reset = true;
            calibrate_enable = IMU_CALIBRATE_DISABLED;
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_SAVE:
            params_save();
            break;
	case IMU_CMD_t::AHRS_YAW_TOGGLE:
            show_yaw = !show_yaw;
            break;
	case IMU_CMD_t::AHRS_PITCH_TOGGLE:
            show_pitch = !show_pitch;
            break;
	case IMU_CMD_t::AHRS_ROLL_TOGGLE:
            show_roll = !show_roll;
            break;
	case IMU_CMD_t::SENSOR_DATA_HOLD_TOGGLE:
	    if (sensor_select >= IMU_SENSOR_MIN && sensor_select <= IMU_SENSOR_MAX)
	    {
                data_hold[sensor_select] = !data_hold[sensor_select];
	    }
            break;
	case IMU_CMD_t::SENSOR_DATA_IDEAL_TOGGLE:
	    if (sensor_select >= IMU_SENSOR_MIN && sensor_select <= IMU_SENSOR_MAX)
	    {
                ideal_data[sensor_select] = !ideal_data[sensor_select];
	    }
            break;
	case IMU_CMD_t::SENSOR_DATA_DISPLAY_TOGGLE:
	    if (sensor_select >= IMU_SENSOR_MIN && sensor_select <= IMU_SENSOR_MAX)
	    {
                display_data[sensor_select] = !display_data[sensor_select];
	    }
            break;
	case IMU_CMD_t::SENSOR_DATA_FIXED_TOGGLE:
            fixed_data = !fixed_data;
            break;
	case IMU_CMD_t::GYROSCOPE_CORRECTION_UP:
            cp.gyroscope_correction *= 10.0f;
            break;
	case IMU_CMD_t::GYROSCOPE_CORRECTION_DOWN:
            cp.gyroscope_correction /= 10.0f;
            break;
	case IMU_CMD_t::MAGNETOMETER_STABILITY_TOGGLE:
            cp.magnetometer_stability = !cp.magnetometer_stability; 
            break;
	case IMU_CMD_t::AHRS_ALGORITHM_TOGGLE:
            if (AHRSalgorithm == AHRS_MAHONY)
            {
                AHRSptr->~AHRS();
                AHRSptr = new MadgwickAHRS();
                AHRSalgorithm = AHRS_MADGWICH;
	    } else if (AHRSalgorithm == AHRS_MADGWICH)
	    {
                AHRSptr->~AHRS();
                AHRSptr = new SimpleAHRS();
                AHRSalgorithm = AHRS_SIMPLE;
            } else 
            {
                AHRSptr->~AHRS();
                AHRSptr = new MahonyAHRS();
                AHRSalgorithm = AHRS_MAHONY;
            }
            break;
	case IMU_CMD_t::GYROSCOPE_ENABLE_TOGGLE:
            cp.gyroscope_enabled = !cp.gyroscope_enabled; 
            break;
	case IMU_CMD_t::UNCALIBRATED_DISPLAY_TOGGLE:
            uncalibrated_display = !uncalibrated_display; 
            break;
	case IMU_CMD_t::SETTINGS_DISPLAY_TOGGLE:
            settings_display = !settings_display; 
            break;
	case IMU_CMD_t::NOISE_THRESHOLD_UP:
	    noise_threshold_mult[sensor_select] += NOISE_THRESHOLD_MULTIPLIER_INCR;
	    if ( noise_threshold_mult[sensor_select] > NOISE_THRESHOLD_MULTIPLIER_MAX ) 
	    {
	        noise_threshold_mult[sensor_select] = NOISE_THRESHOLD_MULTIPLIER_MAX;
	    } 
            reset_calibration_threshold();
            break;
	case IMU_CMD_t::NOISE_THRESHOLD_DOWN:
	    noise_threshold_mult[sensor_select] -= NOISE_THRESHOLD_MULTIPLIER_INCR;
	    if ( noise_threshold_mult[sensor_select] < NOISE_THRESHOLD_MULTIPLIER_MIN ) 
	    {
	        noise_threshold_mult[sensor_select] = NOISE_THRESHOLD_MULTIPLIER_MIN;
	    } 
            reset_calibration_threshold();
            break;
	case IMU_CMD_t::AHRS_PROP_GAIN_UP:
	case IMU_CMD_t::AHRS_PROP_GAIN_DOWN:
	case IMU_CMD_t::AHRS_INTEG_GAIN_UP:
	case IMU_CMD_t::AHRS_INTEG_GAIN_DOWN:
	case IMU_CMD_t::AHRS_SAMPLE_FREQ_UP:
	case IMU_CMD_t::AHRS_SAMPLE_FREQ_DOWN:
	case IMU_CMD_t::AHRS_BETA_GAIN_UP:
	case IMU_CMD_t::AHRS_BETA_GAIN_DOWN:
            // Fall through
            AHRSptr->cmd(i_cmd);
            break;
        default: 
            NRF_LOG_INFO("Invalid IMU cmd %d", i_cmd);
	    break;
    }
}

#endif
