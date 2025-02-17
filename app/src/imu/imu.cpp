#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(IMU, CONFIG_SENSOR_LOG_LEVEL);

#include "imu.h"
#include "imu_cal.h"
#include "imu_cmd.h"
#include "imu_hw.h"

#include "notify.h"
#include "logdef.h"
#include "ble_svcs.h"

#include "app_demux.h"

#include "param_store.h"
#include "param_store_ids.h"

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 4

IMU imu = IMU(DEVICE_DT_GET_ONE(st_lsm6ds3tr_c), DEVICE_DT_GET_ONE(st_lis3mdl_magn), IMU_RECORD_KEY);

void imu_update_work_handler(struct k_work *work)
{
    /* do the processing that needs to be done periodically */
    imu.update();
}

K_WORK_DEFINE(imu_update_work, imu_update_work_handler);

void imu_update_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&imu_update_work);
}

K_TIMER_DEFINE(imu_update_timer, imu_update_timer_handler, NULL);

void imu_thread(void *, void *, void *)
{
    float roll, pitch, yaw;
    int16_t roll_i, pitch_i, yaw_i;
    int ret;
    ret = imu.init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize IMU");
        return;
    }

    /* allow commands to be sent to the IMU */
    appDemuxRegisterHandler(
        std::bind( &IMU::cmd, std::ref(imu), std::placeholders::_1),
        appDemuxCmdType(IMU_CMD_t::CMD_MAX) );

    /* initial update, remaining updates are handled by the timer thread */
    imu.update();

    /* start a periodic timer to perform the update */
    k_timer_start(&imu_update_timer, K_USEC(2403), K_USEC(2403)); 

    while (true) 
    {
        imu.send_all_client_data();
        imu.get_angles(roll, pitch, yaw);
        roll_i = (int16_t)roll;
        pitch_i = (int16_t)pitch;
        yaw_i = (int16_t)yaw;
        ble_svcs_send_euler_angles(roll_i, pitch_i, yaw_i);

    	k_yield();
    }
}

K_THREAD_DEFINE(imu_tid, IMU_THREAD_STACK_SIZE,
                imu_thread, NULL, NULL, NULL,
                IMU_THREAD_PRIORITY, K_ESSENTIAL|K_FP_REGS, 0);

IMU::IMU( const struct device *const dev_accelerometer_gyroscope, const struct device *const dev_magnetometer, 
		const uint16_t param_store_id) :
    dev_accel_gyro(dev_accelerometer_gyroscope),
    dev_magn(dev_magnetometer),
    param_store(param_store_id)
{
    memset(display_data, true, sizeof(display_data));

    memset(noise_threshold_mult, NOISE_THRESHOLD_MULTIPLIER, sizeof(noise_threshold_mult));
    noise_threshold_mult[IMU_ACCELEROMETER] = NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER;

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
    imu_calibration_params_t imu_cal_params;

    if (!device_is_ready(dev_accel_gyro)) {
        LOG_ERR("Accel/Gyro is not ready");
        return rc;
    }

    if (!device_is_ready(dev_magn)) {
        LOG_ERR("Magnetometer is not ready");
        return rc;
    }

    if (set_sampling_freq() != 0) {
        return rc;
    }
    reset_calibration();
    param_store.init(&cp);
    imu_cal_params = param_store.get();
    init_params(imu_cal_params);
    return rc;
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

        cp.gyroscope_min[i] = params.gyroscope_min[i];
        cp.gyroscope_max[i] = params.gyroscope_max[i];
        cp.gyroscope_min_threshold[i] = params.gyroscope_min_threshold[i];

        cp.accelerometer_min[i] = params.accelerometer_min[i];
        cp.accelerometer_max[i] = params.accelerometer_max[i];
        cp.accelerometer_min_threshold[i] = params.accelerometer_min_threshold[i];
    }
}

void IMU::params_save()
{
    param_store.set(&cp);
}

// convert gauss to milligauss
float IMU::sensor_gauss_to_mgauss(const struct sensor_value* const val)
{
    return (float)val->val1 *1000 + (float)val->val2 / 1000;
}

// Convert ms2 sensor_value to milli-G value
int32_t IMU::sensor_ms2_to_mg(const struct sensor_value* const val_ms2)
{
    return (int32_t)(sensor_ms2_to_ug(val_ms2) / 1000);
}

int IMU::read_sensors()
{
    int rc = 0;

    /* lsm6dso accel */
    rc = sensor_sample_fetch_chan(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ);
    if (rc) 
    {
        return rc;
    }
    if (!data_hold[IMU_ACCELEROMETER])
    {
        sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ, accelerometer_sens);
    } 

    /* gyro */
    rc = sensor_sample_fetch_chan(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ);
    if (rc) 
    {
        return rc;
    }
    if (!data_hold[IMU_GYROSCOPE])
    {
        sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ, gyroscope_sens);
    } 

    /* magnetometer */
    rc = sensor_sample_fetch_chan(dev_magn, SENSOR_CHAN_ALL);
    if (rc) 
    {
        return rc;
    }
    if (!data_hold[IMU_MAGNETOMETER])
    {
        sensor_channel_get(dev_magn, SENSOR_CHAN_MAGN_XYZ, magnetometer_sens);
    }

    for (int i=0 ; i<3 ; i++)
    {
        accelerometer_uncal[i] = (uncalibrated_t)sensor_ms2_to_mg(&accelerometer_sens[i]);
        gyroscope_uncal[i] = (uncalibrated_t)sensor_rad_to_10udegrees(&gyroscope_sens[i]);
        magnetometer_uncal[i] = (uncalibrated_t)sensor_gauss_to_mgauss(&magnetometer_sens[i]);
    }

    return rc;
}

int IMU::set_sampling_freq()
{
    int rc = 0;
    struct sensor_value attr;

    attr.val1 = ACCELEROMETER_ODR;
    attr.val2 = 0;
    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
    if (rc != 0) {
        LOG_ERR("Error setting accel/gyro attr %d %d", rc, __LINE__);
        return rc;
    }

    sensor_g_to_ms2(ACCELEROMETER_RANGE, &attr);
    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_FULL_SCALE, &attr);
    if (rc != 0) {
        LOG_ERR("Error setting accel/gyro attr %d %d", rc, __LINE__);
        return rc;
    }

    attr.val1 = GYROSCOPE_ODR;
    attr.val2 = 0;
    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
    if (rc != 0) {
        LOG_ERR("Error setting accel/gyro attr %d %d", rc, __LINE__);
        return rc;
    }

    attr.val1 = GYROSCOPE_RANGE;
    attr.val2 = 0;
    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_FULL_SCALE, &attr);
    if (rc != 0) {
        LOG_ERR("Error setting accel/gyro attr %d %d", rc, __LINE__);
        return rc;
    }

    return rc;
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
    for (int i=0 ; i<3 ; i++)
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
    for (int i=0 ; i<3 ; i++)
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
    for (int i=0 ; i<3 ; i++) {
        if (i == 2)
        {
            accelerometer_bias = 1000;    // 1G bias in Z direction in milli-G units
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

        cp.gyroscope_min[i] = cp.gyroscope_max[i] = cp.gyroscope_min_threshold[i] = 0;

        cp.accelerometer_min[i] = cp.accelerometer_max[i] = cp.accelerometer_min_threshold[i] = 0;
    }
}

void IMU::calibrate_data(void)
{
    uint32_t magnetometer_diff = 0;

    // calibrate raw data values using zero offset and min thresholds.
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
	    // Convert from 10 microdegrees per second to radians per second and apply correction factor.
            gyroscope_cal[i] = gyroscope_cal_before_correction[i] * cp.gyroscope_correction / ( DEGREES_PER_RADIAN * MICRO10DEGREES_PER_DEGREE);
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

void IMU::compute_angles()
{
    if (ideal_data[IMU_ACCELEROMETER])
    {
	float axN, ayN, azN;
        AHRSptr->GetNormalizedVectors(IMU_ACCELEROMETER, axN, ayN, azN);
	if ( abs(axN) < 0.1 && abs(ayN) < 0.1 )
	{
            accelerometer_cal[X_AXIS] = accelerometer_cal[Y_AXIS] = 0.0f;
	    if (accelerometer_cal[Z_AXIS] < 0.0)
	    {
                accelerometer_cal[Z_AXIS] = -1000.0f;
	    } else {
                accelerometer_cal[Z_AXIS] = 1000.0f;
	    }
	}
	if ( abs(ayN) < 0.1 && abs(azN) < 0.1 )
	{
            accelerometer_cal[Y_AXIS] = accelerometer_cal[Z_AXIS] = 0.0f;
	    if (accelerometer_cal[X_AXIS] < 0.0)
	    {
                accelerometer_cal[X_AXIS] = -1000.0f;
	    } else {
                accelerometer_cal[X_AXIS] = 1000.0f;
	    }
	}
	if ( abs(axN) < 0.1 && abs(azN) < 0.1 )
	{
            accelerometer_cal[X_AXIS] = accelerometer_cal[Z_AXIS] = 0.0f;
	    if (accelerometer_cal[Y_AXIS] < 0.0)
	    {
                accelerometer_cal[Y_AXIS] = -1000.0f;
	    } else {
                accelerometer_cal[Y_AXIS] = 1000.0f;
	    }
	}
    }
    if (ideal_data[IMU_GYROSCOPE])
    {
        for (int i = 0 ; i < 3 ; i++) 
	{
            gyroscope_cal[i] = 0.0f;
	}
    }
    if (ideal_data[IMU_MAGNETOMETER])
    {
        for (int i = 0 ; i < 3 ; i++) 
	{
            magnetometer_cal[i] = 0.0f;
	}
    }

    // FIXME -- have AHRS accept sensor values directly
    AHRSptr->update((cp.gyroscope_enabled) ? (float)gyroscope_cal[0] : 0.0f,
		    (cp.gyroscope_enabled) ? (float)gyroscope_cal[1] : 0.0f,
		    (cp.gyroscope_enabled) ? (float)gyroscope_cal[2] : 0.0f,
		    (float)accelerometer_cal[0],
		    (float)accelerometer_cal[1],
		    (float)accelerometer_cal[2],
		    (float)magnetometer_cal[0],
		    (float)magnetometer_cal[1],
		    (float)magnetometer_cal[2]);

    AHRSptr->compute_angles(roll, pitch, yaw);
}

void IMU::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];
    uint32_t bit_flags = 0;

    // Check if BLE connected, otherwise return.
    if ( !ble_svcs_connected() ) {
            return;
    }

    // Euler Angles
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMATI PRINTF_FLOAT_FORMATI , EULER_ANGLES, PRINTF_FLOAT_VALUE2(roll), PRINTF_FLOAT_VALUEI(pitch), PRINTF_FLOAT_VALUEI(yaw));
    send_client_data(s, strlen(s));

    // Accelerometer
    if (display_data[IMU_ACCELEROMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
             ACCELEROMETER_CAL,
            (int)accelerometer_cal[0],
            (int)accelerometer_cal[1],
            (int)accelerometer_cal[2]
            );
        send_client_data(s, strlen(s));

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                 ACCELEROMETER_UNCAL,
                (int)accelerometer_uncal[0],
                (int)accelerometer_uncal[1],
                (int)accelerometer_uncal[2]
                );
            send_client_data(s, strlen(s));
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                 ACCELEROMETER_MIN_THRESHOLD,
                (int)cp.accelerometer_min_threshold[0],
                (int)cp.accelerometer_min_threshold[1],
                (int)cp.accelerometer_min_threshold[2]
                );
            send_client_data(s, strlen(s));
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
        send_client_data(s, strlen(s));

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                GYROSCOPE_UNCAL,
                (int)gyroscope_uncal[0],
                (int)gyroscope_uncal[1],
                (int)gyroscope_uncal[2]
                );
            send_client_data(s, strlen(s));
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT ,
                GYROSCOPE_MIN_THRESHOLD,
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[0]),
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[1]),
                PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[2])
            );
            send_client_data(s, strlen(s));
        }
    }


    if (settings_display)
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT7 ,
            GYRO_CORRECTION,
            PRINTF_FLOAT_VALUE7(cp.gyroscope_correction)
            );
        send_client_data(s, strlen(s));

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %u", AHRS_ALGORITHM, static_cast<int>(AHRSalgorithm));
        send_client_data(s, strlen(s));

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT ,
            ACCELEROMETER_NOISE_THRESHOLD_MULT,
            PRINTF_FLOAT_VALUE(noise_threshold_mult[IMU_ACCELEROMETER]));
        send_client_data(s, strlen(s));

	// FIXME -- send other noise thresholds for gyro and magnetometer

    }

    if (display_data[IMU_ODR])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%u " PRINTF_FLOAT_FORMAT2 ,
            ODR_HZ_ACCELEROMETER + odr_select,
            PRINTF_FLOAT_VALUE2(odr_hz[odr_select])
            );
        send_client_data(s, strlen(s));
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
        send_client_data(s, strlen(s));

        if (uncalibrated_display)
        {
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                MAGNETOMETER_UNCAL,
                (int)magnetometer_uncal[0],
                (int)magnetometer_uncal[1],
                (int)magnetometer_uncal[2]
                );
            send_client_data(s, strlen(s));
    
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
                MAGNETOMETER_MIN_THRESHOLD,
                (int)cp.magnetometer_min_threshold[0],
                (int)cp.magnetometer_min_threshold[1],
                (int)cp.magnetometer_min_threshold[2]
                );
            send_client_data(s, strlen(s));
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

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %u",
            BIT_FLAGS, bit_flags
            );
    send_client_data(s, strlen(s));

    // AHRS sends data to client
    AHRSptr->send_all_client_data(display_data, settings_display);

}

#if 0
//FIXME
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
#endif

void IMU::update(void)
{
    if (!read_sensors())
    {
	switch (calibrate_enable)
	{
            case IMU_CALIBRATE_DISABLED: 
                calibrate_data();
		break;
	    case IMU_CALIBRATE_ZERO_OFFSET:
                calibrate_zero_offset();
		break;
	    case IMU_CALIBRATE_MAGNETOMETER:
                calibrate_magnetometer();
		break;
#if 0
		//FIXME
	    case IMU_CALIBRATE_GYROSCOPE:
                calibrate_gyroscope();
		break;
#endif
	    default: break;
	}
        compute_angles();
    } else
    {
        LOG_ERR("Could not read sensors");
    }

}

void IMU::cmd(const uint8_t i_cmd)
{
    LOG_DBG("cmd %x", i_cmd);
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
            calibrate_enable = IMU_CALIBRATE_DISABLED;
            reset_calibration();
            break;
	case IMU_CMD_t::SENSOR_CALIBRATE_SAVE:
            params_save();
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
            LOG_ERR("Invalid IMU cmd %d", static_cast<uint8_t>(i_cmd));
	    break;
    }
}


