#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "imu.h"
#include "imu_cal.h"
#include "imu_cmd.h"
#include "imu_hw.h"
#include "notify.h"
#include "ble_svcs.h"

IMU::IMU( const struct device *const dev_accelerometer_gyroscope, const struct device *const dev_magnetometer):
    dev_accel_gyro(dev_accelerometer_gyroscope),
    dev_magn(dev_magnetometer)
{
    memset(display_data, 0, sizeof(display_data));

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

    if (!device_is_ready(dev_accel_gyro)) {
        return rc;
    }

    if (!device_is_ready(dev_magn)) {
        return rc;
    }

    if (set_sampling_freq() != 0) {
        return rc;
    }
    reset_calibration();
    return rc;
}

//  old:
//       int32_t              int32_t              float
//       uncalibrated data    calibrated data      AHRS input
//
//  next1:  DSH4
//       float         int32_t              int32_t              float
//       sensor data   uncalibrated data    calibrated data      AHRS input
//
//  next2:
//       float         float                float                float
//       sensor data   uncalibrated data    calibrated data      AHRS input
//
//  read_sensor (eventually):
//       int32_t       int32_t              int32_t              float
//       sensor data   uncalibrated data    calibrated data      AHRS input
//
// 
//
void IMU::update(void)
{
    if (!read_sensors())
    {
	// FIXME: print error if read_sensors returns non-zero value
	switch (calibrate_enable)
	{
            case IMU_CALIBRATE_DISABLED: 
                calibrate_data();
		break;
	    case IMU_CALIBRATE_ZERO_OFFSET:
                calibrate_zero_offset();
		break;
#if 0
	    case IMU_CALIBRATE_MAGNETOMETER:
                calibrate_magnetometer();
		break;
	    case IMU_CALIBRATE_GYROSCOPE:
                calibrate_gyroscope();
		break;
#endif
	    default: break;
	}
        compute_angles();
    }

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
    sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ, accelerometer_sens);

    /* gyro */
    rc = sensor_sample_fetch_chan(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ, gyroscope_sens);

    /* magnetometer */
    rc = sensor_sample_fetch_chan(dev_magn, SENSOR_CHAN_ALL);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_magn, SENSOR_CHAN_MAGN_XYZ, magnetometer_sens);

    for (int i=0 ; i<3 ; i++)
    {
        accelerometer_uncal[i] = (uncalibrated_t)sensor_value_to_float(&accelerometer_sens[i]);
        gyroscope_uncal[i] = (uncalibrated_t)sensor_value_to_float(&gyroscope_sens[i]);
        magnetometer_uncal[i] = (uncalibrated_t)sensor_value_to_float(&magnetometer_sens[i]);
    }

    return rc;
}

void IMU::compute_angles()
{
    // FIXME -- have AHRS accept sensor values directly
    AHRSptr->update(gyroscope_cal[0],
		    gyroscope_cal[1],
		    gyroscope_cal[2],
		    accelerometer_cal[0],
		    accelerometer_cal[1],
		    accelerometer_cal[2],
		    magnetometer_cal[0],
		    magnetometer_cal[1],
		    magnetometer_cal[2]);

    AHRSptr->compute_angles(roll, pitch, yaw);
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

    odr_attr.val1 = ACCELEROMETER_ODR;
    odr_attr.val2 = 0;

    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    if (rc != 0) {
        return rc;
    }

    odr_attr.val1 = GYROSCOPE_ODR;
    odr_attr.val2 = 0;

    rc = sensor_attr_set(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    if (rc != 0) {
        return rc;
    }

    return rc;
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

#if 0
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
            calibrate_enable = IMU_CALIBRATE_DISABLED;
            reset_calibration();
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

