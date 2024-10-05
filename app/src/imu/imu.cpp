#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "imu.h"
#include "imu_hw.h"

IMU::IMU( const struct device *const dev_accelerometer_gyroscope, const struct device *const dev_magnetometer):
    dev_accel_gyro(dev_accelerometer_gyroscope),
    dev_magn(dev_magnetometer)
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

    if (!device_is_ready(dev_accel_gyro)) {
        return rc;
    }

    if (!device_is_ready(dev_magn)) {
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
    rc = sensor_sample_fetch_chan(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_ACCEL_XYZ, accelerometer_uncal);

    /* gyro */
    rc = sensor_sample_fetch_chan(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_accel_gyro, SENSOR_CHAN_GYRO_XYZ, gyroscope_uncal);

    /* magnetometer */
    rc = sensor_sample_fetch_chan(dev_magn, SENSOR_CHAN_ALL);
    if (rc) 
    {
        return rc;
    }
    sensor_channel_get(dev_magn, SENSOR_CHAN_MAGN_XYZ, magnetometer_uncal);

    return rc; 
}

void IMU::compute()
{
    for (int i=0 ; i<3 ; i++)
    {
        accel[i] = sensor_value_to_float(&accelerometer_uncal[i]);
    }

    for (int i=0 ; i<3 ; i++)
    {
        gyro[i] = sensor_value_to_float(&gyroscope_uncal[i]);
    }

    for (int i=0 ; i<3 ; i++)
    {
        magn[i] = sensor_value_to_float(&magnetometer_uncal[i]);
    }

    AHRSptr->update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magn[0], magn[1], magn[2]); 
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

