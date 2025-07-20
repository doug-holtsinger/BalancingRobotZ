/**
 * @brief Header file for IMU
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifndef __QDEC_H__
#define __QDEC_H__

class QDEC {
    public:
        QDEC(const struct device* const dev);
        int init();
        float get_rotation_cumulative();
        float get_rotation();
    private:
        const struct device* const qdec_dev; 
        struct sensor_trigger qdec_trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ROTATION,
        };
};

#endif

