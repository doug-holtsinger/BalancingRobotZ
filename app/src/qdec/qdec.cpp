#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "qdec.h"

LOG_MODULE_REGISTER(QDEC, CONFIG_SENSOR_LOG_LEVEL);

#ifdef __cplusplus
extern "C" {
#endif
    // number of degrees that the wheel has turned.
    static float rotation_cumulative = 0.0;

    // number of degrees that the wheel has turned since last trigger.
    static float rotation = 0.0;

    static void qdec_trigger_handler(const struct device *dev,
                                const struct sensor_trigger *trigger)
    {
        int rc = 0;
        struct sensor_value qdec_sens;
        ARG_UNUSED(trigger);

#if 0
        rc = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ROTATION);
        if (rc)
        {
            LOG_ERR("Unable to fetch QDEC samples");
	    return;
        }
#endif

        rc = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &qdec_sens);
        if (rc)
        {
            LOG_ERR("Unable to get QDEC samples");
	    return;
        }

	rotation = sensor_value_to_float(&qdec_sens);
	rotation_cumulative += rotation;
#if 0
        LOG_DBG("qdec val1 %d val2 %d rot %f crot %f\n", qdec_sens.val1, qdec_sens.val2, 
			(double)rotation, (double)rotation_cumulative);

#endif
    }
#ifdef __cplusplus
}
#endif

QDEC::QDEC(const struct device* const dev):
    qdec_dev(dev)
{
}

int QDEC::init()
{
    int rc = 0;

    if (sensor_trigger_set(qdec_dev, &qdec_trig, qdec_trigger_handler)) {
        LOG_ERR("Could not set QDEC trigger\n");
        return 1;
    }

    return rc;
}

float QDEC::get_rotation_cumulative()
{
	float rotation_cumulative_ret = rotation_cumulative;
	rotation_cumulative = 0.0;
	return rotation_cumulative_ret;
}

float QDEC::get_rotation()
{
	float rotation_ret = rotation;
	rotation = 0.0;
	return rotation_ret;
}

