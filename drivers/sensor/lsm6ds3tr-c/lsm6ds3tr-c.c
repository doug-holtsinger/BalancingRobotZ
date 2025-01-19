
#define DT_DRV_COMPAT st_lsm6ds3tr_c

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "lsm6ds3tr-c.h"

LOG_MODULE_REGISTER(LSM6DS3TR_C, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t lsm6ds3tr_c_odr_map[] = {0, 12, 26, 52, 104, 208, 417, 833,
					1667, 3333, 6667};

static int lsm6ds3tr_c_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6ds3tr_c_odr_map); i++) {
		if (freq <= lsm6ds3tr_c_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int lsm6ds3tr_c_odr_to_freq_val(uint16_t odr)
{
	/* for valid index, return value from map */
	if (odr < ARRAY_SIZE(lsm6ds3tr_c_odr_map)) {
		return lsm6ds3tr_c_odr_map[odr];
	}

	/* invalid index, return last entry */
	return lsm6ds3tr_c_odr_map[ARRAY_SIZE(lsm6ds3tr_c_odr_map) - 1];
}

static const uint16_t lsm6ds3tr_c_accel_fs_map[] = {2, 16, 4, 8};

static int lsm6ds3tr_c_accel_range_to_fs_val(int32_t range, bool double_range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6ds3tr_c_accel_fs_map); i++) {
		if (range == (lsm6ds3tr_c_accel_fs_map[i] << double_range)) {
			return i;
		}
	}

	return -EINVAL;
}

static int lsm6ds3tr_c_accel_fs_val_to_gain(int fs, bool double_range)
{
	/* Range of ±2G has a LSB of GAIN_UNIT_XL, thus divide by 2 */
	return double_range ?
		lsm6ds3tr_c_accel_fs_map[fs] * GAIN_UNIT_XL :
		lsm6ds3tr_c_accel_fs_map[fs] * GAIN_UNIT_XL / 2;
}

static const uint16_t lsm6ds3tr_c_gyro_fs_map[] = {250, 125, 500, 0, 1000, 0, 2000};
static const uint16_t lsm6ds3tr_c_gyro_fs_sens[] = {2, 1, 4, 0, 8, 0, 16};

static int lsm6ds3tr_c_gyro_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6ds3tr_c_gyro_fs_map); i++) {
		if (range == lsm6ds3tr_c_gyro_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static inline int lsm6ds3tr_c_reboot(const struct device *dev)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (lsm6ds3tr_c_boot_set(ctx, 1) < 0) {
		return -EIO;
	}

	/* Wait sensor turn-on time as per datasheet */
	k_busy_wait(35 * USEC_PER_MSEC);

	return 0;
}

static int lsm6ds3tr_c_accel_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6ds3tr_c_data *data = dev->data;

	if (lsm6ds3tr_c_xl_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

static int lsm6ds3tr_c_accel_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6ds3tr_c_data *data = dev->data;

	if (lsm6ds3tr_c_xl_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	data->accel_freq = lsm6ds3tr_c_odr_to_freq_val(odr);

	return 0;
}

static int lsm6ds3tr_c_gyro_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (lsm6ds3tr_c_gy_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_gyro_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (lsm6ds3tr_c_gy_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_accel_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = lsm6ds3tr_c_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	LOG_DBG("accel odr is %d", odr);
	if (lsm6ds3tr_c_accel_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_accel_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct lsm6ds3tr_c_data *data = dev->data;
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	bool range_double = !!(cfg->accel_range & ACCEL_RANGE_DOUBLE);

	fs = lsm6ds3tr_c_accel_range_to_fs_val(range, range_double);
	if (fs < 0) {
		return fs;
	}

	LOG_DBG("accel range is %d", fs);
	if (lsm6ds3tr_c_accel_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->acc_gain = lsm6ds3tr_c_accel_fs_val_to_gain(fs, range_double);
	return 0;
}

static int lsm6ds3tr_c_accel_config(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6ds3tr_c_accel_range_set(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6ds3tr_c_accel_odr_set(dev, val->val1);
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6ds3tr_c_gyro_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = lsm6ds3tr_c_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	LOG_DBG("gyro odr is %d", odr);
	if (lsm6ds3tr_c_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set gyroscope sampling rate");
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_gyro_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct lsm6ds3tr_c_data *data = dev->data;

	fs = lsm6ds3tr_c_gyro_range_to_fs_val(range);
	if (fs < 0) {
		LOG_DBG("failed to find gyroscope full-scale %d %d", fs, range);
		return fs;
	}

	LOG_DBG("gyro range is %d", fs);
	if (lsm6ds3tr_c_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	data->gyro_gain = (lsm6ds3tr_c_gyro_fs_sens[fs] * GAIN_UNIT_G);
	return 0;
}

static int lsm6ds3tr_c_gyro_config(const struct device *dev,
			       enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6ds3tr_c_gyro_range_set(dev, val->val1);
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6ds3tr_c_gyro_odr_set(dev, val->val1);
	default:
		LOG_DBG("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6ds3tr_c_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return lsm6ds3tr_c_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return lsm6ds3tr_c_gyro_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6ds3tr_c_sample_fetch_accel(const struct device *dev)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6ds3tr_c_data *data = dev->data;

	if (lsm6ds3tr_c_acceleration_raw_get(ctx, data->acc) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_sample_fetch_gyro(const struct device *dev)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6ds3tr_c_data *data = dev->data;

	if (lsm6ds3tr_c_angular_rate_raw_get(ctx, data->gyro) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm6ds3tr_c_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		lsm6ds3tr_c_sample_fetch_gyro(dev);
		break;
	case SENSOR_CHAN_ALL:
		lsm6ds3tr_c_sample_fetch_accel(dev);
		lsm6ds3tr_c_sample_fetch_gyro(dev);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void lsm6ds3tr_c_accel_convert(struct sensor_value *val, int raw_val,
					 uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in ug/LSB */
	/* Convert to m/s^2 */
	dval = (int64_t)(raw_val) * sensitivity;
	sensor_ug_to_ms2(dval, val);
}

static inline int lsm6ds3tr_c_accel_get_channel(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct lsm6ds3tr_c_data *data,
					    uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lsm6ds3tr_c_accel_convert(val, data->acc[0], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lsm6ds3tr_c_accel_convert(val, data->acc[1], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lsm6ds3tr_c_accel_convert(val, data->acc[2], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (i = 0; i < 3; i++) {
			lsm6ds3tr_c_accel_convert(val++, data->acc[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6ds3tr_c_accel_channel_get(enum sensor_channel chan,
				     struct sensor_value *val,
				     struct lsm6ds3tr_c_data *data)
{
	return lsm6ds3tr_c_accel_get_channel(chan, val, data, data->acc_gain);
}

static inline void lsm6ds3tr_c_gyro_convert(struct sensor_value *val, int raw_val,
					uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in udps/LSB */
	/* So, calculate value in 10 udps unit and then to rad/s */
	dval = (int64_t)(raw_val) * sensitivity / 10;
	sensor_10udegrees_to_rad(dval, val);
}

static inline int lsm6ds3tr_c_gyro_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct lsm6ds3tr_c_data *data,
					   uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		lsm6ds3tr_c_gyro_convert(val, data->gyro[0], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		lsm6ds3tr_c_gyro_convert(val, data->gyro[1], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		lsm6ds3tr_c_gyro_convert(val, data->gyro[2], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (i = 0; i < 3; i++) {
			lsm6ds3tr_c_gyro_convert(val++, data->gyro[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6ds3tr_c_gyro_channel_get(enum sensor_channel chan,
				    struct sensor_value *val,
				    struct lsm6ds3tr_c_data *data)
{
	return lsm6ds3tr_c_gyro_get_channel(chan, val, data, data->gyro_gain);
}

static int lsm6ds3tr_c_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lsm6ds3tr_c_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm6ds3tr_c_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		lsm6ds3tr_c_gyro_channel_get(chan, val, data);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api lsm6ds3tr_c_driver_api = {
	.attr_set = lsm6ds3tr_c_attr_set,
#if CONFIG_LSM6DS3TR_C_TRIGGER
	.trigger_set = lsm6ds3tr_c_trigger_set,
#endif
	.sample_fetch = lsm6ds3tr_c_sample_fetch,
	.channel_get = lsm6ds3tr_c_channel_get,
};

static int lsm6ds3tr_c_init_chip(const struct device *dev)
{
	const struct lsm6ds3tr_c_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6ds3tr_c_data *lsm6ds3tr_c = dev->data;
	uint8_t chip_id;
	uint8_t odr, fs;

	/* All registers except 0x01 are different between banks, including the WHO_AM_I
	 * register and the register used for a SW reset.  If the lsm6ds3tr_c wasn't on the user
	 * bank when it reset, then both the chip id check and the sw reset will fail unless we
	 * set the bank now.
	 */
	if (lsm6ds3tr_c_mem_bank_set(ctx, LSM6DS3TR_C_USER_BANK) < 0) {
		LOG_DBG("Failed to set user bank");
		return -EIO;
	}

	if (lsm6ds3tr_c_device_id_get(ctx, &chip_id) < 0) {
		LOG_DBG("Failed reading chip id");
		return -EIO;
	}

	LOG_INF("chip id 0x%x", chip_id);

	if (chip_id != LSM6DS3TR_C_ID) {
		LOG_DBG("Invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	/* reset device */
	if (lsm6ds3tr_c_reset_set(ctx, 1) < 0) {
		return -EIO;
	}

	k_busy_wait(100);

	/* set accel power mode */
	LOG_DBG("accel pm is %d", cfg->accel_pm);
	switch (cfg->accel_pm) {
	default:
	case 0:
		lsm6ds3tr_c_xl_power_mode_set(ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);
		break;
	case 1:
		lsm6ds3tr_c_xl_power_mode_set(ctx, LSM6DS3TR_C_XL_NORMAL);
		break;
	}

	fs = cfg->accel_range & ACCEL_RANGE_MASK;
	LOG_DBG("accel range is %d", fs);
	if (lsm6ds3tr_c_accel_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set accelerometer range %d", fs);
		return -EIO;
	}
	lsm6ds3tr_c->acc_gain = lsm6ds3tr_c_accel_fs_val_to_gain(fs, cfg->accel_range & ACCEL_RANGE_DOUBLE);

	odr = cfg->accel_odr;
	LOG_DBG("accel odr is %d", odr);
	lsm6ds3tr_c->accel_freq = lsm6ds3tr_c_odr_to_freq_val(odr);
	if (lsm6ds3tr_c_accel_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set accelerometer odr %d", odr);
		return -EIO;
	}

	/* set gyro power mode */
	LOG_DBG("gyro pm is %d", cfg->gyro_pm);
	switch (cfg->gyro_pm) {
	default:
	case 0:
		lsm6ds3tr_c_gy_power_mode_set(ctx, LSM6DS3TR_C_GY_HIGH_PERFORMANCE);
		break;
	case 1:
		lsm6ds3tr_c_gy_power_mode_set(ctx, LSM6DS3TR_C_GY_NORMAL);
		break;
	}

	fs = cfg->gyro_range;
	LOG_DBG("gyro range is %d", fs);
	if (lsm6ds3tr_c_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set gyroscope range %d", fs);
		return -EIO;
	}
	lsm6ds3tr_c->gyro_gain = (lsm6ds3tr_c_gyro_fs_sens[fs] * GAIN_UNIT_G);

	odr = cfg->gyro_odr;
	LOG_DBG("gyro odr is %d", odr);
	lsm6ds3tr_c->gyro_freq = lsm6ds3tr_c_odr_to_freq_val(odr);
	if (lsm6ds3tr_c_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set gyroscope odr %d", odr);
		return -EIO;
	}

	/* Set FIFO bypass mode */
	if (lsm6ds3tr_c_fifo_mode_set(ctx, LSM6DS3TR_C_BYPASS_MODE) < 0) {
		LOG_DBG("failed to set FIFO mode");
		return -EIO;
	}

	if (lsm6ds3tr_c_block_data_update_set(ctx, 1) < 0) {
		LOG_DBG("failed to set BDU mode");
		return -EIO;
	}

	return 0;
}

static int lsm6ds3tr_c_init(const struct device *dev)
{
#ifdef CONFIG_LSM6DS3TR_C_TRIGGER
	const struct lsm6ds3tr_c_config *cfg = dev->config;
#endif
	struct lsm6ds3tr_c_data *data = dev->data;

	LOG_INF("Initialize device %s", dev->name);
	data->dev = dev;

	if (lsm6ds3tr_c_init_chip(dev) < 0) {
		LOG_DBG("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_LSM6DS3TR_C_TRIGGER
	if (cfg->trig_enabled) {
		if (lsm6ds3tr_c_init_interrupt(dev) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "LSM6DS3TR_C driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by LSM6DS3TR_C_DEFINE_SPI() and
 * LSM6DS3TR_C_DEFINE_I2C().
 */

#define LSM6DS3TR_C_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    lsm6ds3tr_c_init,				\
			    NULL,					\
			    &lsm6ds3tr_c_data_##inst,			\
			    &lsm6ds3tr_c_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &lsm6ds3tr_c_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_LSM6DS3TR_C_TRIGGER
#define LSM6DS3TR_C_CFG_IRQ(inst)						\
	.trig_enabled = true,						\
	.gpio_drdy = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),		\
	.int_pin = DT_INST_PROP(inst, int_pin)
#else
#define LSM6DS3TR_C_CFG_IRQ(inst)
#endif /* CONFIG_LSM6DS3TR_C_TRIGGER */

#define LSM6DS3TR_C_SPI_OP  (SPI_WORD_SET(8) |				\
			 SPI_OP_MODE_MASTER |				\
			 SPI_MODE_CPOL |				\
			 SPI_MODE_CPHA)					\

#define LSM6DS3TR_C_CONFIG_COMMON(inst)					\
	.accel_pm = DT_INST_PROP(inst, accel_pm),			\
	.accel_odr = DT_INST_PROP(inst, accel_odr),			\
	.accel_range = DT_INST_PROP(inst, accel_range) |		\
		(DT_INST_NODE_HAS_COMPAT(inst, st_lsm6ds3tr_c32) ?	        \
			ACCEL_RANGE_DOUBLE : 0),			\
	.gyro_pm = DT_INST_PROP(inst, gyro_pm),				\
	.gyro_odr = DT_INST_PROP(inst, gyro_odr),			\
	.gyro_range = DT_INST_PROP(inst, gyro_range),			\
	.drdy_pulsed = DT_INST_PROP(inst, drdy_pulsed),                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),		\
		(LSM6DS3TR_C_CFG_IRQ(inst)), ())

#define LSM6DS3TR_C_CONFIG_SPI(inst)					\
	{								\
		STMEMSC_CTX_SPI(&lsm6ds3tr_c_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   LSM6DS3TR_C_SPI_OP,		\
					   0),				\
		},							\
		LSM6DS3TR_C_CONFIG_COMMON(inst)				\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LSM6DS3TR_C_CONFIG_I2C(inst)					\
	{								\
		STMEMSC_CTX_I2C(&lsm6ds3tr_c_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		LSM6DS3TR_C_CONFIG_COMMON(inst)				\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LSM6DS3TR_C_DEFINE(inst)						\
	static struct lsm6ds3tr_c_data lsm6ds3tr_c_data_##inst;			\
	static const struct lsm6ds3tr_c_config lsm6ds3tr_c_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			(LSM6DS3TR_C_CONFIG_SPI(inst)),			\
			(LSM6DS3TR_C_CONFIG_I2C(inst)));			\
	LSM6DS3TR_C_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(LSM6DS3TR_C_DEFINE)
