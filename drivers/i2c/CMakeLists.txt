# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/i2c.h)

zephyr_library()

# zephyr_library_sources(i2c_common.c)

zephyr_library_sources(
	i2c_nrfx_twi.c
	i2c_nrfx_twi_common.c
	)

zephyr_library_sources_ifdef(CONFIG_I2C_NRFX_TWIM	i2c_nrfx_twim.c)

zephyr_library_sources_ifdef(CONFIG_I2C_TEST		i2c_test.c)

zephyr_library_sources_ifdef(CONFIG_USERSPACE		i2c_handlers.c)

