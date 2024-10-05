# ST Microelectronics LSM6DSO 6-axis IMU sensor driver
#
# Copyright (c) 2019 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(lsm6ds3tr-c.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/sensor/st/stmemsc)
