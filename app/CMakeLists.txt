#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Controls logging in the NRFX PWM HAL (modules/hal/nordic/nrfx/drivers/src)
zephyr_compile_definitions(NRFX_PWM_CONFIG_LOG_ENABLED=0)
#zephyr_compile_definitions(NRFX_PWM_CONFIG_LOG_LEVEL=4)
# Controls logging in the NRFX QDEC HAL (modules/hal/nordic/nrfx/drivers/src)
#zephyr_compile_definitions(NRFX_QDEC_CONFIG_LOG_ENABLED=1)
#zephyr_compile_definitions(NRFX_QDEC_CONFIG_LOG_LEVEL=4)

# workaround for out-of-tree QDEC driver which ultimately relies on
# the existence of the define in this included file.
target_compile_options(drivers__pinctrl PRIVATE
        "-include"
	"$ENV{ZEPHYR_BASE}/../BalancingRobotZ/drivers/sensor/qdec_nrfx/qdec_pinctrl.h"
    )

add_subdirectory(src)

#target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/fs/nvs)

