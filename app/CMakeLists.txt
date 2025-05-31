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

# Enable debug logging in the NRFX PWM HAL (modules/hal/nordic/nrfx/drivers/src)
zephyr_compile_definitions(NRFX_PWM_CONFIG_LOG_ENABLED=1)
zephyr_compile_definitions(NRFX_PWM_CONFIG_LOG_LEVEL=4)

#target_sources(app PRIVATE src/main.c)
#target_sources(app PRIVATE src/bt_nus.c)

add_subdirectory(src)

#target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/fs/nvs)

