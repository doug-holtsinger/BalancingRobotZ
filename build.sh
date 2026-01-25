#!/bin/bash

# set -x

west build --pristine --shield adafruit_lsm6ds3_lis3mdl -b nrf52840dongle app
#west build --pristine --shield adafruit_lsm6ds3_lis3mdl -b nrf52840dongle app  -- -DCONFIG_COMPILER_SAVE_TEMPS=y

# Find the SEGGER RTT block for use with the J-Link RTT Viewer
if [[ $? -eq 0 ]] ; then
    grep _SEGGER_RTT build/app/zephyr/zephyr_final.map
fi

