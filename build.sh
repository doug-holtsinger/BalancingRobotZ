#!/bin/bash

set -x

#west build -p always -b nrf52840_custom app

#west build -p always --shield adafruit_lsm6ds3_lis3mdl -b nrf52840dongle app
west build --pristine --shield adafruit_lsm6ds3_lis3mdl -b nrf52840dongle app

# Find the SEGGER RTT block for use with the J-Link RTT Viewer
grep _SEGGER_RTT build/zephyr/zephyr_final.map

