# Self-Balancing Robot using the Zephyr RTOS

## Introduction

This project implements a 2-wheeled self-balancing robot using the Zephyr RTOS.  The robot uses the Nordic Semiconductor nRF52840 microcontroller, an accelerometer/gyroscope and magnetometer, a motor driver, a single wheel encoder, and dual N20 micro-motors.  

The two motors are controlled via a cascade (dual) PID controller.  The first PID detects the wheel speed using the wheel encoder, and generates a set point which is fed into the second PID.  The second PID controls the motors via a motor driver and PWM (pulse width modulation).  The tuning of the PID constants can be performed using a BLE (Bluetooth Low Energy) connection to a separate Raspberry Pi computer.  

A python control program running on the Raspberry Pi computer can be used to tune the PID and configure other control variables, and for real-time display of the current operating conditions of the robot.  Currently the robot performs best when it does not have an active BLE connection.

The nRF52840 microcontroller performs sensor fusion in software to support an IMU, reads the wheel encoder value, operates the motor drivers, and supports the BLE connection to the Raspberry Pi computer.

Here is a video of the self-balancing robot in action:\
https://www.youtube.com/shorts/mzc14pLCrEc

[Here is a screenshot of the Python Control Program](https://github.com/doug-holtsinger/BalancingRobotZ/tree/main/doc/AHRSGui.png)

## Software Checkout and Build

The Software development and build platform is an x86 Windows PC running Windows WSL (Windows Subsystem for Linux), using Ubuntu TLS 22.04.04 LTS. 

Commands to create the Workspace on Linux:

```
mkdir -p br_app
cd br_app
west init -m https://github.com/doug-holtsinger/BalancingRobotZ
cd BalancingRobotZ
west update
```

Install the Nordic Semiconductor utility "nrfutil":\
https://www.nordicsemi.com/Products/Development-tools/nRF-Util

Install the nRF Connect for Desktop:\
https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop/download

Command to build the Zephyr application (in br_app directory):
```
source zephyr/zephyr-env.sh
nrfutil toolchain-manager launch --shell
cd BalancingRobotZ
./build.sh
```

Program the Device using the Programmer in the nRF Connect for Desktop.  I installed the Nordic Open DFU bootloader. 

https://docs.nordicsemi.com/bundle/sdk_nrf5_v17.1.0/page/ble_sdk_app_open_bootloader.html 

## Hardware Requirements

### Parts List

1. [Nordic Semiconductor nRF52840 Dongle](https://www.mouser.com/ProductDetail/949-NRF52840-DONGLE)
2. [Adafruit LSM6DS3TR-C + LIS3MDL - Precision 9 DoF IMU](https://www.adafruit.com/product/5543)
3. [Pololu DRV8835 Dual Motor Driver Carrier](https://www.pololu.com/product/2135)
4. [Adafruit N20 DC Motor with Magnetic Encoder - 6V with 1:50 Gear Ratio](https://www.adafruit.com/product/4638)
Note: I used a set of motors and encoders from this M5Stack Bala2 self-balancing robot with a 1:30 Gear Ratio:
https://shop.m5stack.com/products/bala2-fire-self-balancing-robot-kit
Only a single wheel encoder is supported on the nRF52840 microcontroller.
5. [Adafruit MiniBoost 5V @ 1A - TPS61023](https://www.adafruit.com/product/4654)
I used this 5V miniboost board to power the 6V motors from a 3.7V LiPo battery.
6. LiPo Battery, 3.7V
7. Various wires/connectors/cables.
8. Mechanical parts for robot.
9. (Optional) SEGGER J-Link Debugger for nRF52840, provides support for Serial Console and Debug

### Hardware Circuit Diagram

## Software Requirements

### Software Installation for Python Control Program on Raspberry Pi5 (Bullseye)

```
# Make python virtual environment
python -m venv ./venv
cd venv
source bin/activate
# Update OS
sudo apt update
sudo apt upgrade
# Install bluez
sudo apt-get install bluez
# Install pip for python3.10
sudo apt-get install python3-pip
# Install glib
sudo apt-get install libglib2.0-dev
# Install bluepy
pip install bluepy
# Install tkinter
sudo apt-get install python3-tk
# Install matplotlib
pip install matplotlib
# Install PyOpenGL
sudo apt install python3-opengl
pip3 install PyOpenGL
# Install pyopengltk
pip install pyopengltk
# Workaround for PyOpenGL error:
# AttributeError: 'EGLPlatform' object has no attribute 'GLX'. Did you mean: 'GL'?
unset WAYLAND_DISPLAY
# or
pip3 install --force-reinstall "PyOpenGL==3.1.5"
# To get around problem:
# https://github.com/IanHarvey/bluepy/issues/218
sudo setcap 'cap_net_raw,cap_net_admin+eip' bluepy-helper
```

### Python Control Program Installation and Invocation

```
# activate python virtual environment
cd venv
source bin/activate
```

```
# clone program into separate area
git clone https://github.com/doug-holtsinger/WirelessSensor 
cd WirelessSensor/Compute/src/AHRS-VISUAL
./ahrs-console.py
```

### Calibration and Control GUI on the Raspberry Pi

### Calibration Procedure

### 3D Orientation Visualization on the Raspberry Pi

=======
# clone program
git clone https://github.com/doug-holtsinger/WirelessSensor 
cd WirelessSensor/Compute/src/AHRS-VISUAL
./ahrs-console.py
```

### Calibration and Control GUI on the Raspberry Pi

### Calibration Procedure

### 3D Orientation Visualization on the Raspberry Pi

=======
# clone program
git clone https://github.com/doug-holtsinger/WirelessSensor 
cd WirelessSensor/Compute/src/AHRS-VISUAL
./ahrs-console.py
```

### Calibration and Control GUI on the Raspberry Pi

### Calibration Procedure

### 3D Orientation Visualization on the Raspberry Pi

## Remaining Improvements:
1. Refine AHRS calibration
   - Improve calibration algorithm for accelerometer
   - remove fixed constants
   - require less operator intervention to do calibration
   - Angles take a while to stabilize
   - add support for setting hardware register configuration bits
2. Fix singularity at North and South Poles, can't get to 90 degree pitch (gimbal lock)
3. Further testing
4. BLE code
   - DFU Over the Air Updates
   - Improve performance, being aware of battery life
5. Add Doyxgen documentation for all code, procedures, and parameters
6. Add circuit schematic
7. Add general documentation on the project, algorithms, calibration
8. Develop Mechanical Test fixture to hold board and measure actual orientation to compare against calculated orientation.
9. Improve Calibration and Control GUI
10. Look at adding support for Visual Studio Code
11. On Server side, look at adding support for Arduino
12. On Client side, look at adding Android support

