# BalancingRobot
Balancing Robot

Commands to clone repository:

git clone git@github.com:doug-holtsinger/BalancingRobot.git --recurse-submodules
git submodule update --init
git submodule foreach -q --recursive 'git checkout $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master)'
git config --global status.submoduleSummary true



# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840 Dongle and a Raspberry Pi computer.  The Nordic chip is attached to an accelerometer, magnetometer and gyroscope, which is used for an AHRS (Attitude and Heading Reference System).  The output from the AHRS is sent over Bluetooth Low Energy (BLE) from the Nordic to the Raspberry Pi where the object orientation is visualized using the Raspberry Pi graphics hardware, and control of the AHRS parameter settings can be interactively changed using a Python-based GUI.

# Current State of the Project

The project is under active development.  The AHRS calibration routines are still very primitive and so the calculated Euler angles are not terribly accurate at this time.  With improvements to the calibration routines I expect much better accuracy.  

The Raspberry Pi3B is able to receive the Euler angles from the Nordic chip over Bluetooth LE, and to visualize the orientation using the Pi graphics by displaying a primitive 3D cube.  The response time is very good.  The calibration and control panel GUI on the Raspberry Pi operates correctly but still needs a lot of improvement.

The IMU is not currently available from Adafruit, however I am looking into adding support for a more readily available IMU with an I2C interface that would have the same or better performance and functionality.

# Hardware Requirements

1. [Nordic Semiconductor nRF52840 Dongle](https://www.mouser.com/ProductDetail/949-NRF52840-DONGLE)
2. [Adafruit 4485 -- LSM6DS33 + LIS3MDL - 9 DoF IMU with Accel / Gyro / Mag](https://www.adafruit.com/product/4485) or equivalent
3. Raspberry Pi3B (may work with other Pis)
4. Personal computer, Ubuntu Linux or Windows running WSL, with USB-A Connector
5. Optional: [Adafruit #3309 - CP2104 Friend](https://www.adafruit.com/product/3309) - USB to Serial Converter or equivalent, for Serial Console to Nordic chip.
6. Optional: liPo Battery for Nordic chip

## Hardware Circuit Diagram

# Software Requirements

1. [Nordic Semiconductor nRF52 SDK v17.1.0 or later](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download)
Select s140 Softdevice support.
2. [Nordic recommended Development IDE](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_nrf52840_dk%2FUG%2Fcommon%2Fnordic_tools.html).  One of the following:
   - [ARM GNU/GCC](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (Recommended for this project, version 9.2 or later).
   - [SEGGER Embedded Studio (SES)](https://www.segger.com/products/development-tools/embedded-studio/)
   - [MDK-ARM Keil µVision](https://www2.keil.com/mdk5/uvision/)
   - [IAR](https://www.iar.com/iar-embedded-workbench/#!?architecture=ARM)
3. Python3
4. [BlueZ](http://www.bluez.org/) v5.47 or later
5. [Bluepy](https://github.com/IanHarvey/bluepy/tree/v/1.3.0) v1.3 or later

The Software development platform I've used for the Nordic Semiconductor Board is an x86 Windows PC running Windows WSL (Windows Subsystem for Linux).  

I have also used [Cygwin](https://www.cygwin.com/) which is a software layer which runs on top of Windows 10/11 and provides functionality similar to a Linux distribution.  You can compile the Nordic Semiconductor source code once you have installed the GNU GCC toolchain (using Cygwin's package manager). It's a very simple way to run Windows and Linux applications on the same box.

## Software Installation Steps

Assumes Ubuntu TLS 22.04.04 LTS running under Windows WSL.

## GCC Installation

sudo apt-get install gcc-arm-none-eabi

## Nordic SDK Installation

In the Nordic nRF52 SDK, edit the file 
components/toolchain/gcc/Makefile.<platform> 
and change the following Makefile variables to point to your installation 
and version of the ARM gcc compiler (Linux example shown, Makefile.posix).

GNU_INSTALL_ROOT ?= /usr/bin/
GNU_VERSION ?= 10.3.1
GNU_PREFIX ?= arm-none-eabi

Edit the location of the Nordic SDK in the WirelessSensor/Sensor/src/Makefile:

SDK_ROOT := /home/<user>/NordicSDK/nRF5_SDK_17.1.0_ddde560

## Nordic Software Compilation and Download

1. cd into the Directory Sensor/src
2. Type 'make'
3. Install Nordic firmware onto board using nRF Connect for Desktop via USB or SEGGER debugger

## Software Installation for AHRS Console GUI

This installation covers Ubuntu 22.04.04 LTS
Assumes python3.10 is already installed

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
AttributeError: 'EGLPlatform' object has no attribute 'GLX'. Did you mean: 'GL'?

unset WAYLAND_DISPLAY
or
pip3 install --force-reinstall "PyOpenGL==3.1.5"

# to get around problem:
#  https://github.com/IanHarvey/bluepy/issues/218
sudo setcap 'cap_net_raw,cap_net_admin+eip' bluepy-helper

#sudo apt install libgles2
#sudo apt install libgles2-mesa
#sudo apt install libegl-dev


# Software Architecture

# Operation of the AHRS Device

## Calibration and Control GUI on the Raspberry Pi

## Optional Serial Console

## Calibration Procedure

## 3D Orientation Visualization on the Raspberry Pi

# Remaining Improvements:
1. Refine AHRS calibration
   - Improve calibration algorithm for accelerometer
   - remove fixed constants
   - require less operator intervention to do calibration
   - Angles take a while to stabilize
   - use magnetometer to calibrate gyroscope
   - add support for setting hardware register configuration bits
2. Fix singularity at North and South Poles, can't get to 90 degree pitch (gimbal lock)
3. Fix Pitch sign reversal
4. Further testing
5. Nordic Timer to drive AHRS to improve accuracy?  sampleFreq needs to be set right.
6. Re-organize all code
   - Place devi2c better, why does not work in constructor
7. BLE code
   - DFU Over the Air Updates
   - cleanup BLE code to remove junk
   - Remove / replace HRS and other remnants
   - Improve performance, being aware of battery life
8. Battery Power
9. Prototype Board 
10. Calibration Storage, so you don't have to do calibration every time you power on
components/libraries/fds/fds.h
11. Add Doyxgen documentation for all code, procedures, and parameters
12. Add circuit schematic
13. Add general documentation on the project, algorithms, calibration
14. Develop Mechanical Test fixture to hold board and measure actual orientation to compare against calculated orientation.
15. Improve Calibration and Control GUI
16. Look at adding support for Visual Studio Code
17. On Server side, look at adding support for Arduino
18. On Client side, look at adding Android support
