
target_include_directories(app PRIVATE ../imu ../include ../datalog)

target_sources(app PRIVATE AHRS.cpp)
target_sources(app PRIVATE MadgwickAHRS.cpp)
target_sources(app PRIVATE MahonyAHRS.cpp)
target_sources(app PRIVATE SimpleAHRS.cpp)

