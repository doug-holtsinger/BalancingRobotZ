cmake_minimum_required(VERSION 3.15)

target_include_directories(app PRIVATE imu)

target_sources(app PRIVATE main.cpp)

add_subdirectory(bt)
add_subdirectory(ahrs)
add_subdirectory(imu)

