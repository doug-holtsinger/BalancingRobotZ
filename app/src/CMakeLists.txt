cmake_minimum_required(VERSION 3.15)

target_include_directories(app PRIVATE imu)
target_include_directories(app PRIVATE bt)
target_include_directories(app PRIVATE qdec)
target_include_directories(app PRIVATE app_demux)
target_include_directories(app PRIVATE param_store)
target_include_directories(app PRIVATE motor)

target_sources(app PRIVATE main.cpp)

add_subdirectory(bt)
add_subdirectory(ahrs)
add_subdirectory(imu)
add_subdirectory(qdec)
add_subdirectory(app_demux)
add_subdirectory(param_store)
add_subdirectory(motor)

