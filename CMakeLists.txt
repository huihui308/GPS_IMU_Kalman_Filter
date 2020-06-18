cmake_minimum_required(VERSION 3.5)
set( prjName gpsImuEkf )

add_definitions(-std=c++0x)


###### change the default location of executables
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


include_directories(headers)

set(SOURCES src/ekf.cpp
            src/fusion.cpp
            src/geo_ned.cpp
            src/run_fusion.cpp
            src/utils.cpp
            src/main.cpp)

add_executable(${prjName} ${SOURCES})
