cmake_minimum_required(VERSION 3.5)
set( prjName gpsImuEkf )

add_definitions(-std=c++0x)


###### change the default location of executables
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


include_directories(headers)

set(SOURCES src/ekf.cc
            src/utils.cc
            src/fusion.cc
            src/geo_ned.cc
            src/datapoint.cc
            src/run_fusion.cc
            src/parameterReader.cc
            src/main.cc)

add_executable(${prjName} ${SOURCES})


target_link_libraries(${prjName} libglog.a)
target_link_libraries(${prjName} pthread)


