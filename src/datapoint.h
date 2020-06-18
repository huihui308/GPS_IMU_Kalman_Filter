/************************************************************************
    > File Name: src/datapoint.h
    > Author: david
    > Mail: 305573571@qq.com
    > Created Time: 2020年06月18日 星期四 11时58分02秒
 ************************************************************************/
#ifndef _DATAPOINT_
#define _DATAPOINT_

#include <stdio.h>
#include <iostream>
#include "geo_ned.hpp"
#include "Eigen/Dense"


using namespace geodectic_converter;


// Input data format assumed as - latitude, longitude, vel, psi_dot, accel, alt

enum class DataPointType {
    IMU, GPS
};

/**
 * @brief Data interface for getting fusion actionable data from raw sensor measurements
 */
class DataPoint {
public:
    /**
     * @brief Default constructor
     */
    DataPoint(bool verbose);

    /**
     * @brief Retrieves raw sensor data and stores it in private variables
     *
     * @param timestamp Current timestamp for the sensor data
     * @param data_type Data type: Either GPS: which includes GPS+IMU data or IMU: which only includes the IMU data
     * @param raw_data Raw sensor data
     */
     // TODO remove data_type as it is not used anymore
    void set(
        const long long timestamp,
        const DataPointType data_type,
        const Eigen::VectorXd& raw_data);

    /**
     * @brief Returns saved raw data for sensor fusion
     *
     * @return Sensor data measurements
     */
    Eigen::VectorXd get_state() const;

    /**
     * @brief Get raw sensor data
     *
     * @return Raw sensor data
     */
    Eigen::VectorXd get_raw_data() const;

    /**
     * @brief Get data type associated with the data at current timestep
     *
     * @return Data type: Either GPS or IMU
     */
    DataPointType get_data_point_type() const;

    /**
     * @brief Get current timestamp
     *
     * @return Timestamp associated with current data
     */
    long long get_timestamp() const;

private:
    double _x;          // lat
    double _y;          // lon
    double _z;          // vel
    double _north;      // yaw_rate
    double _east;       // long_accel
    double _down;       // alt
    GeodecticConverter GPSDataConverter;
    bool _initialized;
    bool _first_data_point;
    long long _timestamp;
    Eigen::VectorXd _raw_data;
    DataPointType _data_type;

    double _prev_lat;
    double _prev_long;
    double _curr_lat;
    double _curr_long;
    double _dx;
    double _dy;
    double _mx;
    double _my;
    double _ds;

    double _RadiusEarth;
    double _arc;
    bool verbose;
};


#endif /* _DATAPOINT_ */
