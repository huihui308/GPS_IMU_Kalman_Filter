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


//Input data format assumed as - latitude, longitude, vel, psi_dot, accel, alt

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
    DataPoint(bool verbose = false)
    :_initialized(false), _first_data_point(true)
    {
        _dx = 0;
        _dy = 0;
        _mx = 0;
        _my = 0;
        _ds = 0;

        _RadiusEarth = 6378388.0; //m
        _arc = 2.0 * M_PI * (_RadiusEarth + 230)/360.0; // degree
       this->verbose = verbose;
       if(this->verbose) std::cout << "     DATAPOINT: ----- Initialized.....\r\n";
    }

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
    Eigen::VectorXd get_state() const
    {
        Eigen::VectorXd state(6);

        // Convert raw data to fusion readable states
        double x = _mx;
        double y = _my;
        double vel = _raw_data(2);
        double psi = 0;
        double psi_dot = _raw_data(3);
        double a = _raw_data(4);

        state << x, y, psi, vel, psi_dot, a;

        return state;
    }

    /**
     * @brief Get raw sensor data
     *
     * @return Raw sensor data
     */
    Eigen::VectorXd get_raw_data() const
    {
        return _raw_data;
    }

    /**
     * @brief Get data type associated with the data at current timestep
     *
     * @return Data type: Either GPS or IMU
     */
    DataPointType get_data_point_type() const
    {
        return _data_type;
    }

    /**
     * @brief Get current timestamp
     *
     * @return Timestamp associated with current data
     */
    long long get_timestamp() const
    {
        return _timestamp;
    }

private:
    double _x;
    double _y;
    double _z;
    double _north;
    double _east;
    double _down;
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
