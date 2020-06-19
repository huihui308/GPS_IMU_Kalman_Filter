/************************************************************************
    > File Name: src/datapoint.cc
    > Author: david
    > Mail: 305573571@qq.com
    > Created Time: 2020年06月18日 星期四 11时57分58秒
 ************************************************************************/
#include "datapoint.h"


/**
 * @brief Default constructor
 */
DataPoint::DataPoint(bool verbose = false)
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
    if ( this->verbose ) {
        std::cout << "     DATAPOINT: ----- Initialized.....\r\n";
    }
}

/**
 * @brief Retrieves raw sensor data and stores it in private variables
 *
 * @param timestamp Current timestamp for the sensor data
 * @param data_type Data type: Either GPS: which includes GPS+IMU data or IMU: which only includes the IMU data
 * @param raw_data Raw sensor data
 */
// TODO remove data_type as it is not used anymore
void DataPoint::set(
    const long long timestamp,
    const DataPointType data_type,
    const Eigen::VectorXd& raw_data)
{
    if (this->verbose) {
        std::cout << "        DATAPOINT: ----- In set\r\n";
    }
    _raw_data.resize(raw_data.size());
    _timestamp = timestamp;
    _raw_data = raw_data;
    _initialized = true;

    if (_first_data_point && raw_data(0)!=0.0 && raw_data(1)!=0.0) {
        _dx = 0;
        _dy = 0;
        _mx = 0;
        _my = 0;
        _prev_lat = raw_data(0);
        _prev_long = raw_data(1);
        _arc = 2.0 * M_PI * (_RadiusEarth + raw_data(5))/360.0;
        _first_data_point = false;
    } else if ( !_first_data_point ) {
        _arc = 2.0 * M_PI * (_RadiusEarth + raw_data(5))/360.0;
        _dx = _arc * cos(raw_data(0) * M_PI/180.0) * (raw_data(1) - _prev_long);
        _dy = _arc * (raw_data(0) - _prev_lat);
        _ds = sqrt(_dx * _dx + _dy * _dy);

        if (_ds == 0.0) {
            _data_type = DataPointType::IMU;
        } else {
            _data_type = DataPointType::GPS;
        }

        _mx += _dx; // cumulative sum
        _my += _dy; // cumulative sum
        if (this->verbose) {
            std::cout << " Mx, My: " << _mx << ", " << _my << std::endl;
        }
        _prev_lat = raw_data(0);
        _prev_long = raw_data(1);
    }
}

/**
 * @brief Returns saved raw data for sensor fusion
 *
 * @return Sensor data measurements
 */
Eigen::VectorXd DataPoint::get_state() const
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
Eigen::VectorXd DataPoint::get_raw_data() const
{
    return _raw_data;
}


/**
 * @brief Get data type associated with the data at current timestep
 *
 * @return Data type: Either GPS or IMU
 */
DataPointType DataPoint::get_data_point_type() const
{
    return _data_type;
}

/**
 * @brief Get current timestamp
 *
 * @return Timestamp associated with current data
 */
long long DataPoint::get_timestamp() const
{
    return _timestamp;
}


