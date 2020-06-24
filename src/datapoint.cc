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
    :m_initialized(false), m_first_data_point(true)
{
    m_dx = 0;
    m_dy = 0;
    m_mx = 0;
    m_my = 0;
    m_ds = 0;

    m_RadiusEarth = 6378388.0; //m
    m_arc = 2.0 * M_PI * (m_RadiusEarth + 230)/360.0; // degree
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
    if ( this->verbose ) {
        std::cout << "        DATAPOINT: ----- In set\r\n";
    }
    m_raw_data.resize(raw_data.size());
    m_timestamp = timestamp;
    m_raw_data = raw_data;
    m_initialized = true;

    if ((m_first_data_point && raw_data(0) != 0.0) && (raw_data(1) != 0.0)) {
        m_dx = 0;
        m_dy = 0;
        m_mx = 0;
        m_my = 0;
        m_prev_lat = raw_data(0);
        m_prev_long = raw_data(1);
        m_arc = 2.0 * M_PI * (m_RadiusEarth + raw_data(5))/360.0;
        m_first_data_point = false;
    } else if ( !m_first_data_point ) {
        m_arc = 2.0 * M_PI * (m_RadiusEarth + raw_data(5))/360.0;
        m_dx = m_arc * cos(raw_data(0) * M_PI/180.0) * (raw_data(1) - m_prev_long);
        m_dy = m_arc * (raw_data(0) - m_prev_lat);
        m_ds = sqrt(m_dx * m_dx + m_dy * m_dy);

        if (m_ds == 0.0) {
            m_data_type = DataPointType::IMU;
        } else {
            m_data_type = DataPointType::GPS;
        }

        m_mx += m_dx; // cumulative sum
        m_my += m_dy; // cumulative sum
        if ( this->verbose ) {
            std::cout << " Mx, My: " << m_mx << ", " << m_my << std::endl;
        }
        m_prev_lat = raw_data(0);
        m_prev_long = raw_data(1);
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
    double x = m_mx;
    double y = m_my;
    double vel = m_raw_data(2);
    double psi = 0;
    double psi_dot = m_raw_data(3);
    double a = m_raw_data(4);

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
    return m_raw_data;
}


/**
 * @brief Get data type associated with the data at current timestep
 *
 * @return Data type: Either GPS or IMU
 */
DataPointType DataPoint::get_data_point_type() const
{
    return m_data_type;
}

/**
 * @brief Get current timestamp
 *
 * @return Timestamp associated with current data
 */
long long DataPoint::get_timestamp() const
{
    return m_timestamp;
}


