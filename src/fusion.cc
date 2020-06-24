//
//  fusion.cpp
//  Fusion
//
//  Created by Karan on 4/9/18.
//  Copyright � 2018 Karan. All rights reserved.
//
#include <iostream>
#include "fusion.h"
#include "datapoint.h"


Fusion::Fusion(
    double max_acceleration,
    double max_turn_rate,
    double max_yaw_accel,
    double varGPS,
    double varSpeed,
    double varYaw,
    double varAcc,
    double xOffset,
    double yOffset,
    bool verbose)
: m_initialized(false), m_max_turn_rate(max_turn_rate), m_max_acceleration(max_acceleration), m_max_yaw_accel(max_yaw_accel), m_xOffset(xOffset),
  m_yOffset(yOffset), m_KF(verbose)
{
    // Initialize initial uncertainity P0
    m_P = Eigen::MatrixXd(m_n, m_n);
    m_P <<
        1000.0, 0.0,    0.0,    0.0,    0.0,    0.0,
        0.0,    1000.0, 0.0,    0.0,    0.0,    0.0,
        0.0,    0.0,    1000.0, 0.0,    0.0,    0.0,
        0.0,    0.0,    0.0,    1000.0, 0.0,    0.0,
        0.0,    0.0,    0.0,    0.0,    1000.0, 0.0,
        0.0,    0.0,    0.0,    0.0,    0.0,    1000.0;

    // Assuming 5 sources of measurement
    m_R = Eigen::MatrixXd(5, 5);
    double a11 = pow(varGPS, 2);
    double a22 = pow(varGPS, 2);
    double a33 = pow(varSpeed, 2);
    double a44 = pow(varYaw, 2);
    double a55 = pow(varAcc, 2);
    m_R <<
        a11, 0.0, 0.0, 0.0, 0.0,
        0.0, a22, 0.0, 0.0, 0.0,
        0.0, 0.0, a33, 0.0, 0.0,
        0.0, 0.0, 0.0, a44, 0.0,
        0.0, 0.0, 0.0, 0.0, a55;

    this->verbose = verbose;
    if ( verbose ) {
        std::cout << " =========================== FUSION:  Initializing --- " << "\r\n";
    }
}

void const Fusion::updateQ(double dt)
{
    if ( this->verbose ) {
        std::cout << " =========================== FUSION:  Updating Q --- " << "\r\n";
    }
    // Process Noise Covariance Matrix Q
    m_Q = Eigen::MatrixXd(m_n, m_n);
    m_sGPS = 0.5 * m_max_acceleration * pow(dt, 2);
    m_sVelocity = m_max_acceleration * dt;
    m_sCourse = m_max_turn_rate * dt;
    m_sYaw = m_max_yaw_accel * dt;
    m_sAccel = m_max_acceleration;

    double a11 = pow(m_sGPS, 2);
    double a22 = pow(m_sGPS, 2);
    double a33 = pow(m_sCourse, 2);
    double a44 = pow(m_sVelocity, 2);
    double a55 = pow(m_sYaw, 2);
    double a66 = pow(m_sAccel, 2);
    m_Q <<
        a11, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, a22, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, a33, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, a44, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, a55, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, a66;

    m_KF.setQ(m_Q);
}

void Fusion::start(const DataPoint& aiqData)
{
    if ( this->verbose ) {
        std::cout << "    Fusion: ------ In start.....\r\n";
    }
    m_timestamp = aiqData.get_timestamp();
    Eigen::VectorXd state = aiqData.get_state();
    m_KF.start(m_n, state, m_P, m_F, m_Q);
    m_initialized = true;
}

void Fusion::compute(const DataPoint& aiqData)
{
    /*******************************************
     * Prediction Step
     - Assumes current velocity is the same for this dt
     *******************************************/
    if ( this->verbose ) {
        std::cout << "    Fusion: ------ In compute.....\r\n";
    }
    // Assuming 1.e6 for timestamp - confirm after running on the system
    const double dt = (aiqData.get_timestamp())/ 1.e6;
    // const double dt = 0.1;
    m_timestamp = aiqData.get_timestamp();
    // Update Q
    this->updateQ(dt);
    // Update state and calculate jacobian
    m_KF.updateFj(dt);
    // Prediction
    m_KF.predict();
    /*******************************************
     * Update Step
     - Updates appropriate matrices given a measurement
     - Assumes measurement is received either from GPS or IMU
     *******************************************/
    Eigen::VectorXd zz = aiqData.get_state();
    Eigen::VectorXd z;
    z.resize(5);
    z <<
        zz(0), // px
        zz(1), // py
        zz(3), // vel
        zz(4), // yaw_rate
        zz(5); // accel

    const Eigen::VectorXd state = m_KF.get_resulting_state();

    Eigen::VectorXd Hx;
    Eigen::MatrixXd Hj;

    Hx.resize(5);
    Hj.resize(5, 6);

    // measurement function
#if 0
    Hx <<
        state(0) + m_xOffset * cos(state(3)) - m_yOffset * sin(state(3)),
        state(1) + m_xOffset * sin(state(3)) + m_yOffset * cos(state(3)),
        state(3),
        state(4),
        state(5);
    double j13 = -m_xOffset * sin(state(3)) - m_yOffset * cos(state(3));
    double j23 = m_xOffset * cos(state(3)) - m_yOffset * sin(state(3));
#else
    Hx <<
        state(0),
        state(1),
        state(3),
        state(4),
        state(5);
    double j13 = 0.0;
    double j23 = 0.0;
#endif
    if (aiqData.get_data_point_type() == DataPointType::GPS) {
        Hj <<  1.0, 0.0, j13, 0.0, 0.0, 0.0,
               0.0, 1.0, j23, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        m_KF.update(z, Hx, Hj, m_R);
    } else if (aiqData.get_data_point_type() == DataPointType::IMU) {
        Hj <<  0.0, 0.0, j13, 0.0, 0.0, 0.0,
               0.0, 0.0, j23, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        m_KF.update(z, Hx, Hj, m_R);
    }
}

void Fusion::process(const DataPoint& aiqData)
{
    if ( this->verbose ) {
        std::cout << "    Fusion: ------ In process.....\r\n";
    }
    if (0.0 < aiqData.get_timestamp()) {
        m_initialized ? this->compute(aiqData) : this->start(aiqData);
    }
}

Eigen::VectorXd Fusion::get_resulting_state() const
{
    return m_KF.get_resulting_state();
}
