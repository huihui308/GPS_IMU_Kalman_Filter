//
//  ekf.cpp
//  EKF
//
//  Created by Karan on 4/7/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#include <iostream>
#include "ekf.h"


void EKF::start(
    const int nin,
    const Eigen::VectorXd& xin,
    const Eigen::MatrixXd& Pin,
    const Eigen::MatrixXd& Fin,
    const Eigen::MatrixXd& Qin)
{
    m_num_states = nin;
    m_I = Eigen::MatrixXd::Identity(m_num_states, m_num_states);
    if ( this->verbose ) {
        std::cout << "    EKF: Number of states ->" << nin << "\n";
    }
    this->m_state.resize(nin);
    this->m_state = xin;
    if ( this->verbose ) {
        std::cout << "    EKF: Size of Input states ->" << xin.size() << "\n";
    }
    m_P = Pin;
    m_Fj = Fin;
    m_Q = Qin;

    return;
}

void EKF::setQ(const Eigen::MatrixXd& Q_in)
{
    m_Q = Q_in;
}

Eigen::MatrixXd calculate_joacobian(
    const Eigen::VectorXd& v,
    const double dt)
{
    // Assumes Jacobian is 6 x 6
    Eigen::MatrixXd JA = Eigen::MatrixXd::Zero(6, 6);
    // Assumes the size of input vector is 6
    const double psi = v(2);
    const double velocity = v(3);
    const double psi_dot = v(4);
    const double THRESHOLD = 0.001;

    // Avoid dividing by zero
    if (THRESHOLD > psi_dot) {
        return JA;
    }
    //------
    const double turn_radius = (velocity/psi_dot);
    const double psi_dot_inverse = 1/psi_dot;
    const double pdotp = dt * psi_dot + psi;

    const double a13 = turn_radius * (cos(dt * psi_dot + psi) - cos(psi));
    const double a14 = psi_dot_inverse * (sin(psi_dot*dt + psi) - sin(psi));
    const double a15 = dt * turn_radius * cos(pdotp) - (velocity/(pow(psi_dot, 2)))*(sin(pdotp) -sin(psi));

    const double a23 = turn_radius * (-sin(psi) + sin(pdotp));
    const double a24 = psi_dot_inverse * (cos(psi) - cos(pdotp));
    const double a25 = dt * turn_radius * sin(pdotp) - (velocity/(pow(psi_dot, 2))) * (cos(psi) - cos(pdotp));

    JA <<
        1.0, 0.0, a13, a14, a15, 0.0,
        0.0, 1.0, a23, a24, a25, 0.0,
        0.0, 0.0, 1.0, 0.0, dt,  0.0,
        0.0, 0.0, 0.0, 1.0, 0.0,  dt,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0 ;

    return JA;
}

/*******************************************
 * State Equation Update Rule
    x + v/ψ˙(−sin(ψ) + sin(dtψ˙+ψ))
    y + v/ψ˙(cos(ψ) − cos(dtψ˙+ψ))
    dtψ˙+ ψ
    dta + vψ˙
    ψ˙
    a
 *******************************************/
void EKF::updateFj(const double dt)
{
    if ( this->verbose ) {
        std::cout << "Updating Fj: About to update state equations" << "\n";
    }
    if ( this->verbose ) {
        std::cout << "Updating Fj: size of states" << this->m_state.rows() << "x" << this->m_state.cols() << "\n";
    }
    // Updating state equations
    if (0.0001 > fabs(m_state(4))) {
        // Driving straight
        m_state(0) = m_state(0) + (m_state(3) * dt) * cos(m_state(2));
        m_state(1) = m_state(1) + (m_state(3) * dt) * sin(m_state(2));
        m_state(2) = m_state(2);
        m_state(3) = m_state(3) + m_state(5) * dt;
        m_state(4) = 0.0000001;     // avoid numerical issues in Jacobians
        m_state(5) = m_state(5);
    } else {
        // otherwise
        m_state(0) = m_state(0) + (m_state(3)/m_state(4)) * (sin(m_state(4) * dt + m_state(2)) - sin(m_state(2)));
        m_state(1) = m_state(1) + (m_state(3)/m_state(4)) * (-cos(m_state(4) * dt + m_state(2)) + cos(m_state(2)));
        m_state(2) = std::fmod((m_state(2) + m_state(4) * dt + M_PI), (2.0 * M_PI)) - M_PI;
        m_state(3) = m_state(3) + m_state(5) * dt;
        m_state(4) = m_state(4);    // Constant Turn Rate
        m_state(5) = m_state(5);    // Constant Acceleration
    }

    if (this->verbose) {
        std::cout << "Updating Fj: About to calculate jacobian" << "\n";
    }
    // Calculate jacobian
    //m_Fj =  calculate_jacobian(m_state, dt);
    m_Fj = calculate_joacobian(m_state, dt);
}

// Prediction step
void EKF::predict()
{
    m_P = m_Fj * m_P * m_Fj.transpose() + m_Q;
}

void EKF::update(
    const Eigen::VectorXd& Z,
    const Eigen::VectorXd& z_mean,
    const Eigen::MatrixXd& Hj,
    const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd Hj_T = m_P * Hj.transpose();
    // Temporary variable for storing this intermediate value
    Eigen::MatrixXd m_S = Hj * Hj_T + R;
    // Compute the Kalman gain
    m_K = Hj_T * m_S.inverse();
    // Update the estimate
    Eigen::VectorXd y = Z - z_mean;

    m_state = m_state + m_K * y;
    // Update the error covariance
    m_P = (m_I - m_K * Hj) * m_P;
}

Eigen::VectorXd EKF::get_resulting_state() const
{
    return m_state;
}
