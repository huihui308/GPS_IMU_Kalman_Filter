//
//  ekf.hpp
//  EKF
//
//  Created by Karan on 4/7/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#ifndef ekf_hpp
#define ekf_hpp

#include <stdio.h>
#include "utils.h"
#include "Eigen/Dense"


#define GPS_VECTOR_SIZE                     (4)
#define IMU_VECTOR_SIZE                     (2)


/**
 * @brief EKF base class implementing generic Extended Kalman Filter
 */
class EKF {

public:
    /**
     * @brief Default constructor
     */
    EKF(bool verbose = false){this->verbose = verbose;};

    /**
     * @brief Default destructor
     */
    ~EKF(){};

    /**
     * @brief Function to be called the first time EKF is initialized
     *
     * @param nin Number of states
     * @param xin States
     * @param Pin Initial covariance matrix
     * @param Fin Initial Jacobian of the state
     * @param Qin Initial Q matrix
     */
    void start(
        const int32_t nin,
        const Eigen::VectorXd& xin,
        const Eigen::MatrixXd& Pin,
        const Eigen::MatrixXd& Fin,
        const Eigen::MatrixXd& Qin);

    /**
     * @brief set Q value for the filter
     *
     * @param Q_in Input Q matrix
     */
    void setQ(const Eigen::MatrixXd& Q_in);

    /**
     * @brief Returns estimated state
     *
     * @return State of the system
     */
    Eigen::VectorXd get_resulting_state() const;

    /**
     * @brief Integrates system variables to predict the system state
     *
     * @param dt Time interval over which the integration takes place. Usually the difference between the previous and
     * current time step
     */
    void updateFj(const double dt);

    /**
     * @brief Updates the state covairance matrix and adds process noise
     */
    void predict();

    /**
     * @brief Runs the correction/update step of the filter
     *
     * @param Z Measurements for the current time step
     * @param Hx Measurement model
     * @param JH Jacobian of the measurment model
     * @param R Measurement noise
     */
    void update(
        const Eigen::VectorXd& Z,
        const Eigen::VectorXd& Hx,
        const Eigen::MatrixXd& JH,
        const Eigen::MatrixXd& R);

private:
    // Flag to indicate if the filter has started
    bool _init;
    bool verbose;
    int32_t m_num_states;   // Number of states in the EKF

    Eigen::MatrixXd m_P;    // initial covaraince/uncertainity in states
    Eigen::MatrixXd m_Q;    // process noise covariance
    Eigen::MatrixXd _JH;    // measurment jacobian
    Eigen::MatrixXd _R;     // measurement noise covariance
    Eigen::MatrixXd m_I;    // Identity matrix
    Eigen::MatrixXd m_Fj;   // Jacobian state matrix
    Eigen::MatrixXd m_S;    // Matrix for storing intermediate step in update part
    Eigen::MatrixXd m_K;    // Kalman Gain
    Eigen::VectorXd m_state;// State - x y heading velocity yaw_rat long_acceleration
};

#endif /* ekf_hpp */
