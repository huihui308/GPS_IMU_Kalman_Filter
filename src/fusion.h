//
//  fusion.hpp
//  Fusion
//
//  Created by Karan on 4/9/18.
//  Copyright � 2018 Karan. All rights reserved.
//
#ifndef fusion_hpp
#define fusion_hpp

#include <stdio.h>
#include <iostream>
#include "ekf.h"
#include "geo_ned.h"
#include "Eigen/Dense"
#include "datapoint.h"


using namespace geodectic_converter;


/**
 * @brief Class which uses EKF and DataPoint to run the filter
 */
class Fusion {
private:
    const int32_t m_n = 6;
    bool m_initialized;
    Eigen::MatrixXd m_P;
    Eigen::MatrixXd m_Q;
    Eigen::MatrixXd m_F;
    Eigen::MatrixXd m_R;
    EKF m_KF;

    double m_sGPS;
    double m_sCourse;
    double m_sVelocity;
    double m_sYaw;
    double m_sAccel;
    double m_dt;
    double m_max_turn_rate;
    double m_max_acceleration;
    double m_max_yaw_accel;
    long long m_timestamp;
    // GPS offsets
    double m_xOffset, m_yOffset;
    bool verbose;

public:
    /**
     * @brief Constructor
     *
     * @param max_acceleration System parameter specifying maximum acceleration
     * @param max_turn_rate System parameter specifying maximum turn rate
     * @param max_yaw_accel System parameter specifying max yaw acceleration
     */
    Fusion(
        double max_acceleration,
        double max_turn_rate,
        double max_yaw_accel,
        double varGPS,
        double varSpeed,
        double varYaw,
        double varAcc,
        double xOffset,
        double yOffset,
        bool verbose);

    /**
     * @brief Updates the Q matrix
     *
     * @param dt Timestep
     */
    void const updateQ(double dt);

    /**
     * @brief Starts the filter
     *
     * @param data Current sensor data
     */
    void start(const DataPoint& data);

    /**
     * @brief Performs the prediction and update step for the filter based on the datatype of the input data
     *
     * @param data Current sensor data
     */
    void compute(const DataPoint& data);

    /**
     * @brief Looping function to either start or run the filter in a loop
     *
     * @param data Current sensor data
     */
    void process(const DataPoint& data);

    /**
     * @brief Returns the estimated state of the system
     *
     * @return Vector containing the estimated states
     */
    Eigen::VectorXd get_resulting_state() const;
};

#endif /* fusion_hpp */
