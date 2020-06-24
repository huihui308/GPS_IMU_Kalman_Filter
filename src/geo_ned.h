//
//  geo_ned.hpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#ifndef geo_ned_hpp
#define geo_ned_hpp

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


namespace geodectic_converter {


class GeodecticConverter {
public:

    GeodecticConverter();

    bool isInitialised();

    void getReference(
        double& latitude,
        double& longitude,
        double& altitude);

    void intializeReference(
        const double latitude,
        const double longitude,
        const double altitude);

    void geodetic2Ecef(
        const double latitude,
        const double longitude,
        const double altitude,
        double* x,
        double* y,
        double* z);

    void ecef2Geodetic(
        const double x,
        const double y,
        const double z,
        double* latitude,
        double* longitude,
        double* altitude);

    void ecef2Ned(
        const double x,
        const double y,
        const double z,
        double* north,
        double* east,
        double* down);

    void ned2Ecef(
        const double north,
        const double east,
        const double down,
        double* x,
        double* y,
        double* z);

    void geodetic2Ned(
        const double latitude,
        const double longitude,
        const double altitude,
        double* north,
        double* east,
        double* down);

    void ned2Geodetic(
        const double north,
        const double east,
        const double down,
        double* latitude,
        double* longitude,
        double* altitude);

    void geodetic2Enu(
        const double latitude,
        const double longitude,
        const double altitude,
        double* east,
        double* north,
        double* up);

    void enu2Geodetic(
        const double east,
        const double north,
        const double up,
        double* latitude,
        double* longitude,
        double* altitude);

private:
    bool m_have_reference;
    double m_initial_latitude;
    double m_initial_longitude;
    double m_initial_altitude;

    double m_initial_ecef_x;
    double m_initial_ecef_y;
    double m_initial_ecef_z;

    Eigen::Matrix3d ned_to_ecef_matrix_;
    Eigen::Matrix3d ecef_to_ned_matrix_;

    double rad2Deg(const double radians);

    double deg2Rad(const double degrees);

    Eigen::Matrix3d nRe(
        const double lat_radians,
        const double lon_radians);

};

};

#endif /* geo_ned_hpp */
